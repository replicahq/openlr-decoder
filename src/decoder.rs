use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use geo::{HaversineDistance, Point};
use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use thiserror::Error;

use crate::candidates::{find_candidates, Candidate, CandidateConfig};
use crate::graph::{Fow, Frc, RoadNetwork};
use crate::spatial::SpatialIndex;

/// A* search node for the priority queue
#[derive(Clone)]
struct AStarNode {
    node: NodeIndex,
    g_score: f64, // Cost from start to this node
    f_score: f64, // g_score + heuristic (estimated total cost)
}

impl PartialEq for AStarNode {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for AStarNode {}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (lower f_score = higher priority)
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

/// Bounded A* search that stops when path cost exceeds max_cost
///
/// The optional `max_frc` parameter implements LFRCNP (Lowest FRC to Next Point) filtering
/// per OpenLR spec Section 12.1 Step 5. When provided, the search will only traverse edges
/// with FRC <= max_frc (lower FRC = higher road importance).
fn bounded_astar(
    network: &RoadNetwork,
    start: NodeIndex,
    goal: NodeIndex,
    goal_coord: Point<f64>,
    max_cost: f64,
    max_frc: Option<Frc>,
) -> Option<(f64, Vec<NodeIndex>, Vec<EdgeIndex>)> {
    let mut open_set = BinaryHeap::new();
    let mut g_scores: HashMap<NodeIndex, f64> = HashMap::new();
    let mut came_from: HashMap<NodeIndex, (NodeIndex, EdgeIndex)> = HashMap::new();

    // Initialize with start node
    let start_h = network
        .node(start)
        .map(|n| n.coord.haversine_distance(&goal_coord))
        .unwrap_or(0.0);

    g_scores.insert(start, 0.0);
    open_set.push(AStarNode {
        node: start,
        g_score: 0.0,
        f_score: start_h,
    });

    while let Some(current) = open_set.pop() {
        // Found the goal
        if current.node == goal {
            // Reconstruct path
            let mut nodes = vec![goal];
            let mut edges = Vec::new();
            let mut node = goal;
            while let Some(&(prev, edge_idx)) = came_from.get(&node) {
                nodes.push(prev);
                edges.push(edge_idx);
                node = prev;
            }
            nodes.reverse();
            edges.reverse();
            return Some((current.g_score, nodes, edges));
        }

        // Skip if we've already found a better path to this node
        if let Some(&best_g) = g_scores.get(&current.node) {
            if current.g_score > best_g {
                continue;
            }
        }

        // Explore neighbors
        for edge in network.graph.edges(current.node) {
            // LFRCNP filtering: skip edges with FRC higher (less important) than allowed
            // Per OpenLR spec, if LFRCNP = FRC3, only traverse edges with FRC 0, 1, 2, or 3
            //
            // Exception: SlipRoad (ramps/links) are always allowed regardless of FRC.
            // This handles cross-provider mapping where motorway_link is FRC3 in OSM
            // but should be traversable when connecting to/from motorways (FRC0/FRC1).
            if let Some(max) = max_frc {
                let is_slip_road = edge.weight().fow == Fow::SlipRoad;
                if edge.weight().frc > max && !is_slip_road {
                    continue;
                }
            }

            let neighbor = edge.target();
            let edge_cost = edge.weight().length_m;
            let tentative_g = current.g_score + edge_cost;

            // BOUNDED: Skip if this path already exceeds max cost
            if tentative_g > max_cost {
                continue;
            }

            // Check if this is a better path to neighbor
            let dominated = g_scores
                .get(&neighbor)
                .map(|&g| tentative_g >= g)
                .unwrap_or(false);
            if dominated {
                continue;
            }

            // This is a better path
            came_from.insert(neighbor, (current.node, edge.id()));
            g_scores.insert(neighbor, tentative_g);

            let h = network
                .node(neighbor)
                .map(|n| n.coord.haversine_distance(&goal_coord))
                .unwrap_or(0.0);

            open_set.push(AStarNode {
                node: neighbor,
                g_score: tentative_g,
                f_score: tentative_g + h,
            });
        }
    }

    None // No path found within max_cost
}

#[derive(Error, Debug)]
pub enum DecodeError {
    #[error("Failed to deserialize OpenLR: {0}")]
    Deserialize(String),

    #[error("No candidates found for LRP {index}")]
    NoCandidates { index: usize },

    #[error("No valid path found between LRPs {from} and {to}")]
    NoPath { from: usize, to: usize },

    #[error("Path length mismatch: expected {expected}m, got {actual}m")]
    LengthMismatch { expected: f64, actual: f64 },

    #[error("Unsupported location type: {0}")]
    UnsupportedType(String),
}

/// A decoded path result
#[derive(Debug, Clone)]
pub struct DecodedPath {
    /// Sequence of stable edge IDs forming the path
    pub edge_ids: Vec<u64>,
    /// Total path length in meters
    pub length_m: f64,
    /// Positive offset (trim from start) in meters
    pub positive_offset_m: f64,
    /// Negative offset (trim from end) in meters
    pub negative_offset_m: f64,
    /// The edge ID that covers the most distance in the decoded path
    pub primary_edge_id: u64,
    /// The distance covered by the primary edge in meters
    pub primary_edge_coverage_m: f64,
}

/// Information about edge coverage in a path segment
#[derive(Debug, Clone)]
struct EdgeCoverage {
    edge_idx: EdgeIndex,
    coverage_m: f64,
}

/// Result from path finding including edge coverage breakdown
#[derive(Debug, Clone)]
struct PathResult {
    edges: Vec<EdgeIndex>,
    coverages: Vec<EdgeCoverage>,
    total_length: f64,
}

/// Configuration for the decoder
#[derive(Debug, Clone)]
pub struct DecoderConfig {
    pub candidate_config: CandidateConfig,
    /// Tolerance for path length matching (fraction, e.g., 0.15 = 15%)
    pub length_tolerance: f64,
    /// Absolute tolerance for path length matching in meters
    pub absolute_length_tolerance: f64,
    /// Maximum path search distance multiplier
    pub max_search_distance_factor: f64,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        DecoderConfig {
            candidate_config: CandidateConfig::default(),
            length_tolerance: 0.35, // 35% tolerance on path length (for cross-provider decoding)
            absolute_length_tolerance: 100.0, // 100m absolute tolerance
            max_search_distance_factor: 2.0, // Search up to 2x the expected distance
        }
    }
}

/// OpenLR decoder with map-matching capability
pub struct Decoder<'a> {
    network: &'a RoadNetwork,
    spatial: &'a SpatialIndex,
    config: DecoderConfig,
}

impl<'a> Decoder<'a> {
    pub fn new(network: &'a RoadNetwork, spatial: &'a SpatialIndex) -> Self {
        Decoder {
            network,
            spatial,
            config: DecoderConfig::default(),
        }
    }

    pub fn with_config(mut self, config: DecoderConfig) -> Self {
        self.config = config;
        self
    }

    /// Convert edge indices to stable edge IDs
    fn edge_indices_to_ids(&self, indices: &[EdgeIndex]) -> Vec<u64> {
        indices
            .iter()
            .filter_map(|&idx| self.network.edge(idx).map(|e| e.id))
            .collect()
    }

    /// Decode an OpenLR base64 string to a path
    pub fn decode(&self, openlr_base64: &str) -> Result<DecodedPath, DecodeError> {
        // Deserialize using the openlr crate
        let location = openlr::deserialize_base64_openlr(openlr_base64)
            .map_err(|e| DecodeError::Deserialize(format!("{:?}", e)))?;

        match location {
            openlr::LocationReference::Line(line_loc) => self.decode_line(line_loc),
            openlr::LocationReference::PointAlongLine(pal) => self.decode_point_along_line(pal),
            openlr::LocationReference::GeoCoordinate(_) => {
                Err(DecodeError::UnsupportedType("GeoCoordinate".to_string()))
            }
            openlr::LocationReference::Poi(_) => {
                Err(DecodeError::UnsupportedType("Poi".to_string()))
            }
            openlr::LocationReference::Circle(_) => {
                Err(DecodeError::UnsupportedType("Circle".to_string()))
            }
            openlr::LocationReference::Rectangle(_) => {
                Err(DecodeError::UnsupportedType("Rectangle".to_string()))
            }
            openlr::LocationReference::Grid(_) => {
                Err(DecodeError::UnsupportedType("Grid".to_string()))
            }
            openlr::LocationReference::Polygon(_) => {
                Err(DecodeError::UnsupportedType("Polygon".to_string()))
            }
            openlr::LocationReference::ClosedLine(_) => {
                Err(DecodeError::UnsupportedType("ClosedLine".to_string()))
            }
        }
    }

    /// Decode a line location reference
    fn decode_line(&self, line: openlr::Line) -> Result<DecodedPath, DecodeError> {
        let points = &line.points;
        if points.is_empty() {
            return Err(DecodeError::Deserialize("Empty LRP list".to_string()));
        }

        // Find candidates for each LRP
        let mut all_candidates: Vec<Vec<Candidate>> = Vec::with_capacity(points.len());

        for (i, point) in points.iter().enumerate() {
            let coord = Point::new(point.coordinate.lon, point.coordinate.lat);
            let bearing = point.line.bearing.degrees() as f64;
            let frc = Frc::from_u8(point.line.frc.value() as u8);
            let fow = Fow::from_u8(point.line.fow.value() as u8);

            let candidates = find_candidates(
                coord,
                bearing,
                frc,
                fow,
                self.network,
                self.spatial,
                &self.config.candidate_config,
            );

            if candidates.is_empty() {
                return Err(DecodeError::NoCandidates { index: i });
            }

            all_candidates.push(candidates);
        }

        // Require at least one LRP to have a candidate within max_candidate_distance_m.
        // This allows partial network coverage (one LRP outside network) while still
        // rejecting cases where NO LRP has a good spatial match.
        let max_dist = self.config.candidate_config.max_candidate_distance_m;
        let has_close_candidate = all_candidates
            .iter()
            .any(|candidates| candidates.iter().any(|c| c.distance_m <= max_dist));

        if !has_close_candidate {
            // Find the LRP with the closest candidate to give a useful error
            let (worst_idx, _) = all_candidates
                .iter()
                .enumerate()
                .map(|(i, c)| {
                    let min_dist = c.iter().map(|c| c.distance_m).fold(f64::MAX, f64::min);
                    (i, min_dist)
                })
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap_or((0, f64::MAX));
            return Err(DecodeError::NoCandidates { index: worst_idx });
        }

        // Build path between consecutive LRPs
        let mut full_path: Vec<EdgeIndex> = Vec::new();
        let mut total_length = 0.0;
        // Track coverage per edge (edge may appear multiple times, so we aggregate)
        let mut edge_coverage_map: HashMap<EdgeIndex, f64> = HashMap::new();

        for i in 0..points.len() - 1 {
            // Get distance to next point (path attributes are optional for last point)
            let expected_distance = points[i]
                .path
                .as_ref()
                .map(|p| p.dnp.meters())
                .unwrap_or(0.0);

            // Extract LFRCNP (Lowest FRC to Next Point) for path filtering
            let lfrcnp = points[i]
                .path
                .as_ref()
                .map(|p| Frc::from_u8(p.lfrcnp.value() as u8));

            let path_result = self.find_best_path(
                &all_candidates[i],
                &all_candidates[i + 1],
                expected_distance,
                i,
                lfrcnp,
            )?;

            // Add the actual traversed length (accounts for partial edge traversals)
            total_length += path_result.total_length;

            // Accumulate edge coverages (aggregate if edge appears multiple times)
            for coverage in &path_result.coverages {
                *edge_coverage_map.entry(coverage.edge_idx).or_insert(0.0) += coverage.coverage_m;
            }

            // Add edges to full path (avoid duplicating junction edges)
            for (j, &edge_idx) in path_result.edges.iter().enumerate() {
                if j == 0 && !full_path.is_empty() {
                    // Skip first edge if it's the same as the last (junction)
                    if full_path.last() == Some(&edge_idx) {
                        continue;
                    }
                }
                full_path.push(edge_idx);
            }
        }

        // Find the primary edge (the one with maximum coverage)
        let (primary_edge_idx, primary_coverage) = edge_coverage_map
            .iter()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(&idx, &cov)| (idx, cov))
            .unwrap_or_else(|| (full_path.first().copied().unwrap_or(EdgeIndex::new(0)), 0.0));

        let primary_edge_id = self
            .network
            .edge(primary_edge_idx)
            .map(|e| e.id)
            .unwrap_or(0);

        // Extract offsets - convert from fractions (0.0-1.0) to meters
        let positive_offset_m = line.offsets.pos.range() * total_length;
        let negative_offset_m = line.offsets.neg.range() * total_length;

        Ok(DecodedPath {
            edge_ids: self.edge_indices_to_ids(&full_path),
            length_m: total_length,
            positive_offset_m,
            negative_offset_m,
            primary_edge_id,
            primary_edge_coverage_m: primary_coverage,
        })
    }

    /// Decode a point along line location
    fn decode_point_along_line(
        &self,
        pal: openlr::PointAlongLine,
    ) -> Result<DecodedPath, DecodeError> {
        // Point along line has exactly 2 points plus an offset
        let points = &pal.points;

        // Find candidates
        let mut all_candidates: Vec<Vec<Candidate>> = Vec::with_capacity(2);

        for (i, point) in points.iter().enumerate() {
            let coord = Point::new(point.coordinate.lon, point.coordinate.lat);
            let bearing = point.line.bearing.degrees() as f64;
            let frc = Frc::from_u8(point.line.frc.value() as u8);
            let fow = Fow::from_u8(point.line.fow.value() as u8);

            let candidates = find_candidates(
                coord,
                bearing,
                frc,
                fow,
                self.network,
                self.spatial,
                &self.config.candidate_config,
            );

            if candidates.is_empty() {
                return Err(DecodeError::NoCandidates { index: i });
            }

            all_candidates.push(candidates);
        }

        // Require at least one LRP to have a candidate within max_candidate_distance_m
        let max_dist = self.config.candidate_config.max_candidate_distance_m;
        let has_close_candidate = all_candidates
            .iter()
            .any(|candidates| candidates.iter().any(|c| c.distance_m <= max_dist));

        if !has_close_candidate {
            let (worst_idx, _) = all_candidates
                .iter()
                .enumerate()
                .map(|(i, c)| {
                    let min_dist = c.iter().map(|c| c.distance_m).fold(f64::MAX, f64::min);
                    (i, min_dist)
                })
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap_or((0, f64::MAX));
            return Err(DecodeError::NoCandidates { index: worst_idx });
        }

        let expected_distance = points[0]
            .path
            .as_ref()
            .map(|p| p.dnp.meters())
            .unwrap_or(0.0);

        // Extract LFRCNP (Lowest FRC to Next Point) for path filtering
        let lfrcnp = points[0]
            .path
            .as_ref()
            .map(|p| Frc::from_u8(p.lfrcnp.value() as u8));

        let path_result = self.find_best_path(
            &all_candidates[0],
            &all_candidates[1],
            expected_distance,
            0,
            lfrcnp,
        )?;

        // Find the primary edge (the one with maximum coverage)
        let (primary_edge_idx, primary_coverage) = path_result
            .coverages
            .iter()
            .max_by(|a, b| a.coverage_m.partial_cmp(&b.coverage_m).unwrap())
            .map(|c| (c.edge_idx, c.coverage_m))
            .unwrap_or_else(|| {
                (
                    path_result
                        .edges
                        .first()
                        .copied()
                        .unwrap_or(EdgeIndex::new(0)),
                    0.0,
                )
            });

        let primary_edge_id = self
            .network
            .edge(primary_edge_idx)
            .map(|e| e.id)
            .unwrap_or(0);

        // Convert offset from fraction (0.0-1.0) to meters
        let positive_offset_m = pal.offset.range() * path_result.total_length;

        Ok(DecodedPath {
            edge_ids: self.edge_indices_to_ids(&path_result.edges),
            length_m: path_result.total_length,
            positive_offset_m,
            negative_offset_m: 0.0,
            primary_edge_id,
            primary_edge_coverage_m: primary_coverage,
        })
    }

    /// Check if a single edge can serve both LRPs (same-edge solution)
    /// This is prioritized over multi-edge paths to avoid including edges
    /// that contribute nearly zero length to the final path.
    fn try_same_edge_solution(
        &self,
        start_candidates: &[Candidate],
        end_candidates: &[Candidate],
        expected_distance: f64,
        min_distance: f64,
        max_valid_distance: f64,
        max_frc: Option<Frc>,
    ) -> Option<PathResult> {
        // Find edges that appear in both candidate lists
        // Only consider candidates with good spatial match (within 10m)
        const MAX_PROJECTION_DISTANCE: f64 = 10.0;

        let mut best_same_edge: Option<(EdgeIndex, f64, f64)> = None; // (edge_idx, length, score)

        for start_cand in start_candidates.iter().take(10) {
            if start_cand.distance_m > MAX_PROJECTION_DISTANCE {
                continue;
            }

            for end_cand in end_candidates.iter().take(10) {
                if end_cand.distance_m > MAX_PROJECTION_DISTANCE {
                    continue;
                }

                // Check if same edge
                if start_cand.edge_idx != end_cand.edge_idx {
                    continue;
                }

                let edge = self.network.edge(start_cand.edge_idx)?;

                // Check LFRCNP constraint (SlipRoads always allowed)
                if let Some(max) = max_frc {
                    let is_slip_road = edge.fow == Fow::SlipRoad;
                    if edge.frc > max && !is_slip_road {
                        continue;
                    }
                }

                // Check projection order (end must be after start)
                let frac_diff = end_cand.projection_fraction - start_cand.projection_fraction;
                if frac_diff <= 0.0 {
                    continue;
                }

                let path_cost = edge.length_m * frac_diff;

                // For same-edge with excellent spatial match, allow relaxed length tolerance
                let excellent_match = start_cand.distance_m < 5.0 && end_cand.distance_m < 5.0;
                let effective_max = if excellent_match {
                    max_valid_distance * 3.0
                } else {
                    max_valid_distance
                };

                if path_cost < min_distance || path_cost > effective_max {
                    continue;
                }

                let length_diff =
                    (path_cost - expected_distance).abs() / expected_distance.max(1.0);
                let score = start_cand.score + end_cand.score + length_diff;

                // Track the best same-edge solution
                if best_same_edge.is_none() || score < best_same_edge.as_ref().unwrap().2 {
                    best_same_edge = Some((start_cand.edge_idx, path_cost, score));
                }
            }
        }

        best_same_edge.map(|(edge_idx, length, _score)| PathResult {
            edges: vec![edge_idx],
            coverages: vec![EdgeCoverage {
                edge_idx,
                coverage_m: length,
            }],
            total_length: length,
        })
    }

    /// Find the best path between candidate sets
    /// Returns PathResult with edges, coverages, and total length
    ///
    /// The optional `lfrcnp` parameter specifies the Lowest FRC to Next Point constraint.
    /// When provided, the A* search will only traverse edges with FRC <= lfrcnp.
    /// For cross-provider decoding (HERE→OSM), we add +1 tolerance to account for
    /// FRC mapping differences between map providers.
    fn find_best_path(
        &self,
        start_candidates: &[Candidate],
        end_candidates: &[Candidate],
        expected_distance: f64,
        lrp_index: usize,
        lfrcnp: Option<Frc>,
    ) -> Result<PathResult, DecodeError> {
        // Compute distance bounds using both relative and absolute tolerance
        // For short segments, absolute tolerance dominates; for long segments, relative dominates
        let rel_min = expected_distance * (1.0 - self.config.length_tolerance);
        let rel_max = expected_distance * (1.0 + self.config.length_tolerance);

        // For minimum: be lenient to handle OpenLR's DNP quantization
        // DNP encoding can significantly overestimate short distances
        // Use the smallest of: relative tolerance, absolute tolerance, or a 10m floor
        let abs_min = (expected_distance - self.config.absolute_length_tolerance).max(0.0);
        let min_distance = rel_min.min(abs_min).max(10.0); // Absolute floor of 10m

        // For maximum: use whichever is MORE GENEROUS
        // For short segments, absolute tolerance provides necessary slack for cross-provider
        // geometry differences; for long segments, relative tolerance is more appropriate
        let abs_max = expected_distance + self.config.absolute_length_tolerance;
        let max_valid_distance = rel_max.max(abs_max);

        // Maximum distance for A* search - don't explore beyond this
        // Use max_search_distance_factor to bound the search, with a minimum of 500m
        // to handle short segments where 2x might be too restrictive
        let max_search_distance =
            (expected_distance * self.config.max_search_distance_factor).max(500.0);

        // PRIORITY CHECK: Look for same-edge solutions with strict LFRCNP first
        // When both LRPs project closely onto the same edge, prefer this simpler solution
        // over multi-edge paths that may score slightly better on individual candidates
        // but include edges contributing nearly zero length.
        if let Some(result) = self.try_same_edge_solution(
            start_candidates,
            end_candidates,
            expected_distance,
            min_distance,
            max_valid_distance,
            lfrcnp, // Strict LFRCNP
        ) {
            return Ok(result);
        }
        // NOTE: Relaxed same-edge fallback is deferred until after strict A* search fails
        // to ensure we don't bypass LFRCNP when a valid multi-edge path exists.

        // Build and sort candidate pairs by combined score (multiplicative)
        // Multiplicative scoring strongly penalizes pairs where either candidate is poor
        let mut pairs: Vec<(usize, usize, f64)> = Vec::with_capacity(100);
        for (i, start_cand) in start_candidates.iter().enumerate().take(10) {
            for (j, end_cand) in end_candidates.iter().enumerate().take(10) {
                // Multiplicative: pairs where BOTH are good score best
                // Add 1 to avoid multiplication by zero for perfect matches
                let combined = (start_cand.score + 1.0) * (end_cand.score + 1.0);
                pairs.push((i, j, combined));
            }
        }
        // Sort by combined score (lower is better)
        pairs.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

        let mut best_path: Option<PathResult> = None;
        let mut best_score = f64::MAX;

        // Two-pass approach for LFRCNP: try strict first, then fallback to relaxed
        // This prevents mixing valid FRC edges with invalid ones (e.g., taking a
        // residential detour when a primary road path exists within LFRCNP)
        for use_relaxed_lfrcnp in [false, true] {
            // If we found a path in strict pass, skip relaxed pass
            if use_relaxed_lfrcnp && best_path.is_some() {
                break;
            }

            // In relaxed pass, also try same-edge solution with relaxed LFRCNP
            // This is deferred from earlier to ensure strict multi-edge paths are tried first
            if use_relaxed_lfrcnp {
                let relaxed_frc = lfrcnp.map(|frc| Frc::from_u8((frc as u8).saturating_add(1)));
                if let Some(result) = self.try_same_edge_solution(
                    start_candidates,
                    end_candidates,
                    expected_distance,
                    min_distance,
                    max_valid_distance,
                    relaxed_frc,
                ) {
                    return Ok(result);
                }
            }

            for (start_idx, end_idx, _combined_score) in pairs.iter().copied() {
                let start_cand = &start_candidates[start_idx];
                let end_cand = &end_candidates[end_idx];

                // Check if start/end edges respect LFRCNP constraint
                // This prevents routing to lower-class roads (e.g., residential when LFRCNP=3)
                let (start_edge, end_edge) = match (
                    self.network.edge(start_cand.edge_idx),
                    self.network.edge(end_cand.edge_idx),
                ) {
                    (Some(s), Some(e)) => (s, e),
                    _ => continue,
                };

                // Apply LFRCNP to the partial edges connected to the LRPs
                // SlipRoads are always allowed (they connect different road classes)
                let start_is_slip_road = start_edge.fow == Fow::SlipRoad;
                let end_is_slip_road = end_edge.fow == Fow::SlipRoad;
                let max_allowed_frc = match (lfrcnp, use_relaxed_lfrcnp) {
                    (Some(frc), false) => frc,                                        // Strict
                    (Some(frc), true) => Frc::from_u8((frc as u8).saturating_add(1)), // Relaxed +1
                    (None, _) => Frc::Frc7, // No constraint
                };

                // Skip candidates that don't respect current LFRCNP threshold
                if start_edge.frc > max_allowed_frc && !start_is_slip_road {
                    continue;
                }
                if end_edge.frc > max_allowed_frc && !end_is_slip_road {
                    continue;
                }

                // Special case: both LRPs on the same edge
                if start_cand.edge_idx == end_cand.edge_idx {
                    if let Some(edge) = self.network.edge(start_cand.edge_idx) {
                        // Calculate distance along edge between projection points
                        let frac_diff =
                            end_cand.projection_fraction - start_cand.projection_fraction;

                        // Only valid if end is after start on the edge (positive direction)
                        if frac_diff > 0.0 {
                            let path_cost = edge.length_m * frac_diff;

                            // For same-edge case with excellent spatial matches (both < 5m),
                            // allow more length flexibility for cross-provider decoding
                            let excellent_spatial_match =
                                start_cand.distance_m < 5.0 && end_cand.distance_m < 5.0;
                            let relaxed_max = if excellent_spatial_match {
                                max_valid_distance * 3.0 // Allow up to 3x for very close matches
                            } else {
                                max_valid_distance
                            };

                            if path_cost >= min_distance && path_cost <= relaxed_max {
                                let length_diff = (path_cost - expected_distance).abs()
                                    / expected_distance.max(1.0);
                                let score = start_cand.score + end_cand.score + length_diff;

                                if score < best_score {
                                    best_score = score;
                                    best_path = Some(PathResult {
                                        edges: vec![start_cand.edge_idx],
                                        coverages: vec![EdgeCoverage {
                                            edge_idx: start_cand.edge_idx,
                                            coverage_m: path_cost,
                                        }],
                                        total_length: path_cost,
                                    });

                                    if length_diff < 0.1 {
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    continue;
                }

                // For the start edge, we typically exit from the target node (continuing in travel direction)
                // For the end edge, we typically enter at the source node
                let start_node = match self.network.edge_target(start_cand.edge_idx) {
                    Some(n) => n,
                    None => continue,
                };
                let end_node = match self.network.edge_source(end_cand.edge_idx) {
                    Some(n) => n,
                    None => continue,
                };

                // Get end node coordinates for A* heuristic
                let end_coord = match self.network.node(end_node) {
                    Some(n) => n.coord,
                    None => continue,
                };

                // Calculate partial edge lengths to add to path cost
                // Start edge: from projection point to target = (1 - fraction) * length
                // End edge: from source to projection point = fraction * length
                let start_edge_partial =
                    start_edge.length_m * (1.0 - start_cand.projection_fraction);
                let end_edge_partial = end_edge.length_m * end_cand.projection_fraction;

                // Adjust max search for the middle path portion
                let middle_max = max_search_distance - start_edge_partial - end_edge_partial;
                if middle_max <= 0.0 {
                    continue;
                }

                // Run bounded A* for the middle portion (between edges)
                // SlipRoad/link edges are always allowed regardless of FRC (handled in bounded_astar).
                // Only use relaxed LFRCNP (+1) when we're in the relaxed pass for candidates,
                // to prevent mixed-class paths from sneaking through the strict pass.
                let max_frc_for_astar = if use_relaxed_lfrcnp {
                    lfrcnp.map(|frc| Frc::from_u8((frc as u8).saturating_add(1)))
                } else {
                    lfrcnp
                };

                let (middle_cost, _path_nodes, middle_edges) = if start_node == end_node {
                    // Start and end edges share a node - no middle path needed
                    (0.0, vec![start_node], Vec::new())
                } else {
                    match bounded_astar(
                        self.network,
                        start_node,
                        end_node,
                        end_coord,
                        middle_max,
                        max_frc_for_astar,
                    ) {
                        Some(result) => result,
                        None => continue,
                    }
                };

                // Total path cost includes partial edge traversals
                let path_cost = start_edge_partial + middle_cost + end_edge_partial;

                // Check if path length is within tolerance
                if path_cost < min_distance || path_cost > max_valid_distance {
                    continue;
                }

                // Build edge list: start_edge + middle edges + end_edge
                // Skip start/end edges if they contribute negligible length (< 3% of path)
                // This avoids including spurious edges when an LRP is at a junction
                const MIN_EDGE_CONTRIBUTION: f64 = 0.03; // 3% of path length
                let mut edges = Vec::new();
                let mut coverages = Vec::new();

                if start_edge_partial / path_cost >= MIN_EDGE_CONTRIBUTION {
                    edges.push(start_cand.edge_idx);
                    coverages.push(EdgeCoverage {
                        edge_idx: start_cand.edge_idx,
                        coverage_m: start_edge_partial,
                    });
                }

                // Add middle edges with their full lengths
                for &edge_idx in &middle_edges {
                    if let Some(edge) = self.network.edge(edge_idx) {
                        edges.push(edge_idx);
                        coverages.push(EdgeCoverage {
                            edge_idx,
                            coverage_m: edge.length_m,
                        });
                    }
                }

                if end_cand.edge_idx != start_cand.edge_idx
                    && end_edge_partial / path_cost >= MIN_EDGE_CONTRIBUTION
                {
                    edges.push(end_cand.edge_idx);
                    coverages.push(EdgeCoverage {
                        edge_idx: end_cand.edge_idx,
                        coverage_m: end_edge_partial,
                    });
                }

                // Score the path (additive for final ranking)
                let length_diff =
                    (path_cost - expected_distance).abs() / expected_distance.max(1.0);
                let score = start_cand.score + end_cand.score + length_diff;

                if score < best_score {
                    best_score = score;
                    best_path = Some(PathResult {
                        edges,
                        coverages,
                        total_length: path_cost,
                    });

                    // Early termination: if we found a good match, stop searching
                    // Since pairs are sorted by candidate quality, the first valid
                    // path with good length match is likely optimal
                    if length_diff < 0.1 {
                        // Path length within 10% of expected
                        break;
                    }
                }
            } // end inner for loop over pairs
        } // end outer for loop over [strict, relaxed]

        best_path.ok_or(DecodeError::NoPath {
            from: lrp_index,
            to: lrp_index + 1,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::candidates::Candidate;
    use crate::graph::{Fow, Frc};
    use crate::test_utils::TestNetworkBuilder;

    #[test]
    fn test_decoder_config_defaults() {
        let config = DecoderConfig::default();
        assert_eq!(config.length_tolerance, 0.35);
    }

    #[test]
    fn test_find_best_path_prefers_allowed_start_edge() {
        let (network, spatial) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.001)
            .add_node(3, 0.0, 0.002)
            .add_edge(100, 1, 2, 20.0, Frc::Frc4, Fow::SingleCarriageway) // Service road
            .add_edge(101, 1, 2, 20.0, Frc::Frc3, Fow::SingleCarriageway) // Secondary
            .add_edge(102, 2, 3, 20.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let decoder = Decoder::new(&network, &spatial);

        let service_edge_idx = *network
            .edge_id_to_index
            .as_ref()
            .unwrap()
            .get(&100)
            .unwrap();
        let secondary_edge_idx = *network
            .edge_id_to_index
            .as_ref()
            .unwrap()
            .get(&101)
            .unwrap();
        let end_edge_idx = *network
            .edge_id_to_index
            .as_ref()
            .unwrap()
            .get(&102)
            .unwrap();

        let service_candidate = Candidate {
            edge_idx: service_edge_idx,
            distance_m: 0.0,
            bearing_diff: 0.0,
            frc_diff: 0,
            fow_score: 1.0,
            score: 0.1, // Would normally be preferred without LFRCNP filter
            projection_fraction: 0.0,
        };
        let allowed_candidate = Candidate {
            edge_idx: secondary_edge_idx,
            distance_m: 0.0,
            bearing_diff: 0.0,
            frc_diff: 0,
            fow_score: 1.0,
            score: 1.0,
            projection_fraction: 0.0,
        };
        let end_candidate = Candidate {
            edge_idx: end_edge_idx,
            distance_m: 0.0,
            bearing_diff: 0.0,
            frc_diff: 0,
            fow_score: 1.0,
            score: 0.0,
            projection_fraction: 0.5,
        };

        let start_candidates = vec![service_candidate, allowed_candidate];
        let end_candidates = vec![end_candidate];

        let result = decoder
            .find_best_path(&start_candidates, &end_candidates, 30.0, 0, Some(Frc::Frc3))
            .expect("Secondary road should satisfy LFRCNP");

        assert!(
            result.edges.contains(&secondary_edge_idx),
            "Path should include the allowed FRC3 edge"
        );
        assert!(
            !result.edges.contains(&service_edge_idx),
            "Service road must be filtered out when LFRCNP=FRC3"
        );
        assert!(
            (result.total_length - 30.0).abs() < 1e-6,
            "Path length should match the partial traversals"
        );
    }

    // =========================================================================
    // Route Search Tests (ported from Java OpenLR RouteSearchTest)
    // =========================================================================
    //
    // These tests verify the bounded A* search algorithm behavior, inspired by
    // the Java OpenLR decoder's RouteSearchTest. The key scenarios tested are:
    //
    // 1. testValidRoute - A* finds a valid path within max_cost constraints
    // 2. testNoRouteFound - A* returns None when max_cost is too short
    // 3. Path cost verification - The returned cost matches actual path length

    /// Test that bounded_astar finds a valid path when one exists within max_cost
    /// Equivalent to Java testValidRoute - verifies route is found with sufficient max_distance
    #[test]
    fn test_bounded_astar_finds_valid_path() {
        // Build a simple 3-node linear network: 1 -> 2 -> 3
        // Edge 1-2: 100m, Edge 2-3: 150m, Total: 250m
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 49.60, 6.120)
            .add_node(2, 49.60, 6.121)
            .add_node(3, 49.60, 6.122)
            .add_edge(1, 1, 2, 100.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(2, 2, 3, 150.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&3).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        // Max cost of 500m should easily accommodate the 250m path
        let result = bounded_astar(&network, start, goal, goal_coord, 500.0, None);

        assert!(result.is_some(), "Should find a valid path within max_cost");

        let (cost, nodes, _edges) = result.unwrap();
        assert_eq!(
            nodes.len(),
            3,
            "Path should have 3 nodes: start, middle, goal"
        );
        assert_eq!(nodes[0], start);
        assert_eq!(nodes[2], goal);
        assert!(
            (cost - 250.0).abs() < 1.0,
            "Path cost should be ~250m, got {}",
            cost
        );
    }

    /// Test that bounded_astar returns None when max_cost is too short
    /// Equivalent to Java testNoRouteFound - route search fails with max_distance=100m
    #[test]
    fn test_bounded_astar_no_route_when_max_cost_exceeded() {
        // Same network as above: total path = 250m
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 49.60, 6.120)
            .add_node(2, 49.60, 6.121)
            .add_node(3, 49.60, 6.122)
            .add_edge(1, 1, 2, 100.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(2, 2, 3, 150.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&3).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        // Max cost of 100m is less than the 250m path - should fail
        // (This mirrors Java's NO_ROUTE_FOUND_MAX_DISTANCE = 100)
        let result = bounded_astar(&network, start, goal, goal_coord, 100.0, None);

        assert!(
            result.is_none(),
            "Should return None when path exceeds max_cost"
        );
    }

    /// Test that bounded_astar returns the correct path cost
    #[test]
    fn test_bounded_astar_returns_correct_cost() {
        // Build a longer network: 1 -> 2 -> 3 -> 4
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 49.60, 6.120)
            .add_node(2, 49.60, 6.121)
            .add_node(3, 49.60, 6.122)
            .add_node(4, 49.60, 6.123)
            .add_edge(1, 1, 2, 172.0, Frc::Frc3, Fow::SingleCarriageway) // Matches Java edge 4 length
            .add_edge(2, 2, 3, 90.0, Frc::Frc3, Fow::SingleCarriageway) // Matches Java edge 6 length
            .add_edge(3, 3, 4, 50.0, Frc::Frc3, Fow::SingleCarriageway) // Matches Java edge 8 length
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        let result = bounded_astar(&network, start, goal, goal_coord, 1000.0, None);
        assert!(result.is_some());

        let (cost, nodes, _edges) = result.unwrap();

        // Expected total: 172 + 90 + 50 = 312m
        assert!(
            (cost - 312.0).abs() < 1.0,
            "Path cost should be 312m, got {}",
            cost
        );
        assert_eq!(nodes.len(), 4, "Path should traverse all 4 nodes");
    }

    /// Test bounded_astar with branching paths (chooses shortest)
    /// This simulates the topology from Java tests where different FRC constraints
    /// lead to different paths. Here we just verify A* finds the shortest path.
    #[test]
    fn test_bounded_astar_chooses_shortest_path() {
        // Build a diamond-shaped network:
        //     2
        //    / \
        //   1   4
        //    \ /
        //     3
        //
        // Path via 2: 100 + 100 = 200m
        // Path via 3: 150 + 150 = 300m
        // A* should choose the shorter path via node 2
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.001, 0.001)
            .add_node(3, -0.001, 0.001)
            .add_node(4, 0.0, 0.002)
            // Path via node 2 (shorter: 100 + 100 = 200m)
            .add_edge(1, 1, 2, 100.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(2, 2, 4, 100.0, Frc::Frc3, Fow::SingleCarriageway)
            // Path via node 3 (longer: 150 + 150 = 300m)
            .add_edge(3, 1, 3, 150.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(4, 3, 4, 150.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        let result = bounded_astar(&network, start, goal, goal_coord, 500.0, None);
        assert!(result.is_some());

        let (cost, nodes, _edges) = result.unwrap();

        // A* should find the shorter 200m path via node 2
        assert!(
            (cost - 200.0).abs() < 1.0,
            "Should find shortest path (200m), got {}m",
            cost
        );
        assert_eq!(nodes.len(), 3, "Shortest path has 3 nodes");

        // Verify it went through node 2, not node 3
        let node2_idx = *network.node_id_to_index.as_ref().unwrap().get(&2).unwrap();
        assert!(
            nodes.contains(&node2_idx),
            "Path should go through node 2 (shorter route)"
        );
    }

    #[test]
    fn test_bounded_astar_returns_actual_edge_sequence() {
        // Parallel edges between the same nodes (one allowed, one filtered)
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.001)
            .add_node(3, 0.0, 0.002)
            // Allowed secondary edge
            .add_edge(100, 1, 2, 20.0, Frc::Frc3, Fow::SingleCarriageway)
            // Disallowed service edge between same nodes
            .add_edge(101, 1, 2, 20.0, Frc::Frc4, Fow::SingleCarriageway)
            // Downstream edge to reach goal
            .add_edge(102, 2, 3, 20.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&3).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        let result =
            bounded_astar(&network, start, goal, goal_coord, 200.0, Some(Frc::Frc3)).unwrap();
        let (_, _, edges) = result;
        let edge_ids: Vec<u64> = edges
            .iter()
            .filter_map(|idx| network.edge(*idx).map(|e| e.id))
            .collect();

        assert_eq!(
            edge_ids,
            vec![100, 102],
            "A* should return the actual traversed edges respecting LFRCNP"
        );
    }

    /// Test that bounded_astar handles same start and goal node
    #[test]
    fn test_bounded_astar_same_start_and_goal() {
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 49.60, 6.120)
            .add_node(2, 49.60, 6.121)
            .add_edge(1, 1, 2, 100.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = start; // Same node
        let goal_coord = network.node(goal).unwrap().coord;

        let result = bounded_astar(&network, start, goal, goal_coord, 100.0, None);
        assert!(result.is_some());

        let (cost, nodes, _edges) = result.unwrap();
        assert_eq!(cost, 0.0, "Cost to same node should be 0");
        assert_eq!(nodes.len(), 1, "Path to same node should have 1 element");
        assert_eq!(nodes[0], start);
    }

    /// Test bounded_astar with disconnected nodes
    #[test]
    fn test_bounded_astar_disconnected_nodes() {
        // Create two separate components: 1->2 and 3->4 (not connected)
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 49.60, 6.120)
            .add_node(2, 49.60, 6.121)
            .add_node(3, 49.60, 6.130) // Separate component
            .add_node(4, 49.60, 6.131)
            .add_edge(1, 1, 2, 100.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(2, 3, 4, 100.0, Frc::Frc3, Fow::SingleCarriageway) // Disconnected from 1-2
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap(); // In different component
        let goal_coord = network.node(goal).unwrap().coord;

        let result = bounded_astar(&network, start, goal, goal_coord, 10000.0, None);

        assert!(
            result.is_none(),
            "Should return None for disconnected nodes"
        );
    }

    // =========================================================================
    // LFRCNP (Lowest FRC to Next Point) Filtering Tests
    // =========================================================================
    //
    // Per OpenLR spec Section 12.1 Step 5, path search should only use edges
    // with FRC <= LFRCNP. These tests verify that filtering works correctly.

    /// Test that LFRCNP filtering blocks paths through low-importance roads
    /// When LFRCNP = FRC2, paths should not traverse FRC3+ edges
    #[test]
    fn test_bounded_astar_lfrcnp_blocks_low_importance_roads() {
        // Build a diamond network where:
        // - Upper path (via node 2): FRC1 edges (high importance) - 200m total
        // - Lower path (via node 3): FRC5 edges (low importance) - 150m total (shorter!)
        //
        // With LFRCNP = FRC2, only the upper path should be usable
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.001, 0.001) // Upper path node
            .add_node(3, -0.001, 0.001) // Lower path node
            .add_node(4, 0.0, 0.002)
            // Upper path: FRC1 (major routes) - longer but high importance
            .add_edge(1, 1, 2, 100.0, Frc::Frc1, Fow::SingleCarriageway)
            .add_edge(2, 2, 4, 100.0, Frc::Frc1, Fow::SingleCarriageway)
            // Lower path: FRC5 (local roads) - shorter but low importance
            .add_edge(3, 1, 3, 75.0, Frc::Frc5, Fow::SingleCarriageway)
            .add_edge(4, 3, 4, 75.0, Frc::Frc5, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        // Without LFRCNP: should find shorter path via node 3 (150m)
        let result_no_filter = bounded_astar(&network, start, goal, goal_coord, 500.0, None);
        assert!(result_no_filter.is_some());
        let (cost_no_filter, _, _) = result_no_filter.unwrap();
        assert!(
            (cost_no_filter - 150.0).abs() < 1.0,
            "Without LFRCNP, should find shorter 150m path, got {}m",
            cost_no_filter
        );

        // With LFRCNP = FRC2: should find path via node 2 (200m), avoiding FRC5 roads
        let result_filtered =
            bounded_astar(&network, start, goal, goal_coord, 500.0, Some(Frc::Frc2));
        assert!(
            result_filtered.is_some(),
            "Should find path via high-importance roads"
        );
        let (cost_filtered, nodes, _edges) = result_filtered.unwrap();
        assert!(
            (cost_filtered - 200.0).abs() < 1.0,
            "With LFRCNP=FRC2, should find 200m path via FRC1 roads, got {}m",
            cost_filtered
        );

        // Verify path went through node 2 (high importance), not node 3 (low importance)
        let node2_idx = *network.node_id_to_index.as_ref().unwrap().get(&2).unwrap();
        let node3_idx = *network.node_id_to_index.as_ref().unwrap().get(&3).unwrap();
        assert!(nodes.contains(&node2_idx), "Path should go through node 2");
        assert!(
            !nodes.contains(&node3_idx),
            "Path should NOT go through node 3"
        );
    }

    /// Test that LFRCNP = FRC7 allows all roads (effectively no filtering)
    #[test]
    fn test_bounded_astar_lfrcnp_frc7_allows_all() {
        // Same diamond network as above
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.001, 0.001)
            .add_node(3, -0.001, 0.001)
            .add_node(4, 0.0, 0.002)
            .add_edge(1, 1, 2, 100.0, Frc::Frc1, Fow::SingleCarriageway)
            .add_edge(2, 2, 4, 100.0, Frc::Frc1, Fow::SingleCarriageway)
            .add_edge(3, 1, 3, 75.0, Frc::Frc7, Fow::SingleCarriageway) // Even FRC7 (lowest)
            .add_edge(4, 3, 4, 75.0, Frc::Frc7, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        // With LFRCNP = FRC7, even the lowest importance roads should be allowed
        let result = bounded_astar(&network, start, goal, goal_coord, 500.0, Some(Frc::Frc7));
        assert!(result.is_some());
        let (cost, _, _) = result.unwrap();
        assert!(
            (cost - 150.0).abs() < 1.0,
            "LFRCNP=FRC7 should allow FRC7 roads, finding 150m path, got {}m",
            cost
        );
    }

    /// Test that LFRCNP filtering can make a path unreachable
    #[test]
    fn test_bounded_astar_lfrcnp_no_valid_path() {
        // Network where only path uses FRC5 roads
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.001)
            .add_edge(1, 1, 2, 100.0, Frc::Frc5, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&2).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        // With LFRCNP = FRC3, the FRC5 edge should be blocked, making goal unreachable
        let result = bounded_astar(&network, start, goal, goal_coord, 500.0, Some(Frc::Frc3));
        assert!(
            result.is_none(),
            "With LFRCNP=FRC3, path through FRC5 road should be blocked"
        );

        // But without filtering, path should exist
        let result_unfiltered = bounded_astar(&network, start, goal, goal_coord, 500.0, None);
        assert!(
            result_unfiltered.is_some(),
            "Without LFRCNP, path should exist"
        );
    }
}
