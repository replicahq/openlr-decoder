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

/// Constraints for the bounded A* search beyond basic start/goal/cost.
struct AStarConstraints {
    /// LFRCNP filter: only traverse edges with FRC <= this value (None = no filter)
    max_frc: Option<Frc>,
    /// Extra cost added per slip road edge (soft preference for main roads)
    slip_road_cost_penalty: f64,
    /// Extra cost added per access road edge (soft preference for tertiary over residential)
    access_road_cost_penalty: f64,
}

impl Default for AStarConstraints {
    fn default() -> Self {
        AStarConstraints {
            max_frc: None,
            slip_road_cost_penalty: 0.0,
            access_road_cost_penalty: 0.0,
        }
    }
}

/// Bounded A* search that stops when path cost exceeds max_cost
///
/// The `constraints.max_frc` field implements LFRCNP (Lowest FRC to Next Point) filtering
/// per OpenLR spec Section 12.1 Step 5. When provided, the search will only traverse edges
/// with FRC <= max_frc (lower FRC = higher road importance).
///
/// The cost penalties add soft preferences that make the search favor main roads over slip roads
/// and tertiary over residential when paths are similar in length. The returned path length is
/// the real (unpenalized) distance, but with non-zero penalties the path found may not be the
/// globally shortest — the search trades minimal real-cost optimality for preferring higher-class
/// roads.
fn bounded_astar(
    network: &RoadNetwork,
    start: NodeIndex,
    goal: NodeIndex,
    goal_coord: Point<f64>,
    max_cost: f64,
    constraints: &AStarConstraints,
) -> Option<(f64, Vec<NodeIndex>, Vec<EdgeIndex>)> {
    let mut open_set = BinaryHeap::new();
    // Track both real distance (g_scores) and penalized cost (for priority)
    let mut g_scores: HashMap<NodeIndex, f64> = HashMap::new();
    let mut penalized_scores: HashMap<NodeIndex, f64> = HashMap::new();
    let mut came_from: HashMap<NodeIndex, (NodeIndex, EdgeIndex)> = HashMap::new();

    // Initialize with start node
    let start_h = network
        .node(start)
        .map(|n| n.coord.haversine_distance(&goal_coord))
        .unwrap_or(0.0);

    g_scores.insert(start, 0.0);
    penalized_scores.insert(start, 0.0);
    open_set.push(AStarNode {
        node: start,
        g_score: 0.0, // This tracks the penalized cost for priority
        f_score: start_h,
    });

    while let Some(current) = open_set.pop() {
        // Found the goal - return the REAL path length, not penalized cost
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
            // Return real g_score (path length), not the penalized cost
            let real_cost = g_scores.get(&goal).copied().unwrap_or(0.0);
            return Some((real_cost, nodes, edges));
        }

        // Skip if we've already found a better path to this node (using penalized cost)
        if let Some(&best_penalized) = penalized_scores.get(&current.node) {
            if current.g_score > best_penalized {
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
            if let Some(max) = constraints.max_frc {
                let is_slip_road = edge.weight().fow == Fow::SlipRoad;
                if edge.weight().frc > max && !is_slip_road {
                    continue;
                }
            }

            let neighbor = edge.target();
            let real_edge_cost = edge.weight().length_m;
            let penalty = if edge.weight().fow == Fow::SlipRoad {
                constraints.slip_road_cost_penalty
            } else if edge.weight().is_access_road {
                constraints.access_road_cost_penalty
            } else {
                0.0
            };

            let current_real_g = g_scores.get(&current.node).copied().unwrap_or(0.0);
            let tentative_real_g = current_real_g + real_edge_cost;
            let tentative_penalized_g = current.g_score + real_edge_cost + penalty;

            // BOUNDED: Skip if real path already exceeds max cost
            if tentative_real_g > max_cost {
                continue;
            }

            // Check if this is a better path to neighbor (using penalized cost for comparison)
            let dominated = penalized_scores
                .get(&neighbor)
                .map(|&g| tentative_penalized_g >= g)
                .unwrap_or(false);
            if dominated {
                continue;
            }

            // This is a better path
            came_from.insert(neighbor, (current.node, edge.id()));
            g_scores.insert(neighbor, tentative_real_g);
            penalized_scores.insert(neighbor, tentative_penalized_g);

            let h = network
                .node(neighbor)
                .map(|n| n.coord.haversine_distance(&goal_coord))
                .unwrap_or(0.0);

            open_set.push(AStarNode {
                node: neighbor,
                g_score: tentative_penalized_g, // Priority uses penalized cost
                f_score: tentative_penalized_g + h,
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
    /// Distance from start of first edge to first LRP projection (trim from start) in meters
    pub positive_offset_m: f64,
    /// Distance from last LRP projection to end of last edge (trim from end) in meters
    pub negative_offset_m: f64,
    /// `positive_offset_m` as a fraction of the first edge's length. Normally 0.0-1.0;
    /// may exceed 1.0 when a terminal-node snap prepended edges (the offset then
    /// spans more than the first edge — use `positive_offset_m` as authoritative).
    pub positive_offset_fraction: f64,
    /// `negative_offset_m` as a fraction of the last edge's length, measured from its
    /// end. Normally 0.0-1.0; may exceed 1.0 when a terminal-node snap appended edges.
    pub negative_offset_fraction: f64,
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

/// Length bookkeeping for one LRP-to-LRP segment, used to bound terminal snaps.
#[derive(Debug, Clone, Copy, Default)]
struct SegmentInfo {
    /// Decoded path length of the segment (meters)
    length_m: f64,
    /// Encoded DNP for the segment (meters)
    expected_m: f64,
}

/// Offsets computed for a finalized path.
#[derive(Debug, Clone, Copy, Default)]
struct PathOffsets {
    positive_offset_m: f64,
    negative_offset_m: f64,
    positive_offset_fraction: f64,
    negative_offset_fraction: f64,
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
    /// Cost penalty in meters for traversing slip roads (ramps/links) in A* search.
    /// This makes the decoder prefer staying on main roads over taking slip roads
    /// when paths are similar in length. Default 20m.
    pub slip_road_cost_penalty: f64,
    /// Cost penalty in meters for traversing access/local roads (residential, living_street,
    /// service, track) in A* search. This makes the decoder prefer tertiary/unclassified
    /// roads over residential when paths are similar in length. Default 10m.
    pub access_road_cost_penalty: f64,
    /// Extend decoded paths so their start and end lie on *valid* nodes (junctions),
    /// per OpenLR whitepaper Rule 4. When the selected path terminates on a node
    /// with no routing choice (1-in/1-out), the forced continuation is followed to
    /// the first junction and the added length is absorbed into the offsets, so
    /// the LRP-to-LRP geometry is unchanged. Default true.
    pub snap_to_valid_nodes: bool,
    /// Absolute cap (meters) on how far a terminal-node snap may extend a path
    /// at each end. Default 25m.
    pub max_snap_extension_m: f64,
    /// Cap on terminal-node snap length as a fraction of the adjacent segment's DNP.
    /// The effective cap is `min(max_snap_extension_m, fraction * dnp)`. Default 0.10.
    pub max_snap_extension_fraction: f64,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        DecoderConfig {
            candidate_config: CandidateConfig::default(),
            length_tolerance: 0.35, // 35% tolerance on path length (for cross-provider decoding)
            absolute_length_tolerance: 100.0, // 100m absolute tolerance
            max_search_distance_factor: 2.0, // Search up to 2x the expected distance
            slip_road_cost_penalty: 20.0, // 20m penalty gently prefers main roads over slip roads
            access_road_cost_penalty: 10.0, // 10m penalty gently prefers tertiary over residential
            snap_to_valid_nodes: true,
            max_snap_extension_m: 25.0,
            max_snap_extension_fraction: 0.10,
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

    /// Upper bound on acceptable decoded length for a segment with the given DNP:
    /// whichever of the relative and absolute tolerances is more generous.
    fn max_valid_distance(&self, expected_distance: f64) -> f64 {
        let rel_max = expected_distance * (1.0 + self.config.length_tolerance);
        let abs_max = expected_distance + self.config.absolute_length_tolerance;
        rel_max.max(abs_max)
    }

    /// Maximum length a terminal-node snap may add to one end of the path, given
    /// the adjacent segment. Returns 0 when snapping is disabled or the segment has
    /// no room left under `max_valid_distance`.
    fn snap_cap_m(&self, seg: SegmentInfo) -> f64 {
        if !self.config.snap_to_valid_nodes {
            return 0.0;
        }
        let cap = self
            .config
            .max_snap_extension_m
            .min(self.config.max_snap_extension_fraction * seg.expected_m);
        let headroom = self.max_valid_distance(seg.expected_m) - seg.length_m;
        cap.min(headroom).max(0.0)
    }

    /// Walk forward from the end of `path` along forced continuations (OpenLR Rule 4:
    /// nodes with no routing choice) until a valid node is reached. Returns the edges
    /// to append and their total length, or `None` if no valid node is reachable
    /// within `cap_m` (a partial extension would still end on an invalid node, so
    /// it is not worth applying).
    fn forward_extension(&self, path: &[EdgeIndex], cap_m: f64) -> Option<(Vec<EdgeIndex>, f64)> {
        let mut incoming = *path.last()?;
        let mut node = self.network.edge_target(incoming)?;
        let mut added = Vec::new();
        let mut added_m = 0.0;
        while let Some(next) = self.network.forced_continuation(node, incoming) {
            // Don't loop back onto edges we already traverse.
            if path.contains(&next) || added.contains(&next) {
                return None;
            }
            let len = self.network.edge(next)?.length_m;
            if added_m + len > cap_m {
                return None;
            }
            added_m += len;
            added.push(next);
            incoming = next;
            node = self.network.edge_target(next)?;
        }
        if added.is_empty() {
            None
        } else {
            Some((added, added_m))
        }
    }

    /// Mirror of `forward_extension`: walk backward from the start of `path` along
    /// forced predecessors to the nearest valid node. Returned edges are in path
    /// order (ready to be prepended).
    fn backward_extension(&self, path: &[EdgeIndex], cap_m: f64) -> Option<(Vec<EdgeIndex>, f64)> {
        let mut outgoing = *path.first()?;
        let mut node = self.network.edge_source(outgoing)?;
        let mut added = Vec::new();
        let mut added_m = 0.0;
        while let Some(prev) = self.network.forced_predecessor(node, outgoing) {
            if path.contains(&prev) || added.contains(&prev) {
                return None;
            }
            let len = self.network.edge(prev)?.length_m;
            if added_m + len > cap_m {
                return None;
            }
            added_m += len;
            added.push(prev);
            outgoing = prev;
            node = self.network.edge_source(prev)?;
        }
        if added.is_empty() {
            None
        } else {
            added.reverse();
            Some((added, added_m))
        }
    }

    /// Compute LRP-projection offsets for `path`, then snap its ends to valid nodes.
    ///
    /// Spec deviation: OpenLR Section 12.10 says offsets come from the binary-encoded
    /// fractions, but HERE data always encodes them as 0. Instead we project the LRP
    /// coordinates onto the first/last edges in our network, giving edge-relative
    /// offsets that are meaningful for consumers working with OSM edges.
    ///
    /// If `snap_to_valid_nodes` is enabled and a terminal edge ends on an invalid
    /// node (Rule 4), the path is extended along the forced continuation to the first
    /// junction and the added length is folded into the corresponding offset, so the
    /// offset-trimmed geometry is unchanged while the edge set covers the whole link.
    fn finalize_path(
        &self,
        path: &mut Vec<EdgeIndex>,
        first_lrp_coord: Point<f64>,
        last_lrp_coord: Point<f64>,
        first_segment: SegmentInfo,
        last_segment: SegmentInfo,
    ) -> PathOffsets {
        let (Some(&first_edge_idx), Some(&last_edge_idx)) = (path.first(), path.last()) else {
            return PathOffsets::default();
        };

        let mut positive_offset_m = self
            .network
            .edge(first_edge_idx)
            .map(|e| {
                e.length_m
                    * crate::spatial::project_point_to_line_fraction(first_lrp_coord, &e.geometry)
            })
            .unwrap_or(0.0);
        let mut negative_offset_m = self
            .network
            .edge(last_edge_idx)
            .map(|e| {
                e.length_m
                    * (1.0
                        - crate::spatial::project_point_to_line_fraction(
                            last_lrp_coord,
                            &e.geometry,
                        ))
            })
            .unwrap_or(0.0);

        if let Some((tail, added_m)) = self.forward_extension(path, self.snap_cap_m(last_segment)) {
            path.extend(tail);
            negative_offset_m += added_m;
        }
        if let Some((head, added_m)) = self.backward_extension(path, self.snap_cap_m(first_segment))
        {
            path.splice(0..0, head);
            positive_offset_m += added_m;
        }

        let frac = |offset_m: f64, edge_idx: Option<&EdgeIndex>| {
            edge_idx
                .and_then(|&idx| self.network.edge(idx))
                .filter(|e| e.length_m > 0.0)
                .map(|e| offset_m / e.length_m)
                .unwrap_or(0.0)
        };

        PathOffsets {
            positive_offset_m,
            negative_offset_m,
            positive_offset_fraction: frac(positive_offset_m, path.first()),
            negative_offset_fraction: frac(negative_offset_m, path.last()),
        }
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
        // (path length, expected DNP) of the first and last LRP segments, used to
        // bound the terminal-node snap.
        let mut first_segment = SegmentInfo::default();
        let mut last_segment = SegmentInfo::default();

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
            let seg = SegmentInfo {
                length_m: path_result.total_length,
                expected_m: expected_distance,
            };
            if i == 0 {
                first_segment = seg;
            }
            last_segment = seg;

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

        let first_lrp_coord = Point::new(points[0].coordinate.lon, points[0].coordinate.lat);
        let last_lrp_coord = Point::new(
            points[points.len() - 1].coordinate.lon,
            points[points.len() - 1].coordinate.lat,
        );
        let offsets = self.finalize_path(
            &mut full_path,
            first_lrp_coord,
            last_lrp_coord,
            first_segment,
            last_segment,
        );

        Ok(DecodedPath {
            edge_ids: self.edge_indices_to_ids(&full_path),
            length_m: total_length,
            positive_offset_m: offsets.positive_offset_m,
            negative_offset_m: offsets.negative_offset_m,
            positive_offset_fraction: offsets.positive_offset_fraction,
            negative_offset_fraction: offsets.negative_offset_fraction,
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

        let mut full_path = path_result.edges.clone();
        let first_lrp_coord = Point::new(points[0].coordinate.lon, points[0].coordinate.lat);
        let last_lrp_coord = Point::new(points[1].coordinate.lon, points[1].coordinate.lat);
        let seg = SegmentInfo {
            length_m: path_result.total_length,
            expected_m: expected_distance,
        };
        let offsets = self.finalize_path(&mut full_path, first_lrp_coord, last_lrp_coord, seg, seg);

        Ok(DecodedPath {
            edge_ids: self.edge_indices_to_ids(&full_path),
            length_m: path_result.total_length,
            positive_offset_m: offsets.positive_offset_m,
            negative_offset_m: offsets.negative_offset_m,
            positive_offset_fraction: offsets.positive_offset_fraction,
            negative_offset_fraction: offsets.negative_offset_fraction,
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
    ) -> Option<(PathResult, f64)> {
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

                // For same-edge with excellent spatial match, relax both min and max
                // length tolerances. DNP quantization can significantly overestimate
                // short distances, and when both LRPs project very close to the edge,
                // the spatial evidence is strong enough to trust even with large
                // length mismatches.
                let excellent_match = start_cand.distance_m < 5.0 && end_cand.distance_m < 5.0;
                let (effective_min, effective_max) = if excellent_match {
                    (0.0, max_valid_distance * 3.0)
                } else {
                    (min_distance, max_valid_distance)
                };

                if path_cost < effective_min || path_cost > effective_max {
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

        best_same_edge.map(|(edge_idx, length, score)| {
            (
                PathResult {
                    edges: vec![edge_idx],
                    coverages: vec![EdgeCoverage {
                        edge_idx,
                        coverage_m: length,
                    }],
                    total_length: length,
                },
                score,
            )
        })
    }

    /// Find the best path between candidate sets
    /// Returns PathResult with edges, coverages, and total length
    ///
    /// The optional `lfrcnp` parameter specifies the Lowest FRC to Next Point constraint.
    /// When provided, the search will only traverse edges with FRC <= lfrcnp + frc_tolerance.
    /// The frc_tolerance widens the threshold to account for FRC mapping differences
    /// between the encoding source and the target OSM network.
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
        let max_valid_distance = self.max_valid_distance(expected_distance);
        debug_assert!(max_valid_distance >= rel_max);

        // Maximum distance for A* search - don't explore beyond this
        // Use max_search_distance_factor to bound the search, with a minimum of 500m
        // to handle short segments where 2x might be too restrictive
        let max_search_distance =
            (expected_distance * self.config.max_search_distance_factor).max(500.0);

        // Apply frc_tolerance to LFRCNP: if frc_tolerance says FRC values may be off
        // by N levels, the same tolerance applies to the LFRCNP path constraint.
        let frc_tol = self.config.candidate_config.frc_tolerance;
        let max_path_frc = lfrcnp.map(|frc| Frc::from_u8((frc as u8).saturating_add(frc_tol)));

        // PRIORITY CHECK: Look for same-edge solutions first.
        // When both LRPs project closely onto the same edge, prefer this simpler solution
        // over multi-edge paths that may score slightly better on individual candidates
        // but include edges contributing nearly zero length.
        if let Some((result, _score)) = self.try_same_edge_solution(
            start_candidates,
            end_candidates,
            expected_distance,
            min_distance,
            max_valid_distance,
            max_path_frc,
        ) {
            return Ok(result);
        }

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

        for (start_idx, end_idx, _combined_score) in pairs.iter().copied() {
            let start_cand = &start_candidates[start_idx];
            let end_cand = &end_candidates[end_idx];

            // Check if start/end edges respect LFRCNP constraint (widened by frc_tolerance)
            let (start_edge, end_edge) = match (
                self.network.edge(start_cand.edge_idx),
                self.network.edge(end_cand.edge_idx),
            ) {
                (Some(s), Some(e)) => (s, e),
                _ => continue,
            };

            // SlipRoads are always allowed (they connect different road classes)
            let start_is_slip_road = start_edge.fow == Fow::SlipRoad;
            let end_is_slip_road = end_edge.fow == Fow::SlipRoad;
            let max_allowed_frc = max_path_frc.unwrap_or(Frc::Frc7);

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
                    let frac_diff = end_cand.projection_fraction - start_cand.projection_fraction;

                    // Only valid if end is after start on the edge (positive direction)
                    if frac_diff > 0.0 {
                        let path_cost = edge.length_m * frac_diff;

                        // For same-edge case with excellent spatial matches (both < 5m),
                        // allow more length flexibility for cross-provider decoding.
                        // Both min and max are relaxed: DNP quantization can significantly
                        // overestimate short distances, and close spatial match provides
                        // strong enough evidence.
                        let excellent_spatial_match =
                            start_cand.distance_m < 5.0 && end_cand.distance_m < 5.0;
                        let (effective_min, relaxed_max) = if excellent_spatial_match {
                            (0.0, max_valid_distance * 3.0)
                        } else {
                            (min_distance, max_valid_distance)
                        };

                        if path_cost >= effective_min && path_cost <= relaxed_max {
                            let length_diff =
                                (path_cost - expected_distance).abs() / expected_distance.max(1.0);
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
            let start_edge_partial = start_edge.length_m * (1.0 - start_cand.projection_fraction);
            let end_edge_partial = end_edge.length_m * end_cand.projection_fraction;

            // Adjust max search for the middle path portion
            let middle_max = max_search_distance - start_edge_partial - end_edge_partial;
            if middle_max <= 0.0 {
                continue;
            }

            // Run bounded A* for the middle portion (between edges)
            // SlipRoad/link edges are always allowed regardless of FRC (handled in bounded_astar).
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
                    &AStarConstraints {
                        max_frc: max_path_frc,
                        slip_road_cost_penalty: self.config.slip_road_cost_penalty,
                        access_road_cost_penalty: self.config.access_road_cost_penalty,
                    },
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
            let length_diff = (path_cost - expected_distance).abs() / expected_distance.max(1.0);
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
        }

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
            .add_edge(100, 1, 2, 20.0, Frc::Frc7, Fow::SingleCarriageway) // Low-class road
            .add_edge(101, 1, 2, 20.0, Frc::Frc3, Fow::SingleCarriageway) // Secondary
            .add_edge(102, 2, 3, 20.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        // Use frc_tolerance=0 to test strict LFRCNP filtering
        let decoder = Decoder::new(&network, &spatial).with_config(DecoderConfig {
            candidate_config: CandidateConfig {
                frc_tolerance: 0,
                ..CandidateConfig::default()
            },
            ..DecoderConfig::default()
        });

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
            "Low-class road must be filtered out by LFRCNP"
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
        let result = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints::default(),
        );

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
        let result = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            100.0,
            &AStarConstraints::default(),
        );

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

        let result = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            1000.0,
            &AStarConstraints::default(),
        );
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

        let result = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints::default(),
        );
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

        let result = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            200.0,
            &AStarConstraints {
                max_frc: Some(Frc::Frc3),
                ..AStarConstraints::default()
            },
        )
        .unwrap();
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

        let result = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            100.0,
            &AStarConstraints::default(),
        );
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

        let result = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            10000.0,
            &AStarConstraints::default(),
        );

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
        let result_no_filter = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints::default(),
        );
        assert!(result_no_filter.is_some());
        let (cost_no_filter, _, _) = result_no_filter.unwrap();
        assert!(
            (cost_no_filter - 150.0).abs() < 1.0,
            "Without LFRCNP, should find shorter 150m path, got {}m",
            cost_no_filter
        );

        // With LFRCNP = FRC2: should find path via node 2 (200m), avoiding FRC5 roads
        let result_filtered = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints {
                max_frc: Some(Frc::Frc2),
                ..AStarConstraints::default()
            },
        );
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
        let result = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints {
                max_frc: Some(Frc::Frc7),
                ..AStarConstraints::default()
            },
        );
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
        let result = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints {
                max_frc: Some(Frc::Frc3),
                ..AStarConstraints::default()
            },
        );
        assert!(
            result.is_none(),
            "With LFRCNP=FRC3, path through FRC5 road should be blocked"
        );

        // But without filtering, path should exist
        let result_unfiltered = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints::default(),
        );
        assert!(
            result_unfiltered.is_some(),
            "Without LFRCNP, path should exist"
        );
    }

    // =========================================================================
    // Slip Road Cost Penalty Tests
    // =========================================================================
    //
    // When slip_road_cost_penalty > 0, A* should prefer main roads over slip roads
    // even if the slip road path is slightly shorter.

    /// Test that slip road penalty makes A* prefer main roads
    #[test]
    fn test_bounded_astar_slip_road_penalty_prefers_main_road() {
        // Build a diamond network where:
        // - Upper path (via node 2): SlipRoad - 100m
        // - Lower path (via node 3): SingleCarriageway - 120m (longer but preferred with penalty)
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.001, 0.001) // Slip road path node
            .add_node(3, -0.001, 0.001) // Main road path node
            .add_node(4, 0.0, 0.002)
            // Upper path: SlipRoad (shorter at 100m)
            .add_edge(1, 1, 2, 50.0, Frc::Frc3, Fow::SlipRoad)
            .add_edge(2, 2, 4, 50.0, Frc::Frc3, Fow::SlipRoad)
            // Lower path: SingleCarriageway (longer at 120m)
            .add_edge(3, 1, 3, 60.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(4, 3, 4, 60.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        // Without penalty: should find shorter slip road path (100m)
        let result_no_penalty = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints::default(),
        );
        assert!(result_no_penalty.is_some());
        let (cost_no_penalty, nodes_no_penalty, _) = result_no_penalty.unwrap();
        assert!(
            (cost_no_penalty - 100.0).abs() < 1.0,
            "Without penalty, should find 100m slip road path, got {}m",
            cost_no_penalty
        );

        // With 25m penalty per slip road: slip road path effective cost = 100 + 25*2 = 150m
        // Main road path stays at 120m, so should be preferred
        let result_with_penalty = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints {
                slip_road_cost_penalty: 25.0,
                ..AStarConstraints::default()
            },
        );
        assert!(result_with_penalty.is_some());
        let (cost_with_penalty, nodes_with_penalty, _) = result_with_penalty.unwrap();
        assert!(
            (cost_with_penalty - 120.0).abs() < 1.0,
            "With 50m penalty, should find 120m main road path, got {}m",
            cost_with_penalty
        );

        // Verify paths went through different nodes
        let node2_idx = *network.node_id_to_index.as_ref().unwrap().get(&2).unwrap();
        let node3_idx = *network.node_id_to_index.as_ref().unwrap().get(&3).unwrap();
        assert!(
            nodes_no_penalty.contains(&node2_idx),
            "No penalty path should use slip road (node 2)"
        );
        assert!(
            nodes_with_penalty.contains(&node3_idx),
            "With penalty path should use main road (node 3)"
        );
    }

    /// Test that slip road penalty doesn't affect returned path length (only search priority)
    #[test]
    fn test_bounded_astar_slip_road_penalty_returns_real_length() {
        // Simple network with only slip roads
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.001)
            .add_edge(1, 1, 2, 100.0, Frc::Frc3, Fow::SlipRoad)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&2).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        // Even with penalty, the returned cost should be the REAL path length (100m)
        // Penalty only affects search priority, not the returned path length
        let result = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints {
                slip_road_cost_penalty: 50.0,
                ..AStarConstraints::default()
            },
        );
        assert!(result.is_some());
        let (cost, _, _) = result.unwrap();

        assert!(
            (cost - 100.0).abs() < 1.0,
            "A* should return real path length (100m), not penalized cost, got {}m",
            cost
        );
    }

    /// Test that access road penalty makes A* prefer tertiary over residential
    #[test]
    fn test_bounded_astar_access_road_penalty_prefers_tertiary() {
        // Diamond network:
        // - Upper path (via node 2): access road - 100m
        // - Lower path (via node 3): tertiary - 120m (longer but preferred with penalty)
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.001, 0.001)
            .add_node(3, -0.001, 0.001)
            .add_node(4, 0.0, 0.002)
            // Upper path: access road (shorter at 100m)
            .add_access_road(1, 1, 2, 50.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_access_road(2, 2, 4, 50.0, Frc::Frc4, Fow::SingleCarriageway)
            // Lower path: tertiary (longer at 120m)
            .add_edge(3, 1, 3, 60.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(4, 3, 4, 60.0, Frc::Frc4, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

        // Without penalty: should find shorter access road path (100m)
        let result_no_penalty = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints::default(),
        );
        assert!(result_no_penalty.is_some());
        let (cost_no_penalty, _, _) = result_no_penalty.unwrap();
        assert!(
            (cost_no_penalty - 100.0).abs() < 1.0,
            "Without penalty, should find 100m access road path, got {}m",
            cost_no_penalty
        );

        // With 15m penalty per access road edge: effective cost = 100 + 15*2 = 130m
        // Tertiary path stays at 120m, so should be preferred
        let result_with_penalty = bounded_astar(
            &network,
            start,
            goal,
            goal_coord,
            500.0,
            &AStarConstraints {
                access_road_cost_penalty: 15.0,
                ..AStarConstraints::default()
            },
        );
        assert!(result_with_penalty.is_some());
        let (cost_with_penalty, nodes_with_penalty, _) = result_with_penalty.unwrap();
        assert!(
            (cost_with_penalty - 120.0).abs() < 1.0,
            "With access road penalty, should find 120m tertiary path, got {}m",
            cost_with_penalty
        );

        let node3_idx = *network.node_id_to_index.as_ref().unwrap().get(&3).unwrap();
        assert!(
            nodes_with_penalty.contains(&node3_idx),
            "With penalty, path should use tertiary road (node 3)"
        );
    }

    /// Test that same-edge solutions with excellent spatial match relax the min_distance floor.
    /// This verifies the fix in commit 9e13bcb.
    #[test]
    fn test_same_edge_short_path_relaxed_min_distance() {
        // Build a simple network with one 5m edge
        let (network, spatial) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.000045, 0.0)
            .add_edge(100, 1, 2, 5.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let decoder = Decoder::new(&network, &spatial);

        let edge_idx = *network
            .edge_id_to_index
            .as_ref()
            .unwrap()
            .get(&100)
            .unwrap();

        // Start candidate: near start of edge
        let start_cand = Candidate {
            edge_idx,
            distance_m: 2.0, // < 5.0m "excellent match"
            bearing_diff: 0.0,
            frc_diff: 0,
            fow_score: 1.0,
            score: 0.0,
            projection_fraction: 0.1,
        };

        // End candidate: near end of edge
        let end_cand = Candidate {
            edge_idx,
            distance_m: 2.0, // < 5.0m "excellent match"
            bearing_diff: 0.0,
            frc_diff: 0,
            fow_score: 1.0,
            score: 0.0,
            projection_fraction: 0.9,
        };

        // path_cost = edge.length_m * (0.9 - 0.1) = 5.0 * 0.8 = 4.0m
        // min_distance would normally be 10.0 (default floor)

        let start_candidates = vec![start_cand];
        let end_candidates = vec![end_cand];

        // Use an expected distance of 5.0m
        let result = decoder.find_best_path(&start_candidates, &end_candidates, 5.0, 0, None);

        assert!(
            result.is_ok(),
            "Should find a path even if shorter than 10m floor because of excellent spatial match. Error: {:?}",
            result.err()
        );
        let path = result.unwrap();
        assert_eq!(path.edges.len(), 1);
        assert_eq!(path.edges[0], edge_idx);
        assert!((path.total_length - 4.0).abs() < 0.1);
    }

    /// Network for terminal-snap tests (all edges one-way, west→east):
    ///
    /// ```text
    /// N1 --e1(100m)--> N2 --e2(100m)--> N3 --e3(15m)--> N4 --e4(100m)--> N5
    ///                  ^                                 |
    ///                  |                                 v
    ///                  N6 --e6(20m)                     N7 (e7, 100m)
    /// ```
    ///
    /// N3 is 1-in/1-out (invalid); N2 and N4 are junctions (valid).
    fn snap_network() -> (RoadNetwork, SpatialIndex) {
        TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.001)
            .add_node(3, 0.0, 0.002)
            .add_node(4, 0.0, 0.0021)
            .add_node(5, 0.0, 0.003)
            .add_node(6, -0.0002, 0.001)
            .add_node(7, -0.001, 0.0021)
            .add_edge(1, 1, 2, 100.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(2, 2, 3, 100.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(3, 3, 4, 15.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(4, 4, 5, 100.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(6, 6, 2, 20.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(7, 4, 7, 100.0, Frc::Frc4, Fow::SingleCarriageway)
            .build()
    }

    fn edge_by_id(network: &RoadNetwork, id: u64) -> EdgeIndex {
        *network.edge_id_to_index.as_ref().unwrap().get(&id).unwrap()
    }

    fn ids(network: &RoadNetwork, path: &[EdgeIndex]) -> Vec<u64> {
        path.iter().map(|&i| network.edge(i).unwrap().id).collect()
    }

    #[test]
    fn test_snap_extends_end_to_valid_node_and_folds_into_offset() {
        let (network, spatial) = snap_network();
        let decoder = Decoder::new(&network, &spatial);

        // Path e1,e2 ends at N3 (invalid). Last LRP sits at the end of e2.
        let mut path = vec![edge_by_id(&network, 1), edge_by_id(&network, 2)];
        let n1 = network
            .node(network.edge_source(path[0]).unwrap())
            .unwrap()
            .coord;
        let n3 = network
            .node(network.edge_target(path[1]).unwrap())
            .unwrap()
            .coord;
        let seg = SegmentInfo {
            length_m: 200.0,
            expected_m: 200.0,
        };

        let offsets = decoder.finalize_path(&mut path, n1, n3, seg, seg);

        assert_eq!(
            ids(&network, &path),
            vec![1, 2, 3],
            "e3 (15m) appended to reach N4"
        );
        assert!((offsets.negative_offset_m - 15.0).abs() < 1e-6);
        assert!((offsets.negative_offset_fraction - 1.0).abs() < 1e-6);
        // Start (N1, a source node) is valid — untouched.
        assert!(offsets.positive_offset_m.abs() < 1e-6);
    }

    #[test]
    fn test_snap_respects_absolute_cap() {
        let (network, spatial) = snap_network();
        let config = DecoderConfig {
            max_snap_extension_m: 10.0, // e3 is 15m
            ..DecoderConfig::default()
        };
        let decoder = Decoder::new(&network, &spatial).with_config(config);

        let mut path = vec![edge_by_id(&network, 1), edge_by_id(&network, 2)];
        let n1 = network
            .node(network.edge_source(path[0]).unwrap())
            .unwrap()
            .coord;
        let n3 = network
            .node(network.edge_target(path[1]).unwrap())
            .unwrap()
            .coord;
        let seg = SegmentInfo {
            length_m: 200.0,
            expected_m: 200.0,
        };

        let offsets = decoder.finalize_path(&mut path, n1, n3, seg, seg);
        assert_eq!(ids(&network, &path), vec![1, 2]);
        assert!(offsets.negative_offset_m.abs() < 1e-6);
    }

    #[test]
    fn test_snap_respects_dnp_fraction_and_length_headroom() {
        let (network, spatial) = snap_network();
        let decoder = Decoder::new(&network, &spatial);
        let mut path = vec![edge_by_id(&network, 1), edge_by_id(&network, 2)];
        let n1 = network
            .node(network.edge_source(path[0]).unwrap())
            .unwrap()
            .coord;
        let n3 = network
            .node(network.edge_target(path[1]).unwrap())
            .unwrap()
            .coord;

        // 10% of a 100m DNP = 10m < 15m → no snap
        let short_dnp = SegmentInfo {
            length_m: 200.0,
            expected_m: 100.0,
        };
        decoder.finalize_path(&mut path, n1, n3, short_dnp, short_dnp);
        assert_eq!(ids(&network, &path), vec![1, 2]);

        // Segment already at max_valid_distance (DNP 200 → max 300) → no headroom
        let no_headroom = SegmentInfo {
            length_m: 295.0,
            expected_m: 200.0,
        };
        decoder.finalize_path(&mut path, n1, n3, no_headroom, no_headroom);
        assert_eq!(ids(&network, &path), vec![1, 2]);
    }

    #[test]
    fn test_snap_disabled_by_config() {
        let (network, spatial) = snap_network();
        let config = DecoderConfig {
            snap_to_valid_nodes: false,
            ..DecoderConfig::default()
        };
        let decoder = Decoder::new(&network, &spatial).with_config(config);
        let mut path = vec![edge_by_id(&network, 1), edge_by_id(&network, 2)];
        let n1 = network
            .node(network.edge_source(path[0]).unwrap())
            .unwrap()
            .coord;
        let n3 = network
            .node(network.edge_target(path[1]).unwrap())
            .unwrap()
            .coord;
        let seg = SegmentInfo {
            length_m: 200.0,
            expected_m: 200.0,
        };
        decoder.finalize_path(&mut path, n1, n3, seg, seg);
        assert_eq!(ids(&network, &path), vec![1, 2]);
    }

    #[test]
    fn test_snap_extends_start_to_valid_node() {
        let (network, spatial) = snap_network();
        let decoder = Decoder::new(&network, &spatial);

        // Path e4 starts at N4 (valid). Path e3,e4? e3 starts at N3 (invalid) and
        // its forced predecessor is e2 (100m) — too long for the cap. Use a small
        // cap-friendly case instead: path starting at N3 with a big DNP.
        let mut path = vec![edge_by_id(&network, 3), edge_by_id(&network, 4)];
        let n3 = network
            .node(network.edge_source(path[0]).unwrap())
            .unwrap()
            .coord;
        let n5 = network
            .node(network.edge_target(path[1]).unwrap())
            .unwrap()
            .coord;
        let config = DecoderConfig {
            max_snap_extension_m: 150.0,
            ..DecoderConfig::default()
        };
        let decoder = decoder.with_config(config);
        let seg = SegmentInfo {
            length_m: 115.0,
            expected_m: 1500.0, // 10% = 150m ≥ 100m
        };

        let offsets = decoder.finalize_path(&mut path, n3, n5, seg, seg);
        assert_eq!(
            ids(&network, &path),
            vec![2, 3, 4],
            "e2 prepended to reach junction N2"
        );
        assert!((offsets.positive_offset_m - 100.0).abs() < 1e-6);
        assert!((offsets.positive_offset_fraction - 1.0).abs() < 1e-6);
        // End N5 is a sink (valid) — untouched.
        assert!(offsets.negative_offset_m.abs() < 1e-6);
    }

    #[test]
    fn test_snap_noop_when_terminal_nodes_already_valid() {
        let (network, spatial) = snap_network();
        let decoder = Decoder::new(&network, &spatial);
        // e1 runs N1 (source) → N2 (junction): both valid.
        let mut path = vec![edge_by_id(&network, 1)];
        let n1 = network
            .node(network.edge_source(path[0]).unwrap())
            .unwrap()
            .coord;
        let n2 = network
            .node(network.edge_target(path[0]).unwrap())
            .unwrap()
            .coord;
        let seg = SegmentInfo {
            length_m: 100.0,
            expected_m: 100.0,
        };
        let offsets = decoder.finalize_path(&mut path, n1, n2, seg, seg);
        assert_eq!(ids(&network, &path), vec![1]);
        assert!(offsets.positive_offset_m.abs() < 1e-6);
        assert!(offsets.negative_offset_m.abs() < 1e-6);
    }
}
