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
fn bounded_astar(
    network: &RoadNetwork,
    start: NodeIndex,
    goal: NodeIndex,
    goal_coord: Point<f64>,
    max_cost: f64,
) -> Option<(f64, Vec<NodeIndex>)> {
    let mut open_set = BinaryHeap::new();
    let mut g_scores: HashMap<NodeIndex, f64> = HashMap::new();
    let mut came_from: HashMap<NodeIndex, NodeIndex> = HashMap::new();

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
            let mut path = vec![goal];
            let mut node = goal;
            while let Some(&prev) = came_from.get(&node) {
                path.push(prev);
                node = prev;
            }
            path.reverse();
            return Some((current.g_score, path));
        }

        // Skip if we've already found a better path to this node
        if let Some(&best_g) = g_scores.get(&current.node) {
            if current.g_score > best_g {
                continue;
            }
        }

        // Explore neighbors
        for edge in network.graph.edges(current.node) {
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
            came_from.insert(neighbor, current.node);
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

        // Build path between consecutive LRPs
        let mut full_path: Vec<EdgeIndex> = Vec::new();
        let mut total_length = 0.0;

        for i in 0..points.len() - 1 {
            // Get distance to next point (path attributes are optional for last point)
            let expected_distance = points[i]
                .path
                .as_ref()
                .map(|p| p.dnp.meters())
                .unwrap_or(0.0);

            let (path_segment, segment_length) = self.find_best_path(
                &all_candidates[i],
                &all_candidates[i + 1],
                expected_distance,
                i,
            )?;

            // Add the actual traversed length (accounts for partial edge traversals)
            total_length += segment_length;

            // Add edges to full path (avoid duplicating junction edges)
            for (j, &edge_idx) in path_segment.iter().enumerate() {
                if j == 0 && !full_path.is_empty() {
                    // Skip first edge if it's the same as the last (junction)
                    if full_path.last() == Some(&edge_idx) {
                        continue;
                    }
                }
                full_path.push(edge_idx);
            }
        }

        // Extract offsets
        let positive_offset_m = line.offsets.pos.range();
        let negative_offset_m = line.offsets.neg.range();

        Ok(DecodedPath {
            edge_ids: self.edge_indices_to_ids(&full_path),
            length_m: total_length,
            positive_offset_m,
            negative_offset_m,
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

        let expected_distance = points[0]
            .path
            .as_ref()
            .map(|p| p.dnp.meters())
            .unwrap_or(0.0);

        let (path, total_length) =
            self.find_best_path(&all_candidates[0], &all_candidates[1], expected_distance, 0)?;

        let positive_offset_m = pal.offset.range();

        Ok(DecodedPath {
            edge_ids: self.edge_indices_to_ids(&path),
            length_m: total_length,
            positive_offset_m,
            negative_offset_m: 0.0,
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
    ) -> Option<(Vec<EdgeIndex>, f64)> {
        // Find edges that appear in both candidate lists
        // Only consider candidates with good spatial match (within 10m)
        const MAX_PROJECTION_DISTANCE: f64 = 10.0;

        let mut best_same_edge: Option<(Vec<EdgeIndex>, f64, f64)> = None; // (edges, length, score)

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
                    best_same_edge = Some((vec![start_cand.edge_idx], path_cost, score));
                }
            }
        }

        best_same_edge.map(|(edges, length, _score)| (edges, length))
    }

    /// Find the best path between candidate sets
    /// Returns (edges, actual_length_m)
    fn find_best_path(
        &self,
        start_candidates: &[Candidate],
        end_candidates: &[Candidate],
        expected_distance: f64,
        lrp_index: usize,
    ) -> Result<(Vec<EdgeIndex>, f64), DecodeError> {
        // Compute distance bounds using both relative and absolute tolerance
        // For short segments, absolute tolerance dominates; for long segments, relative dominates
        let rel_min = expected_distance * (1.0 - self.config.length_tolerance);
        let rel_max = expected_distance * (1.0 + self.config.length_tolerance);

        // For minimum: be lenient to handle OpenLR's DNP quantization
        // DNP encoding can significantly overestimate short distances
        // Use the smallest of: relative tolerance, absolute tolerance, or a 10m floor
        let abs_min = (expected_distance - self.config.absolute_length_tolerance).max(0.0);
        let min_distance = rel_min.min(abs_min).max(10.0); // Absolute floor of 10m

        // For maximum: use whichever is STRICTER (limits longer paths)
        let abs_max = expected_distance + self.config.absolute_length_tolerance;
        let max_valid_distance = rel_max.min(abs_max);

        // Maximum distance for A* search - don't explore beyond this
        // Use max_search_distance_factor to bound the search, with a minimum of 500m
        // to handle short segments where 2x might be too restrictive
        let max_search_distance =
            (expected_distance * self.config.max_search_distance_factor).max(500.0);

        // PRIORITY CHECK: Look for same-edge solutions first
        // When both LRPs project closely onto the same edge, prefer this simpler solution
        // over multi-edge paths that may score slightly better on individual candidates
        // but include edges contributing nearly zero length.
        if let Some(result) = self.try_same_edge_solution(
            start_candidates,
            end_candidates,
            expected_distance,
            min_distance,
            max_valid_distance,
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

        let mut best_path: Option<(Vec<EdgeIndex>, f64)> = None;
        let mut best_score = f64::MAX;

        // Try pairs in order of best combined candidate score
        for (start_idx, end_idx, _combined_score) in pairs {
            let start_cand = &start_candidates[start_idx];
            let end_cand = &end_candidates[end_idx];

            // Special case: both LRPs on the same edge
            if start_cand.edge_idx == end_cand.edge_idx {
                if let Some(edge) = self.network.edge(start_cand.edge_idx) {
                    // Calculate distance along edge between projection points
                    let frac_diff = end_cand.projection_fraction - start_cand.projection_fraction;

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
                            let length_diff =
                                (path_cost - expected_distance).abs() / expected_distance.max(1.0);
                            let score = start_cand.score + end_cand.score + length_diff;

                            if score < best_score {
                                best_score = score;
                                best_path = Some((vec![start_cand.edge_idx], path_cost));

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
            let start_edge_partial = self
                .network
                .edge(start_cand.edge_idx)
                .map(|e| e.length_m * (1.0 - start_cand.projection_fraction))
                .unwrap_or(0.0);
            let end_edge_partial = self
                .network
                .edge(end_cand.edge_idx)
                .map(|e| e.length_m * end_cand.projection_fraction)
                .unwrap_or(0.0);

            // Adjust max search for the middle path portion
            let middle_max = max_search_distance - start_edge_partial - end_edge_partial;
            if middle_max <= 0.0 {
                continue;
            }

            // Run bounded A* for the middle portion (between edges)
            let (middle_cost, path_nodes) = if start_node == end_node {
                // Start and end edges share a node - no middle path needed
                (0.0, vec![start_node])
            } else {
                match bounded_astar(self.network, start_node, end_node, end_coord, middle_max) {
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
            let mut edges = vec![start_cand.edge_idx];
            edges.extend(self.nodes_to_edges(&path_nodes));
            if end_cand.edge_idx != start_cand.edge_idx {
                edges.push(end_cand.edge_idx);
            }

            // Score the path (additive for final ranking)
            let length_diff = (path_cost - expected_distance).abs() / expected_distance.max(1.0);
            let score = start_cand.score + end_cand.score + length_diff;

            if score < best_score {
                best_score = score;
                best_path = Some((edges, path_cost));

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

    /// Convert a sequence of nodes to the edges connecting them
    fn nodes_to_edges(&self, nodes: &[NodeIndex]) -> Vec<EdgeIndex> {
        let mut edges = Vec::with_capacity(nodes.len().saturating_sub(1));

        for i in 0..nodes.len().saturating_sub(1) {
            // Find edge between consecutive nodes
            if let Some(edge_ref) = self
                .network
                .graph
                .edges_connecting(nodes[i], nodes[i + 1])
                .next()
            {
                edges.push(edge_ref.id());
            }
        }

        edges
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decoder_config_defaults() {
        let config = DecoderConfig::default();
        assert_eq!(config.length_tolerance, 0.35);
    }
}
