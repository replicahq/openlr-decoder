use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use geo::{Distance, Haversine, Point};
use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use thiserror::Error;

use crate::candidates::{find_candidates, Candidate, CandidateConfig};
use crate::graph::{Fow, Frc, RoadNetwork};
use crate::spatial::{BearingDirection, SpatialIndex};

#[derive(Clone)]
struct AStarNode {
    node: NodeIndex,
    g_score: f64, // penalized cost from start
    f_score: f64, // g_score + straight-line distance to goal
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
        // Reversed so BinaryHeap pops the lowest f_score first.
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

struct AStarConstraints {
    /// LFRCNP filter: skip edges with a higher FRC than this, except slip roads.
    max_frc: Option<Frc>,
    /// Meters added to the search cost of each slip road edge.
    slip_road_cost_penalty: f64,
    /// Meters added to the search cost of each access road edge.
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

/// A* from `start` to `goal` that never extends a path past `max_cost` meters
/// of real length. Returns the real length, the nodes and the edges.
///
/// `max_frc` is the LFRCNP filter (whitepaper §12.1 Step 5). The penalties
/// order the search but are not part of the returned length, so with nonzero
/// penalties the path may be slightly longer than the shortest one.
///
/// Dominance uses penalized cost while the bound uses real length, so a route
/// that is cheaper only because of penalties can displace one that would have
/// fit under `max_cost`. A multi-label search would fix this, but on the KC
/// corpus penalties change 3 of 47,931 paths and lifting the bound changes
/// none, so it isn't worth the complexity.
fn bounded_astar(
    network: &RoadNetwork,
    start: NodeIndex,
    goal: NodeIndex,
    goal_coord: Point<f64>,
    max_cost: f64,
    constraints: &AStarConstraints,
) -> Option<(f64, Vec<NodeIndex>, Vec<EdgeIndex>)> {
    let mut open_set = BinaryHeap::new();
    // Real length decides the bound; penalized cost decides priority and dominance.
    let mut g_scores: HashMap<NodeIndex, f64> = HashMap::new();
    let mut penalized_scores: HashMap<NodeIndex, f64> = HashMap::new();
    let mut came_from: HashMap<NodeIndex, (NodeIndex, EdgeIndex)> = HashMap::new();

    let start_h = network
        .node(start)
        .map(|n| Haversine.distance(n.coord, goal_coord))
        .unwrap_or(0.0);

    g_scores.insert(start, 0.0);
    penalized_scores.insert(start, 0.0);
    open_set.push(AStarNode {
        node: start,
        g_score: 0.0,
        f_score: start_h,
    });

    while let Some(current) = open_set.pop() {
        if current.node == goal {
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
            let real_cost = g_scores.get(&goal).copied().unwrap_or(0.0);
            return Some((real_cost, nodes, edges));
        }

        // Stale heap entry.
        if let Some(&best_penalized) = penalized_scores.get(&current.node) {
            if current.g_score > best_penalized {
                continue;
            }
        }

        for edge in network.graph.edges(current.node) {
            // Slip roads are exempt: the loader puts motorway_link at Frc3, but a
            // motorway-to-motorway LFRCNP of Frc0 must still be able to use it.
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

            if tentative_real_g > max_cost {
                continue;
            }

            let dominated = penalized_scores
                .get(&neighbor)
                .map(|&g| tentative_penalized_g >= g)
                .unwrap_or(false);
            if dominated {
                continue;
            }

            came_from.insert(neighbor, (current.node, edge.id()));
            g_scores.insert(neighbor, tentative_real_g);
            penalized_scores.insert(neighbor, tentative_penalized_g);

            let h = network
                .node(neighbor)
                .map(|n| Haversine.distance(n.coord, goal_coord))
                .unwrap_or(0.0);

            open_set.push(AStarNode {
                node: neighbor,
                g_score: tentative_penalized_g,
                f_score: tentative_penalized_g + h,
            });
        }
    }

    None
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

#[derive(Debug, Clone)]
pub struct DecodedPath {
    /// Stable edge ids, in travel order.
    pub edge_ids: Vec<u64>,
    pub edge_lengths_m: Vec<f64>,
    /// Meters from the first LRP's projection to the last's. This includes the
    /// partial terminal edges that were too short to keep in `edge_ids` and
    /// excludes any terminal-node snap.
    pub length_m: f64,
    /// Meters to trim from the start of the first edge to reach the first LRP.
    pub positive_offset_m: f64,
    /// Meters to trim from the end of the last edge to reach the last LRP.
    pub negative_offset_m: f64,
    /// `positive_offset_m` over the first edge's length. Exceeds 1.0 when a
    /// terminal-node snap prepended edges; the meter value is authoritative.
    pub positive_offset_fraction: f64,
    /// `negative_offset_m` over the last edge's length. Exceeds 1.0 when a
    /// terminal-node snap appended edges.
    pub negative_offset_fraction: f64,
    /// The edge covering the most meters of the LRP-to-LRP path. Snap
    /// extensions don't count.
    pub primary_edge_id: u64,
    pub primary_edge_coverage_m: f64,
}

/// Meters of one edge that a path segment actually traverses.
#[derive(Debug, Clone)]
struct EdgeCoverage {
    edge_idx: EdgeIndex,
    coverage_m: f64,
}

/// Path between one pair of LRPs.
#[derive(Debug, Clone)]
struct PathResult {
    edges: Vec<EdgeIndex>,
    coverages: Vec<EdgeCoverage>,
    total_length: f64, // meters, LRP projection to LRP projection
    /// Meters from the start of `edges[0]` to the first LRP's projection.
    start_offset_m: f64,
    /// Meters from the second LRP's projection to the end of the last edge.
    end_offset_m: f64,
}

/// One LRP-to-LRP segment's lengths and offsets, all in meters. Used to bound
/// terminal snaps.
#[derive(Debug, Clone, Copy, Default)]
struct SegmentInfo {
    length_m: f64,   // decoded
    expected_m: f64, // DNP
    start_offset_m: f64,
    end_offset_m: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct PathOffsets {
    positive_offset_m: f64,
    negative_offset_m: f64,
    positive_offset_fraction: f64,
    negative_offset_fraction: f64,
}

#[derive(Debug, Clone)]
pub struct DecoderConfig {
    pub candidate_config: CandidateConfig,
    /// Relative path length tolerance around the DNP, as a fraction. The
    /// accepted range takes the looser of this and `absolute_length_tolerance`.
    pub length_tolerance: f64,
    /// Absolute path length tolerance around the DNP, in meters.
    pub absolute_length_tolerance: f64,
    /// Paths are searched up to this multiple of the DNP, and at least 500 m.
    pub max_search_distance_factor: f64,
    /// Meters added to A* cost per slip road edge, so the search stays on the
    /// main road when the two routes are similar in length.
    pub slip_road_cost_penalty: f64,
    /// Meters added to A* cost per access road edge (see
    /// [`Frc::is_access_road`]), so tertiary beats residential.
    pub access_road_cost_penalty: f64,
    /// Extend each end of the path along forced continuations to the nearest
    /// valid node (OpenLR Rule 4). The added length goes into the offsets, so
    /// the trimmed geometry doesn't change.
    pub snap_to_valid_nodes: bool,
    /// Meters a terminal-node snap may add at each end. Not scaled by DNP:
    /// HERE links are often only 20–40 m long and still stop a ~7–10 m
    /// crossing stub short of the junction.
    pub max_snap_extension_m: f64,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        DecoderConfig {
            candidate_config: CandidateConfig::default(),
            length_tolerance: 0.35,
            absolute_length_tolerance: 100.0,
            max_search_distance_factor: 2.0,
            slip_road_cost_penalty: 20.0,
            access_road_cost_penalty: 10.0,
            snap_to_valid_nodes: true,
            max_snap_extension_m: 25.0,
        }
    }
}

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

    /// Longest acceptable decoded length for a segment with this DNP, using
    /// whichever of the relative and absolute tolerances is looser.
    fn max_valid_distance(&self, expected_distance: f64) -> f64 {
        let rel_max = expected_distance * (1.0 + self.config.length_tolerance);
        let abs_max = expected_distance + self.config.absolute_length_tolerance;
        rel_max.max(abs_max)
    }

    /// Meters a terminal-node snap may add next to `seg`: the configured cap,
    /// limited by the room left under [`max_valid_distance`](Self::max_valid_distance).
    /// 0 when snapping is disabled.
    fn snap_cap_m(&self, seg: SegmentInfo) -> f64 {
        if !self.config.snap_to_valid_nodes {
            return 0.0;
        }
        let cap = self.config.max_snap_extension_m;
        let headroom = self.max_valid_distance(seg.expected_m) - seg.length_m;
        cap.min(headroom).max(0.0)
    }

    /// Follow forced continuations from the end of `path` to the first valid
    /// node. Returns the edges to append and their length. `None` if the end is
    /// already valid, the walk would revisit an edge, or it can't reach a valid
    /// node within `cap_m` (stopping partway would still end on an invalid node).
    fn forward_extension(&self, path: &[EdgeIndex], cap_m: f64) -> Option<(Vec<EdgeIndex>, f64)> {
        let mut incoming = *path.last()?;
        let mut node = self.network.edge_target(incoming)?;
        let mut added = Vec::new();
        let mut added_m = 0.0;
        while let Some(next) = self.network.forced_continuation(node, incoming) {
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

    /// [`forward_extension`](Self::forward_extension) from the start of `path`,
    /// walking backward. The edges come back in travel order, ready to prepend.
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

    /// Snap the ends of `path` to valid nodes and compute its offsets.
    ///
    /// The offsets are the LRP projections onto the terminal edges, not the
    /// encoded offsets of whitepaper §12.10: no HERE code in the KC corpus has
    /// one, and edge-relative offsets are what OSM consumers need.
    fn finalize_path(
        &self,
        path: &mut Vec<EdgeIndex>,
        first_lrp_coord: Point<f64>,
        last_lrp_coord: Point<f64>,
        first_segment: SegmentInfo,
        last_segment: SegmentInfo,
    ) -> PathOffsets {
        if path.is_empty() {
            return PathOffsets::default();
        }

        // Take the offsets from path finding rather than re-projecting the LRPs.
        // When a negligible terminal edge was dropped, the LRP lies beyond the
        // new terminal edge, its projection clamps to the far end, and the
        // offsets could meet or cross.
        let mut positive_offset_m = first_segment.start_offset_m;
        let mut negative_offset_m = last_segment.end_offset_m;

        // Only extend when the LRP is closer to the new terminal node than to the
        // current one. Otherwise HERE probably has a junction our OSM car graph
        // lacks, and extending would overshoot. On the KC corpus this removed all
        // ~100 overshoots and kept every extension that landed on HERE's endpoint.
        let lrp_prefers =
            |lrp: Point<f64>, old_node: Option<NodeIndex>, new_node: Option<NodeIndex>| match (
                old_node.and_then(|n| self.network.node(n)),
                new_node.and_then(|n| self.network.node(n)),
            ) {
                (Some(old), Some(new)) => {
                    Haversine.distance(lrp, new.coord) < Haversine.distance(lrp, old.coord)
                }
                _ => false,
            };

        if let Some((tail, added_m)) = self.forward_extension(path, self.snap_cap_m(last_segment)) {
            let old_node = path.last().and_then(|&e| self.network.edge_target(e));
            let new_node = tail.last().and_then(|&e| self.network.edge_target(e));
            if lrp_prefers(last_lrp_coord, old_node, new_node) {
                path.extend(tail);
                negative_offset_m += added_m;
            }
        }
        if let Some((head, added_m)) = self.backward_extension(path, self.snap_cap_m(first_segment))
        {
            let old_node = path.first().and_then(|&e| self.network.edge_source(e));
            let new_node = head.first().and_then(|&e| self.network.edge_source(e));
            if lrp_prefers(first_lrp_coord, old_node, new_node) {
                path.splice(0..0, head);
                positive_offset_m += added_m;
            }
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

    fn edge_ids_and_lengths(&self, indices: &[EdgeIndex]) -> (Vec<u64>, Vec<f64>) {
        indices
            .iter()
            .filter_map(|&idx| self.network.edge(idx).map(|e| (e.id, e.length_m)))
            .unzip()
    }

    /// Decode a base64 OpenLR string. Only Line and PointAlongLine are supported.
    pub fn decode(&self, openlr_base64: &str) -> Result<DecodedPath, DecodeError> {
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

    fn decode_line(&self, line: openlr::Line) -> Result<DecodedPath, DecodeError> {
        let points = &line.points;
        if points.is_empty() {
            return Err(DecodeError::Deserialize("Empty LRP list".to_string()));
        }

        let mut all_candidates: Vec<Vec<Candidate>> = Vec::with_capacity(points.len());

        for (i, point) in points.iter().enumerate() {
            let coord = Point::new(point.coordinate.lon, point.coordinate.lat);
            let bearing = point.line.bearing.degrees() as f64;
            let frc = Frc::from_u8(point.line.frc.value() as u8);
            let fow = Fow::from_u8(point.line.fow.value() as u8);

            let direction = if i == points.len() - 1 {
                BearingDirection::Backward
            } else {
                BearingDirection::Forward
            };
            let candidates = find_candidates(
                coord,
                bearing,
                frc,
                fow,
                direction,
                self.network,
                self.spatial,
                &self.config.candidate_config,
            );

            if candidates.is_empty() {
                return Err(DecodeError::NoCandidates { index: i });
            }

            all_candidates.push(candidates);
        }

        // Only one LRP needs a candidate within max_candidate_distance_m, so a
        // location that runs slightly off our network can still decode.
        let max_dist = self.config.candidate_config.max_candidate_distance_m;
        let has_close_candidate = all_candidates
            .iter()
            .any(|candidates| candidates.iter().any(|c| c.distance_m <= max_dist));

        if !has_close_candidate {
            // Report the LRP whose nearest candidate is closest.
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

        let mut full_path: Vec<EdgeIndex> = Vec::new();
        let mut total_length = 0.0;
        // Summed per edge, in first-traversal order so primary-edge ties resolve
        // deterministically.
        let mut edge_coverages: Vec<(EdgeIndex, f64)> = Vec::new();
        let mut first_segment = SegmentInfo::default();
        let mut last_segment = SegmentInfo::default();

        for i in 0..points.len() - 1 {
            let expected_distance = points[i]
                .path
                .as_ref()
                .map(|p| p.dnp.meters())
                .unwrap_or(0.0);

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

            total_length += path_result.total_length;
            let seg = SegmentInfo {
                length_m: path_result.total_length,
                expected_m: expected_distance,
                start_offset_m: path_result.start_offset_m,
                end_offset_m: path_result.end_offset_m,
            };
            if i == 0 {
                first_segment = seg;
            }
            last_segment = seg;

            for coverage in &path_result.coverages {
                match edge_coverages
                    .iter_mut()
                    .find(|(idx, _)| *idx == coverage.edge_idx)
                {
                    Some((_, total)) => *total += coverage.coverage_m,
                    None => edge_coverages.push((coverage.edge_idx, coverage.coverage_m)),
                }
            }

            // Consecutive segments share the edge under their common LRP.
            for (j, &edge_idx) in path_result.edges.iter().enumerate() {
                if j == 0 && full_path.last() == Some(&edge_idx) {
                    continue;
                }
                full_path.push(edge_idx);
            }
        }

        // Ties go to the earliest edge.
        let (primary_edge_idx, primary_coverage) = edge_coverages
            .iter()
            .copied()
            .reduce(|best, c| if c.1 > best.1 { c } else { best })
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

        let (edge_ids, edge_lengths_m) = self.edge_ids_and_lengths(&full_path);
        Ok(DecodedPath {
            edge_ids,
            edge_lengths_m,
            length_m: total_length,
            positive_offset_m: offsets.positive_offset_m,
            negative_offset_m: offsets.negative_offset_m,
            positive_offset_fraction: offsets.positive_offset_fraction,
            negative_offset_fraction: offsets.negative_offset_fraction,
            primary_edge_id,
            primary_edge_coverage_m: primary_coverage,
        })
    }

    /// Decode the line between the two LRPs. The point's offset, side of road
    /// and orientation are ignored.
    fn decode_point_along_line(
        &self,
        pal: openlr::PointAlongLine,
    ) -> Result<DecodedPath, DecodeError> {
        let points = &pal.points;

        let mut all_candidates: Vec<Vec<Candidate>> = Vec::with_capacity(2);

        for (i, point) in points.iter().enumerate() {
            let coord = Point::new(point.coordinate.lon, point.coordinate.lat);
            let bearing = point.line.bearing.degrees() as f64;
            let frc = Frc::from_u8(point.line.frc.value() as u8);
            let fow = Fow::from_u8(point.line.fow.value() as u8);

            let direction = if i == points.len() - 1 {
                BearingDirection::Backward
            } else {
                BearingDirection::Forward
            };
            let candidates = find_candidates(
                coord,
                bearing,
                frc,
                fow,
                direction,
                self.network,
                self.spatial,
                &self.config.candidate_config,
            );

            if candidates.is_empty() {
                return Err(DecodeError::NoCandidates { index: i });
            }

            all_candidates.push(candidates);
        }

        // Same close-candidate check as decode_line.
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
            start_offset_m: path_result.start_offset_m,
            end_offset_m: path_result.end_offset_m,
        };
        let offsets = self.finalize_path(&mut full_path, first_lrp_coord, last_lrp_coord, seg, seg);

        let (edge_ids, edge_lengths_m) = self.edge_ids_and_lengths(&full_path);
        Ok(DecodedPath {
            edge_ids,
            edge_lengths_m,
            length_m: path_result.total_length,
            positive_offset_m: offsets.positive_offset_m,
            negative_offset_m: offsets.negative_offset_m,
            positive_offset_fraction: offsets.positive_offset_fraction,
            negative_offset_fraction: offsets.negative_offset_fraction,
            primary_edge_id,
            primary_edge_coverage_m: primary_coverage,
        })
    }

    /// Best single-edge path where both LRPs project within 10 m of the same
    /// edge, start before end.
    fn try_same_edge_solution(
        &self,
        start_candidates: &[Candidate],
        end_candidates: &[Candidate],
        expected_distance: f64,
        min_distance: f64,
        max_valid_distance: f64,
        max_frc: Option<Frc>,
    ) -> Option<PathResult> {
        const MAX_PROJECTION_DISTANCE: f64 = 10.0;

        // (edge_idx, length, start_offset_m, end_offset_m, score)
        let mut best_same_edge: Option<(EdgeIndex, f64, f64, f64, f64)> = None;

        for start_cand in start_candidates.iter().take(10) {
            if start_cand.distance_m > MAX_PROJECTION_DISTANCE {
                continue;
            }

            for end_cand in end_candidates.iter().take(10) {
                if end_cand.distance_m > MAX_PROJECTION_DISTANCE {
                    continue;
                }

                if start_cand.edge_idx != end_cand.edge_idx {
                    continue;
                }

                let edge = self.network.edge(start_cand.edge_idx)?;

                // LFRCNP, with slip roads exempt as in bounded_astar.
                if let Some(max) = max_frc {
                    let is_slip_road = edge.fow == Fow::SlipRoad;
                    if edge.frc > max && !is_slip_road {
                        continue;
                    }
                }

                let frac_diff = end_cand.projection_fraction - start_cand.projection_fraction;
                if frac_diff <= 0.0 {
                    continue;
                }

                let path_cost = edge.length_m * frac_diff;

                // With both LRPs within 5 m of the edge, trust the geometry over
                // the DNP: drop the minimum and triple the maximum. DNP quantization
                // badly overestimates short distances.
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

                if best_same_edge.is_none() || score < best_same_edge.as_ref().unwrap().4 {
                    best_same_edge = Some((
                        start_cand.edge_idx,
                        path_cost,
                        edge.length_m * start_cand.projection_fraction,
                        edge.length_m * (1.0 - end_cand.projection_fraction),
                        score,
                    ));
                }
            }
        }

        best_same_edge.map(
            |(edge_idx, length, start_offset_m, end_offset_m, _score)| PathResult {
                edges: vec![edge_idx],
                coverages: vec![EdgeCoverage {
                    edge_idx,
                    coverage_m: length,
                }],
                total_length: length,
                start_offset_m,
                end_offset_m,
            },
        )
    }

    /// Best path from one of `start_candidates` to one of `end_candidates` whose
    /// length fits `expected_distance` (the DNP). `lfrcnp` is widened by
    /// `frc_tolerance`, since HERE and OSM classes don't line up. `lrp_index`
    /// only labels the error.
    fn find_best_path(
        &self,
        start_candidates: &[Candidate],
        end_candidates: &[Candidate],
        expected_distance: f64,
        lrp_index: usize,
        lfrcnp: Option<Frc>,
    ) -> Result<PathResult, DecodeError> {
        let rel_min = expected_distance * (1.0 - self.config.length_tolerance);
        let rel_max = expected_distance * (1.0 + self.config.length_tolerance);

        // The looser of the two minimums. DNP quantization overestimates short
        // distances, so the minimum is lenient.
        let abs_min = (expected_distance - self.config.absolute_length_tolerance).max(0.0);
        let min_distance = rel_min.min(abs_min);

        let max_valid_distance = self.max_valid_distance(expected_distance);
        debug_assert!(max_valid_distance >= rel_max);

        let max_search_distance =
            (expected_distance * self.config.max_search_distance_factor).max(500.0);

        let frc_tol = self.config.candidate_config.frc_tolerance;
        let max_path_frc = lfrcnp.map(|frc| Frc::from_u8((frc as u8).saturating_add(frc_tol)));

        // A close same-edge match wins outright. Multi-edge alternatives can
        // score slightly better per candidate but add edges that contribute
        // almost no length.
        if let Some(result) = self.try_same_edge_solution(
            start_candidates,
            end_candidates,
            expected_distance,
            min_distance,
            max_valid_distance,
            max_path_frc,
        ) {
            return Ok(result);
        }

        // Try pairs in order of (score + 1) products, which ranks a pair down
        // hard when either candidate is poor. The +1 keeps a perfect 0 score from
        // zeroing the product.
        let mut pairs: Vec<(usize, usize, f64)> = Vec::with_capacity(100);
        for (i, start_cand) in start_candidates.iter().enumerate().take(10) {
            for (j, end_cand) in end_candidates.iter().enumerate().take(10) {
                let combined = (start_cand.score + 1.0) * (end_cand.score + 1.0);
                pairs.push((i, j, combined));
            }
        }
        pairs.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

        let mut best_path: Option<PathResult> = None;
        let mut best_score = f64::MAX;

        for (start_idx, end_idx, _combined_score) in pairs.iter().copied() {
            let start_cand = &start_candidates[start_idx];
            let end_cand = &end_candidates[end_idx];

            let (start_edge, end_edge) = match (
                self.network.edge(start_cand.edge_idx),
                self.network.edge(end_cand.edge_idx),
            ) {
                (Some(s), Some(e)) => (s, e),
                _ => continue,
            };

            // LFRCNP, with slip roads exempt as in bounded_astar.
            let start_is_slip_road = start_edge.fow == Fow::SlipRoad;
            let end_is_slip_road = end_edge.fow == Fow::SlipRoad;
            let max_allowed_frc = max_path_frc.unwrap_or(Frc::Frc7);

            if start_edge.frc > max_allowed_frc && !start_is_slip_road {
                continue;
            }
            if end_edge.frc > max_allowed_frc && !end_is_slip_road {
                continue;
            }

            // Same edge, start before end. Same rules as try_same_edge_solution,
            // which has already covered the pairs within 10 m.
            if start_cand.edge_idx == end_cand.edge_idx {
                if let Some(edge) = self.network.edge(start_cand.edge_idx) {
                    let frac_diff = end_cand.projection_fraction - start_cand.projection_fraction;

                    if frac_diff > 0.0 {
                        let path_cost = edge.length_m * frac_diff;

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
                                    start_offset_m: edge.length_m * start_cand.projection_fraction,
                                    end_offset_m: edge.length_m
                                        * (1.0 - end_cand.projection_fraction),
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

            // Route from the start edge's target to the end edge's source.
            let start_node = match self.network.edge_target(start_cand.edge_idx) {
                Some(n) => n,
                None => continue,
            };
            let end_node = match self.network.edge_source(end_cand.edge_idx) {
                Some(n) => n,
                None => continue,
            };

            let end_coord = match self.network.node(end_node) {
                Some(n) => n.coord,
                None => continue,
            };

            let start_edge_partial = start_edge.length_m * (1.0 - start_cand.projection_fraction);
            let end_edge_partial = end_edge.length_m * end_cand.projection_fraction;

            let middle_max = max_search_distance - start_edge_partial - end_edge_partial;
            if middle_max <= 0.0 {
                continue;
            }

            let (middle_cost, _path_nodes, middle_edges) = if start_node == end_node {
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

            let path_cost = start_edge_partial + middle_cost + end_edge_partial;

            // Zero length means both LRPs clamped to the same node; every edge
            // would fall under MIN_EDGE_CONTRIBUTION below and leave no path.
            if path_cost <= 0.0 || path_cost < min_distance || path_cost > max_valid_distance {
                continue;
            }

            // Drop a terminal edge that contributes under 3% of the path, which
            // happens when its LRP sits at a junction. Its partial still counts
            // in path_cost, and the path then starts (ends) at offset 0 on the
            // next edge in.
            const MIN_EDGE_CONTRIBUTION: f64 = 0.03;
            let mut edges = Vec::new();
            let mut coverages = Vec::new();
            let mut start_offset_m = 0.0;
            let mut end_offset_m = 0.0;

            if start_edge_partial / path_cost >= MIN_EDGE_CONTRIBUTION {
                start_offset_m = start_edge.length_m * start_cand.projection_fraction;
                edges.push(start_cand.edge_idx);
                coverages.push(EdgeCoverage {
                    edge_idx: start_cand.edge_idx,
                    coverage_m: start_edge_partial,
                });
            }

            for &edge_idx in &middle_edges {
                if let Some(edge) = self.network.edge(edge_idx) {
                    edges.push(edge_idx);
                    coverages.push(EdgeCoverage {
                        edge_idx,
                        coverage_m: edge.length_m,
                    });
                }
            }

            if end_edge_partial / path_cost >= MIN_EDGE_CONTRIBUTION {
                end_offset_m = end_edge.length_m * (1.0 - end_cand.projection_fraction);
                edges.push(end_cand.edge_idx);
                coverages.push(EdgeCoverage {
                    edge_idx: end_cand.edge_idx,
                    coverage_m: end_edge_partial,
                });
            }

            // Final ranking is additive, unlike the multiplicative pair order.
            let length_diff = (path_cost - expected_distance).abs() / expected_distance.max(1.0);
            let score = start_cand.score + end_cand.score + length_diff;

            if score < best_score {
                best_score = score;
                best_path = Some(PathResult {
                    edges,
                    coverages,
                    total_length: path_cost,
                    start_offset_m,
                    end_offset_m,
                });

                // Pairs come best-first, so stop at the first improvement whose
                // length is within 10% of the DNP.
                if length_diff < 0.1 {
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

    /// Offsets are relative to the terminal edges path finding keeps. A dropped
    /// terminal edge means offset 0 on the next edge in, not a projection that
    /// clamps to the far end and lets the offsets cross.
    #[test]
    fn test_find_best_path_offsets_follow_kept_terminal_edges() {
        let (network, spatial) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.001)
            .add_node(3, 0.0, 0.002)
            .add_node(4, 0.0, 0.003)
            .add_edge(1, 1, 2, 100.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(2, 2, 3, 46.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(3, 3, 4, 100.0, Frc::Frc4, Fow::SingleCarriageway)
            .build();
        let decoder = Decoder::new(&network, &spatial);
        let idx = |id| *network.edge_id_to_index.as_ref().unwrap().get(&id).unwrap();
        let candidate = |edge_idx, projection_fraction| Candidate {
            edge_idx,
            distance_m: 0.0,
            bearing_diff: 0.0,
            frc_diff: 0,
            fow_score: 1.0,
            score: 0.0,
            projection_fraction,
        };

        // Both LRPs sit at the junction ends of e1 and e3, so only e2 is kept.
        let result = decoder
            .find_best_path(
                &[candidate(idx(1), 0.999)],
                &[candidate(idx(3), 0.001)],
                46.0,
                0,
                None,
            )
            .unwrap();
        assert_eq!(result.edges, vec![idx(2)]);
        assert_eq!(result.start_offset_m, 0.0);
        assert_eq!(result.end_offset_m, 0.0);

        // Kept terminal edges carry their projection offsets.
        let result = decoder
            .find_best_path(
                &[candidate(idx(1), 0.5)],
                &[candidate(idx(3), 0.25)],
                121.0,
                0,
                None,
            )
            .unwrap();
        assert_eq!(result.edges, vec![idx(1), idx(2), idx(3)]);
        assert!((result.start_offset_m - 50.0).abs() < 1e-9);
        assert!((result.end_offset_m - 75.0).abs() < 1e-9);
    }

    #[test]
    fn test_find_best_path_prefers_allowed_start_edge() {
        let (network, spatial) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.001)
            .add_node(3, 0.0, 0.002)
            .add_edge(100, 1, 2, 20.0, Frc::Frc7, Fow::SingleCarriageway)
            .add_edge(101, 1, 2, 20.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(102, 2, 3, 20.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        // frc_tolerance 0 so LFRCNP is applied exactly.
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
            score: 0.1, // better than allowed_candidate
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

    // bounded_astar tests. The first few follow the Java OpenLR decoder's
    // RouteSearchTest.

    /// Java testValidRoute.
    #[test]
    fn test_bounded_astar_finds_valid_path() {
        // 1 -> 2 -> 3, 100 m + 150 m.
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

    /// Java testNoRouteFound: a 250 m path with max_cost 100 m.
    #[test]
    fn test_bounded_astar_no_route_when_max_cost_exceeded() {
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

    #[test]
    fn test_bounded_astar_returns_correct_cost() {
        // Edge lengths match Java test edges 4, 6 and 8.
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 49.60, 6.120)
            .add_node(2, 49.60, 6.121)
            .add_node(3, 49.60, 6.122)
            .add_node(4, 49.60, 6.123)
            .add_edge(1, 1, 2, 172.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(2, 2, 3, 90.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(3, 3, 4, 50.0, Frc::Frc3, Fow::SingleCarriageway)
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
        assert!(
            (cost - 312.0).abs() < 1.0,
            "Path cost should be 312m, got {}",
            cost
        );
        assert_eq!(nodes.len(), 4, "Path should traverse all 4 nodes");
    }

    #[test]
    fn test_bounded_astar_chooses_shortest_path() {
        //     2
        //    / \
        //   1   4      via 2: 200 m, via 3: 300 m
        //    \ /
        //     3
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.001, 0.001)
            .add_node(3, -0.001, 0.001)
            .add_node(4, 0.0, 0.002)
            .add_edge(1, 1, 2, 100.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(2, 2, 4, 100.0, Frc::Frc3, Fow::SingleCarriageway)
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
        assert!(
            (cost - 200.0).abs() < 1.0,
            "Should find shortest path (200m), got {}m",
            cost
        );
        assert_eq!(nodes.len(), 3, "Shortest path has 3 nodes");

        let node2_idx = *network.node_id_to_index.as_ref().unwrap().get(&2).unwrap();
        assert!(
            nodes.contains(&node2_idx),
            "Path should go through node 2 (shorter route)"
        );
    }

    /// With parallel edges between two nodes, the returned edges must be the
    /// ones actually traversed, not just any edge between consecutive nodes.
    #[test]
    fn test_bounded_astar_returns_actual_edge_sequence() {
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.001)
            .add_node(3, 0.0, 0.002)
            .add_edge(100, 1, 2, 20.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(101, 1, 2, 20.0, Frc::Frc4, Fow::SingleCarriageway) // fails LFRCNP
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

    #[test]
    fn test_bounded_astar_same_start_and_goal() {
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 49.60, 6.120)
            .add_node(2, 49.60, 6.121)
            .add_edge(1, 1, 2, 100.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = start;
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

    #[test]
    fn test_bounded_astar_disconnected_nodes() {
        // 1 -> 2 and 3 -> 4, not connected.
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 49.60, 6.120)
            .add_node(2, 49.60, 6.121)
            .add_node(3, 49.60, 6.130)
            .add_node(4, 49.60, 6.131)
            .add_edge(1, 1, 2, 100.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(2, 3, 4, 100.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap();
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

    // LFRCNP filtering (whitepaper §12.1 Step 5).

    #[test]
    fn test_bounded_astar_lfrcnp_blocks_low_importance_roads() {
        // Diamond: via node 2 is Frc1 and 200 m, via node 3 is Frc5 and 150 m.
        // LFRCNP Frc2 must force the longer route.
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.001, 0.001)
            .add_node(3, -0.001, 0.001)
            .add_node(4, 0.0, 0.002)
            .add_edge(1, 1, 2, 100.0, Frc::Frc1, Fow::SingleCarriageway)
            .add_edge(2, 2, 4, 100.0, Frc::Frc1, Fow::SingleCarriageway)
            .add_edge(3, 1, 3, 75.0, Frc::Frc5, Fow::SingleCarriageway)
            .add_edge(4, 3, 4, 75.0, Frc::Frc5, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

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

        let node2_idx = *network.node_id_to_index.as_ref().unwrap().get(&2).unwrap();
        let node3_idx = *network.node_id_to_index.as_ref().unwrap().get(&3).unwrap();
        assert!(nodes.contains(&node2_idx), "Path should go through node 2");
        assert!(
            !nodes.contains(&node3_idx),
            "Path should NOT go through node 3"
        );
    }

    #[test]
    fn test_bounded_astar_lfrcnp_frc7_allows_all() {
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.001, 0.001)
            .add_node(3, -0.001, 0.001)
            .add_node(4, 0.0, 0.002)
            .add_edge(1, 1, 2, 100.0, Frc::Frc1, Fow::SingleCarriageway)
            .add_edge(2, 2, 4, 100.0, Frc::Frc1, Fow::SingleCarriageway)
            .add_edge(3, 1, 3, 75.0, Frc::Frc7, Fow::SingleCarriageway)
            .add_edge(4, 3, 4, 75.0, Frc::Frc7, Fow::SingleCarriageway)
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

    #[test]
    fn test_bounded_astar_lfrcnp_no_valid_path() {
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.001)
            .add_edge(1, 1, 2, 100.0, Frc::Frc5, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&2).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

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

    // Cost penalties.

    #[test]
    fn test_bounded_astar_slip_road_penalty_prefers_main_road() {
        // Diamond: via node 2 is slip road and 100 m, via node 3 is main road
        // and 120 m.
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.001, 0.001)
            .add_node(3, -0.001, 0.001)
            .add_node(4, 0.0, 0.002)
            .add_edge(1, 1, 2, 50.0, Frc::Frc3, Fow::SlipRoad)
            .add_edge(2, 2, 4, 50.0, Frc::Frc3, Fow::SlipRoad)
            .add_edge(3, 1, 3, 60.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(4, 3, 4, 60.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

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

        // 25 m on each of two slip road edges makes that route cost 150 m.
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

    /// Penalties order the search but don't leak into the returned length.
    #[test]
    fn test_bounded_astar_slip_road_penalty_returns_real_length() {
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.001)
            .add_edge(1, 1, 2, 100.0, Frc::Frc3, Fow::SlipRoad)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&2).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

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

    #[test]
    fn test_bounded_astar_access_road_penalty_prefers_tertiary() {
        // Diamond: via node 2 is access road and 100 m, via node 3 is tertiary
        // and 120 m.
        let (network, _) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.001, 0.001)
            .add_node(3, -0.001, 0.001)
            .add_node(4, 0.0, 0.002)
            .add_access_road(1, 1, 2, 50.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_access_road(2, 2, 4, 50.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(3, 1, 3, 60.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(4, 3, 4, 60.0, Frc::Frc4, Fow::SingleCarriageway)
            .build();

        let start = *network.node_id_to_index.as_ref().unwrap().get(&1).unwrap();
        let goal = *network.node_id_to_index.as_ref().unwrap().get(&4).unwrap();
        let goal_coord = network.node(goal).unwrap().coord;

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

        // 15 m on each of two access road edges makes that route cost 130 m.
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

    fn short_candidate(edge_idx: EdgeIndex, projection_fraction: f64) -> Candidate {
        Candidate {
            edge_idx,
            distance_m: 7.0, // too far for the same-edge "excellent match" relaxation
            bearing_diff: 0.0,
            frc_diff: 0,
            fow_score: 1.0,
            score: 0.0,
            projection_fraction,
        }
    }

    /// Two 5 m edges meeting at node 2.
    fn two_short_edges() -> (RoadNetwork, SpatialIndex, EdgeIndex, EdgeIndex) {
        let (network, spatial) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.000045, 0.0)
            .add_node(3, 0.00009, 0.0)
            .add_edge(100, 1, 2, 5.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(101, 2, 3, 5.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();
        let ids = network.edge_id_to_index.as_ref().unwrap();
        let (e1, e2) = (ids[&100], ids[&101]);
        (network, spatial, e1, e2)
    }

    /// HERE's shortest DNP bucket is 0–58 m, so a 4 m path for DNP 29 is valid.
    #[test]
    fn test_short_same_edge_path_accepted() {
        let (network, spatial, e1, _) = two_short_edges();
        let decoder = Decoder::new(&network, &spatial);

        let path = decoder
            .find_best_path(
                &[short_candidate(e1, 0.1)],
                &[short_candidate(e1, 0.9)],
                29.0,
                0,
                None,
            )
            .unwrap();
        assert_eq!(path.edges, vec![e1]);
        assert!((path.total_length - 4.0).abs() < 0.1);
    }

    #[test]
    fn test_short_multi_edge_path_accepted() {
        let (network, spatial, e1, e2) = two_short_edges();
        let decoder = Decoder::new(&network, &spatial);

        let path = decoder
            .find_best_path(
                &[short_candidate(e1, 0.5)],
                &[short_candidate(e2, 0.5)],
                29.0,
                0,
                None,
            )
            .unwrap();
        assert_eq!(path.edges, vec![e1, e2]);
        assert!((path.total_length - 5.0).abs() < 0.1);
    }

    /// Both LRPs clamped to node 2: a zero-length path, which would come back
    /// with no edges.
    #[test]
    fn test_zero_length_path_rejected() {
        let (network, spatial, e1, e2) = two_short_edges();
        let decoder = Decoder::new(&network, &spatial);

        let result = decoder.find_best_path(
            &[short_candidate(e1, 1.0)],
            &[short_candidate(e2, 0.0)],
            29.0,
            0,
            None,
        );
        assert!(result.is_err(), "{result:?}");
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
    /// N3 is 1-in/1-out, so invalid. N2 and N4 are junctions.
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

        // e1,e2 ends at the invalid N3. The last LRP is at the junction N4,
        // 15 m further on, where Rule 4 says the encoder would put it.
        let mut path = vec![edge_by_id(&network, 1), edge_by_id(&network, 2)];
        let n1 = network
            .node(network.edge_source(path[0]).unwrap())
            .unwrap()
            .coord;
        let n4 = network
            .node(network.edge_target(edge_by_id(&network, 3)).unwrap())
            .unwrap()
            .coord;
        let seg = SegmentInfo {
            length_m: 200.0,
            expected_m: 200.0,
            ..SegmentInfo::default()
        };

        let offsets = decoder.finalize_path(&mut path, n1, n4, seg, seg);

        assert_eq!(
            ids(&network, &path),
            vec![1, 2, 3],
            "e3 (15m) appended to reach N4"
        );
        assert!((offsets.negative_offset_m - 15.0).abs() < 1e-6);
        assert!((offsets.negative_offset_fraction - 1.0).abs() < 1e-6);
        // N1 is a dead end, so valid.
        assert!(offsets.positive_offset_m.abs() < 1e-6);
    }

    #[test]
    fn test_snap_respects_absolute_cap() {
        let (network, spatial) = snap_network();
        let config = DecoderConfig {
            max_snap_extension_m: 10.0, // e3 is 15 m
            ..DecoderConfig::default()
        };
        let decoder = Decoder::new(&network, &spatial).with_config(config);

        let mut path = vec![edge_by_id(&network, 1), edge_by_id(&network, 2)];
        let n1 = network
            .node(network.edge_source(path[0]).unwrap())
            .unwrap()
            .coord;
        let n4 = network
            .node(network.edge_target(edge_by_id(&network, 3)).unwrap())
            .unwrap()
            .coord;
        let seg = SegmentInfo {
            length_m: 200.0,
            expected_m: 200.0,
            ..SegmentInfo::default()
        };

        let offsets = decoder.finalize_path(&mut path, n1, n4, seg, seg);
        assert_eq!(ids(&network, &path), vec![1, 2]);
        assert!(offsets.negative_offset_m.abs() < 1e-6);
    }

    #[test]
    fn test_snap_respects_length_headroom() {
        let (network, spatial) = snap_network();
        let decoder = Decoder::new(&network, &spatial);
        let mut path = vec![edge_by_id(&network, 1), edge_by_id(&network, 2)];
        let n1 = network
            .node(network.edge_source(path[0]).unwrap())
            .unwrap()
            .coord;
        let n4 = network
            .node(network.edge_target(edge_by_id(&network, 3)).unwrap())
            .unwrap()
            .coord;

        // DNP 200 allows up to 300 m, leaving 5 m of headroom for the 15 m e3.
        let no_headroom = SegmentInfo {
            length_m: 295.0,
            expected_m: 200.0,
            ..SegmentInfo::default()
        };
        decoder.finalize_path(&mut path, n1, n4, no_headroom, no_headroom);
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
        let n4 = network
            .node(network.edge_target(edge_by_id(&network, 3)).unwrap())
            .unwrap()
            .coord;
        let seg = SegmentInfo {
            length_m: 200.0,
            expected_m: 200.0,
            ..SegmentInfo::default()
        };
        decoder.finalize_path(&mut path, n1, n4, seg, seg);
        assert_eq!(ids(&network, &path), vec![1, 2]);
    }

    #[test]
    fn test_snap_extends_start_to_valid_node() {
        let (network, spatial) = snap_network();
        let decoder = Decoder::new(&network, &spatial);

        // e3,e4 starts at the invalid N3. Its forced predecessor e2 (100 m) leads
        // back to the junction N2, where the first LRP sits. Needs a large cap.
        let mut path = vec![edge_by_id(&network, 3), edge_by_id(&network, 4)];
        let n2 = network
            .node(network.edge_source(edge_by_id(&network, 2)).unwrap())
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
            expected_m: 1500.0,
            ..SegmentInfo::default()
        };

        let offsets = decoder.finalize_path(&mut path, n2, n5, seg, seg);
        assert_eq!(
            ids(&network, &path),
            vec![2, 3, 4],
            "e2 prepended to reach junction N2"
        );
        assert!((offsets.positive_offset_m - 100.0).abs() < 1e-6);
        assert!((offsets.positive_offset_fraction - 1.0).abs() < 1e-6);
        // N5 is a dead end, so valid.
        assert!(offsets.negative_offset_m.abs() < 1e-6);
    }

    #[test]
    fn test_snap_skipped_when_lrp_sits_on_through_node() {
        // As in the end-snap test, but the last LRP is at N3 instead of N4. This
        // models HERE having a junction at N3 that our OSM car graph lacks, so
        // extending would overshoot.
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
        let seg = SegmentInfo {
            length_m: 200.0,
            expected_m: 200.0,
            ..SegmentInfo::default()
        };
        let offsets = decoder.finalize_path(&mut path, n1, n3, seg, seg);
        assert_eq!(ids(&network, &path), vec![1, 2], "LRP at N3: no extension");
        assert!(offsets.negative_offset_m.abs() < 1e-6);
    }

    #[test]
    fn test_snap_noop_when_terminal_nodes_already_valid() {
        let (network, spatial) = snap_network();
        let decoder = Decoder::new(&network, &spatial);
        // e1 runs from a dead end to a junction, both valid.
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
            ..SegmentInfo::default()
        };
        let offsets = decoder.finalize_path(&mut path, n1, n2, seg, seg);
        assert_eq!(ids(&network, &path), vec![1]);
        assert!(offsets.positive_offset_m.abs() < 1e-6);
        assert!(offsets.negative_offset_m.abs() < 1e-6);
    }
}
