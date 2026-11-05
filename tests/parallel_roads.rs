//! Port of the Java reference decoder's ParallelRoadsTest
//! (openlr/decoder/src/test/java/openlr/decoder/ParallelRoadsTest.java).
//!
//! ```text
//!        ____
//! N1---N2____N3---N4
//! ```
//!
//! Every line is drivable both ways. Lines 2 (750 m) and 4 (800 m) both join N2
//! and N3, so only the DNP can tell them apart; each test expects line 2.

use geo::{Bearing, Geodesic, LineString, Point};
use openlr_decoder::spatial::BearingDirection;
use openlr_decoder::{
    find_candidates, Candidate, DecoderConfig, Edge, Fow, Frc, RoadNetwork, SpatialIndex,
};
use petgraph::visit::EdgeRef;

/// Node coordinates from ParallelRoadsMap.xml, in (lon, lat) order.
const NODE_1: (f64, f64) = (-73.96526, 40.75629);
const NODE_2: (f64, f64) = (-73.96696, 40.75702);
const NODE_3: (f64, f64) = (-73.96922, 40.75797);
const NODE_4: (f64, f64) = (-73.9708, 40.75862);

/// Edge lengths from ParallelRoadsMap.xml, in meters.
const LINE_1_LENGTH: f64 = 500.0; // N1 -> N2
const LINE_2_LENGTH: f64 = 750.0; // N2 -> N3, shorter parallel
const LINE_3_LENGTH: f64 = 550.0; // N3 -> N4
const LINE_4_LENGTH: f64 = 800.0; // N2 -> N3, longer parallel

/// Geodesic bearing in [0, 360), from (lon, lat) points.
fn compute_bearing(from: (f64, f64), to: (f64, f64)) -> f64 {
    let p1 = Point::new(from.0, from.1);
    let p2 = Point::new(to.0, to.1);
    let bearing = Geodesic.bearing(p1, p2);
    ((bearing % 360.0) + 360.0) % 360.0
}

fn build_parallel_roads_network() -> (RoadNetwork, SpatialIndex) {
    let mut network = RoadNetwork::new();

    network.get_or_add_node(1, Point::new(NODE_1.0, NODE_1.1));
    network.get_or_add_node(2, Point::new(NODE_2.0, NODE_2.1));
    network.get_or_add_node(3, Point::new(NODE_3.0, NODE_3.1));
    network.get_or_add_node(4, Point::new(NODE_4.0, NODE_4.1));

    let geom_1_2 = LineString::from(vec![NODE_1, NODE_2]);
    let geom_2_1 = LineString::from(vec![NODE_2, NODE_1]);
    let geom_3_2 = LineString::from(vec![NODE_3, NODE_2]);
    let geom_3_4 = LineString::from(vec![NODE_3, NODE_4]);
    let geom_4_3 = LineString::from(vec![NODE_4, NODE_3]);

    let geom_2_short = LineString::from(vec![NODE_2, NODE_3]);

    // Line 4 detours through an offset midpoint so the two parallels have
    // distinct geometry. Its length still comes from LINE_4_LENGTH.
    let mid_point_4: (f64, f64) = (
        (NODE_2.0 + NODE_3.0) / 2.0 - 0.0005,
        (NODE_2.1 + NODE_3.1) / 2.0 + 0.0003,
    );
    let geom_4_long = LineString::from(vec![NODE_2, mid_point_4, NODE_3]);
    let geom_4_long_rev = LineString::from(vec![NODE_3, mid_point_4, NODE_2]);

    // Reverse edges get id + 100.
    let edge_1 = Edge::new_with_length(
        1,
        geom_1_2.clone(),
        Frc::Frc5,
        Fow::MultipleCarriageway,
        LINE_1_LENGTH,
    );
    let edge_neg1 = Edge::new_with_length(
        101,
        geom_2_1.clone(),
        Frc::Frc5,
        Fow::MultipleCarriageway,
        LINE_1_LENGTH,
    );

    let edge_2 = Edge::new_with_length(
        2,
        geom_2_short.clone(),
        Frc::Frc5,
        Fow::MultipleCarriageway,
        LINE_2_LENGTH,
    );
    let edge_neg2 = Edge::new_with_length(
        102,
        geom_3_2.clone(),
        Frc::Frc5,
        Fow::MultipleCarriageway,
        LINE_2_LENGTH,
    );

    let edge_3 = Edge::new_with_length(
        3,
        geom_3_4.clone(),
        Frc::Frc5,
        Fow::MultipleCarriageway,
        LINE_3_LENGTH,
    );
    let edge_neg3 = Edge::new_with_length(
        103,
        geom_4_3.clone(),
        Frc::Frc5,
        Fow::MultipleCarriageway,
        LINE_3_LENGTH,
    );

    let edge_4 = Edge::new_with_length(
        4,
        geom_4_long.clone(),
        Frc::Frc5,
        Fow::MultipleCarriageway,
        LINE_4_LENGTH,
    );
    let edge_neg4 = Edge::new_with_length(
        104,
        geom_4_long_rev.clone(),
        Frc::Frc5,
        Fow::MultipleCarriageway,
        LINE_4_LENGTH,
    );

    network.add_edge(1, 2, edge_1);
    network.add_edge(2, 1, edge_neg1);
    network.add_edge(2, 3, edge_2);
    network.add_edge(3, 2, edge_neg2);
    network.add_edge(3, 4, edge_3);
    network.add_edge(4, 3, edge_neg3);
    network.add_edge(2, 3, edge_4);
    network.add_edge(3, 2, edge_neg4);

    let spatial = build_spatial_index(&network);

    (network, spatial)
}

fn build_spatial_index(network: &RoadNetwork) -> SpatialIndex {
    use openlr_decoder::spatial::EdgeEnvelope;

    let envelopes: Vec<EdgeEnvelope> = network
        .graph
        .edge_indices()
        .filter_map(|idx| {
            let edge = network.edge(idx)?;
            Some(EdgeEnvelope::new(idx, &edge.geometry))
        })
        .collect();

    SpatialIndex::new(envelopes)
}

/// Stand-alone decoder over raw LRP tuples. It reuses `find_candidates` but has
/// its own path search, which enumerates every parallel edge between nodes.
struct TestDecoder<'a> {
    network: &'a RoadNetwork,
    spatial: &'a SpatialIndex,
    config: DecoderConfig,
}

impl<'a> TestDecoder<'a> {
    fn new(network: &'a RoadNetwork, spatial: &'a SpatialIndex) -> Self {
        TestDecoder {
            network,
            spatial,
            config: DecoderConfig::default(),
        }
    }

    /// LRPs are (lat, lon, bearing, dnp); the last LRP's dnp is ignored.
    /// Returns the edge ids of the decoded path.
    fn decode_lrps(&self, lrps: &[(f64, f64, f64, f64)]) -> Result<Vec<u64>, String> {
        if lrps.len() < 2 {
            return Err("Need at least 2 LRPs".to_string());
        }

        let mut all_candidates: Vec<Vec<Candidate>> = Vec::new();

        for (i, &(lat, lon, bearing, _dnp)) in lrps.iter().enumerate() {
            let coord = Point::new(lon, lat);
            let candidates = find_candidates(
                coord,
                bearing,
                Frc::Frc5, // matches every edge in this network
                Fow::MultipleCarriageway,
                if i == lrps.len() - 1 {
                    BearingDirection::Backward
                } else {
                    BearingDirection::Forward
                },
                self.network,
                self.spatial,
                &self.config.candidate_config,
            );

            if candidates.is_empty() {
                return Err(format!("No candidates found for LRP {}", i));
            }

            all_candidates.push(candidates);
        }

        let mut full_path: Vec<u64> = Vec::new();

        for i in 0..lrps.len() - 1 {
            let expected_distance = lrps[i].3;
            if expected_distance < 0.0 {
                continue;
            }

            let segment = self.find_best_path_segment(
                &all_candidates[i],
                &all_candidates[i + 1],
                expected_distance,
            )?;

            // Consecutive segments share the edge at the LRP between them.
            for &edge_id in &segment {
                if full_path.last() != Some(&edge_id) {
                    full_path.push(edge_id);
                }
            }
        }

        Ok(full_path)
    }

    fn find_best_path_segment(
        &self,
        start_candidates: &[Candidate],
        end_candidates: &[Candidate],
        expected_distance: f64,
    ) -> Result<Vec<u64>, String> {
        let tolerance = self.config.length_tolerance;
        let abs_tolerance = self.config.absolute_length_tolerance;
        let min_distance = (expected_distance * (1.0 - tolerance))
            .min(expected_distance - abs_tolerance)
            .max(10.0);
        let max_distance =
            (expected_distance * (1.0 + tolerance)).max(expected_distance + abs_tolerance);

        let mut best_path: Option<Vec<u64>> = None;
        let mut best_score = f64::MAX;

        // Try all combinations of start and end candidates
        for start_cand in start_candidates.iter().take(10) {
            for end_cand in end_candidates.iter().take(10) {
                if let Some((path, path_length)) = self.find_path_with_target(
                    start_cand,
                    end_cand,
                    expected_distance * 2.0,
                    Some(expected_distance),
                ) {
                    if path_length >= min_distance && path_length <= max_distance {
                        let length_diff =
                            (path_length - expected_distance).abs() / expected_distance.max(1.0);
                        let score = start_cand.score + end_cand.score + length_diff;

                        if score < best_score {
                            best_score = score;
                            best_path = Some(path);
                        }
                    }
                }
            }
        }

        best_path.ok_or_else(|| "No valid path found".to_string())
    }

    /// With `target_length`, picks the edge path whose length is closest to it.
    fn find_path_with_target(
        &self,
        start_cand: &Candidate,
        end_cand: &Candidate,
        max_distance: f64,
        target_length: Option<f64>,
    ) -> Option<(Vec<u64>, f64)> {
        use petgraph::algo::astar;

        if start_cand.edge_idx == end_cand.edge_idx {
            let edge = self.network.edge(start_cand.edge_idx)?;
            let frac_diff = end_cand.projection_fraction - start_cand.projection_fraction;
            if frac_diff > 0.0 {
                let path_length = edge.length_m * frac_diff;
                return Some((vec![edge.id], path_length));
            }
            return None;
        }

        let start_node = self.network.edge_target(start_cand.edge_idx)?;
        let end_node = self.network.edge_source(end_cand.edge_idx)?;

        let start_edge = self.network.edge(start_cand.edge_idx)?;
        let end_edge = self.network.edge(end_cand.edge_idx)?;

        let start_partial = start_edge.length_m * (1.0 - start_cand.projection_fraction);
        let end_partial = end_edge.length_m * end_cand.projection_fraction;

        if start_node == end_node {
            let path_length = start_partial + end_partial;
            if path_length <= max_distance {
                return Some((vec![start_edge.id, end_edge.id], path_length));
            }
            return None;
        }

        let result = astar(
            &self.network.graph,
            start_node,
            |n| n == end_node,
            |e| e.weight().length_m,
            |_| 0.0, // no heuristic, so this is Dijkstra
        )?;

        let (_middle_cost, path_nodes) = result;

        // A* returns a node path; parallel edges between the same nodes can
        // differ in length, so score every edge combination along it.
        let edge_options = self.enumerate_edge_paths(&path_nodes);

        let mut best_path: Option<(Vec<u64>, f64)> = None;
        let mut best_diff = f64::MAX;

        for middle_edges in edge_options {
            let middle_cost: f64 = middle_edges
                .iter()
                .filter_map(|&eid| self.network.edge_id_to_index.as_ref().unwrap().get(&eid))
                .filter_map(|&idx| self.network.edge(idx))
                .map(|e| e.length_m)
                .sum();

            let path_cost = start_partial + middle_cost + end_partial;
            if path_cost > max_distance {
                continue;
            }

            let mut edge_ids = vec![start_edge.id];
            edge_ids.extend(middle_edges);
            edge_ids.push(end_edge.id);
            edge_ids.dedup();

            let diff = if let Some(target) = target_length {
                (path_cost - target).abs()
            } else {
                0.0
            };

            if diff < best_diff || best_path.is_none() {
                best_diff = diff;
                best_path = Some((edge_ids, path_cost));
            }
        }

        best_path
    }

    /// Every edge sequence along `node_path`, one per choice of parallel edge.
    fn enumerate_edge_paths(&self, nodes: &[petgraph::graph::NodeIndex]) -> Vec<Vec<u64>> {
        if nodes.len() < 2 {
            return vec![vec![]];
        }

        let first_edges: Vec<u64> = self
            .network
            .graph
            .edges_connecting(nodes[0], nodes[1])
            .filter_map(|e| self.network.edge(e.id()).map(|edge| edge.id))
            .collect();

        if nodes.len() == 2 {
            return first_edges.into_iter().map(|e| vec![e]).collect();
        }

        let rest_paths = self.enumerate_edge_paths(&nodes[1..]);

        let mut result = Vec::new();
        for first_edge in first_edges {
            for rest in &rest_paths {
                let mut path = vec![first_edge];
                path.extend(rest.iter().cloned());
                result.push(path);
            }
        }

        result
    }
}

/// N1 -> N4 with the parallel pair in the middle. Expects edges [1, 2, 3].
#[test]
fn test_parallel_roads() {
    let (network, spatial) = build_parallel_roads_network();
    let decoder = TestDecoder::new(&network, &spatial);

    let bearing_n1_n2 = compute_bearing(NODE_1, NODE_2);
    let bearing_n3_n4 = compute_bearing(NODE_3, NODE_4);

    let lrps = vec![
        (NODE_1.1, NODE_1.0, bearing_n1_n2, 1800.0),
        (NODE_4.1, NODE_4.0, bearing_n3_n4, -1.0),
    ];

    let result = decoder.decode_lrps(&lrps);
    assert!(result.is_ok(), "Decoding failed: {:?}", result.err());

    let edge_ids = result.unwrap();
    println!("test_parallel_roads: decoded edges = {:?}", edge_ids);

    assert!(
        edge_ids.contains(&1),
        "Path should contain edge 1 (N1->N2). Got: {:?}",
        edge_ids
    );
    assert!(
        edge_ids.contains(&3),
        "Path should contain edge 3 (N3->N4). Got: {:?}",
        edge_ids
    );

    let has_shorter = edge_ids.contains(&2);
    let has_longer = edge_ids.contains(&4);
    assert!(
        has_shorter || has_longer,
        "Path should contain either edge 2 or 4 (parallel section). Got: {:?}",
        edge_ids
    );

    // Via line 2: 500 + 750 + 550 = 1800 m, the DNP. Via line 4: 1850 m.
    assert!(
        has_shorter && !has_longer,
        "Should choose edge 2 (shorter, 750m) over edge 4 (longer, 800m) for 1800m path. Got: {:?}",
        edge_ids
    );
    println!("test_parallel_roads: Correctly chose shorter parallel road (edge 2)");
}

/// N2 -> N3, covering only the parallel pair. Expects edge [2].
#[test]
fn test_parallel_roads_only() {
    let (network, spatial) = build_parallel_roads_network();
    let decoder = TestDecoder::new(&network, &spatial);

    let bearing_n2_n3 = compute_bearing(NODE_2, NODE_3);

    let lrps = vec![
        (NODE_2.1, NODE_2.0, bearing_n2_n3, 750.0),
        (NODE_3.1, NODE_3.0, bearing_n2_n3, -1.0),
    ];

    let result = decoder.decode_lrps(&lrps);
    assert!(result.is_ok(), "Decoding failed: {:?}", result.err());

    let edge_ids = result.unwrap();
    println!("test_parallel_roads_only: decoded edges = {:?}", edge_ids);

    assert!(
        !edge_ids.is_empty(),
        "Path should not be empty. Got: {:?}",
        edge_ids
    );

    let has_shorter = edge_ids.contains(&2);
    let has_longer = edge_ids.contains(&4);
    assert!(
        has_shorter || has_longer,
        "Path should contain edge 2 or 4 (parallel roads). Got: {:?}",
        edge_ids
    );

    assert!(
        has_shorter && !has_longer,
        "Should choose edge 2 (750m, exact match) over edge 4 (800m) for 750m DNP. Got: {:?}",
        edge_ids
    );
    println!("test_parallel_roads_only: Correctly chose shorter parallel road (edge 2, 750m)");
}

/// N1 -> N3, ending on the parallel pair. Expects edges [1, 2].
#[test]
fn test_parallel_roads_at_end() {
    let (network, spatial) = build_parallel_roads_network();
    let decoder = TestDecoder::new(&network, &spatial);

    let bearing_n1_n2 = compute_bearing(NODE_1, NODE_2);
    let bearing_n2_n3 = compute_bearing(NODE_2, NODE_3);

    let lrps = vec![
        (NODE_1.1, NODE_1.0, bearing_n1_n2, 1250.0),
        (NODE_3.1, NODE_3.0, bearing_n2_n3, -1.0),
    ];

    let result = decoder.decode_lrps(&lrps);
    assert!(result.is_ok(), "Decoding failed: {:?}", result.err());

    let edge_ids = result.unwrap();
    println!("test_parallel_roads_at_end: decoded edges = {:?}", edge_ids);

    assert!(
        edge_ids.contains(&1),
        "Path should contain edge 1 (N1->N2). Got: {:?}",
        edge_ids
    );

    assert!(
        !edge_ids.contains(&3),
        "Path should NOT contain edge 3 (N3->N4). Got: {:?}",
        edge_ids
    );

    let has_shorter = edge_ids.contains(&2);
    let has_longer = edge_ids.contains(&4);
    assert!(
        has_shorter || has_longer,
        "Path should contain edge 2 or 4 (parallel section). Got: {:?}",
        edge_ids
    );

    // Via line 2: 500 + 750 = 1250 m, the DNP. Via line 4: 1300 m.
    assert!(
        has_shorter && !has_longer,
        "Should choose edge 2 (shorter, 750m) over edge 4 (longer, 800m) for 1250m path. Got: {:?}",
        edge_ids
    );
    println!("test_parallel_roads_at_end: Correctly chose shorter parallel road (edge 2)");
}

/// `Edge::new` with the geometry length replaced by `length_m`.
trait EdgeExt {
    fn new_with_length(
        id: u64,
        geometry: LineString<f64>,
        frc: Frc,
        fow: Fow,
        length_m: f64,
    ) -> Self;
}

impl EdgeExt for Edge {
    fn new_with_length(
        id: u64,
        geometry: LineString<f64>,
        frc: Frc,
        fow: Fow,
        length_m: f64,
    ) -> Self {
        let mut edge = Edge::new(id, geometry, frc, fow);
        edge.length_m = length_m;
        edge
    }
}
