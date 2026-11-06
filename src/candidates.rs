use geo::Point;
use petgraph::graph::EdgeIndex;

use crate::graph::{Fow, Frc, RoadNetwork};
use crate::spatial::{
    bearing_along, bearing_difference, project_onto_line, BearingDirection, SpatialIndex,
};

/// An edge that an LRP may lie on.
#[derive(Debug, Clone)]
pub struct Candidate {
    pub edge_idx: EdgeIndex,
    pub distance_m: f64,
    pub bearing_diff: f64, // degrees, 0–180
    pub frc_diff: u8,
    pub fow_score: f64,           // see [`Fow::substitution_score`]
    pub score: f64,               // lower is better, see [`compute_score`]
    pub projection_fraction: f64, // where the LRP projects, 0–1 by length along the edge
}

#[derive(Debug, Clone)]
pub struct CandidateConfig {
    /// Hard limit on LRP-to-edge distance, and the scale for the distance penalty.
    pub search_radius_m: f64,
    /// Degrees. Hard filter, and the scale for the bearing penalty.
    pub max_bearing_diff: f64,
    /// Hard filter, and the scale for the FRC penalty.
    pub frc_tolerance: u8,
    pub max_candidates: usize,
    /// At least one LRP must have a candidate within this distance (meters) for the
    /// decode to proceed. Not a per-candidate filter; those go up to `search_radius_m`.
    pub max_candidate_distance_m: f64,
    // Penalty weights; see [`compute_score`].
    pub distance_weight: f64,
    pub bearing_weight: f64,
    pub frc_weight: f64,
    pub fow_weight: f64,
}

impl Default for CandidateConfig {
    fn default() -> Self {
        CandidateConfig {
            search_radius_m: 100.0,
            // Wide because HERE and OSM digitize curves and intersections differently.
            max_bearing_diff: 90.0,
            frc_tolerance: 2,
            max_candidates: 10,
            max_candidate_distance_m: 35.0,
            // HERE places LRPs on the road, so the nearest OSM edge is almost always
            // right even when its FRC/FOW disagree; HERE→OSM class mappings are too
            // loose to weigh more. Bearing mostly separates parallel roads.
            distance_weight: 10.0,
            bearing_weight: 0.2,
            frc_weight: 0.1,
            fow_weight: 0.1,
        }
    }
}

/// BEARDIST from the OpenLR spec (§5.2.4): an edge's bearing is measured over
/// a 20 m chord from the LRP's projection.
pub const BEARDIST_M: f64 = 20.0;

/// Up to `max_candidates` edges near an LRP, best score first. `direction` is
/// [`BearingDirection::Forward`] for every LRP but the last, whose attributes
/// describe the outgoing line. The last LRP uses `Backward`, the chord leading
/// up to it, but its bearing still points in the travel direction, as HERE
/// encodes it; the spec points it back along the line.
#[allow(clippy::too_many_arguments)]
pub fn find_candidates(
    coord: Point<f64>,
    bearing: f64,
    frc: Frc,
    fow: Fow,
    direction: BearingDirection,
    network: &RoadNetwork,
    spatial: &SpatialIndex,
    config: &CandidateConfig,
) -> Vec<Candidate> {
    let nearby = spatial.find_nearby(coord, config.search_radius_m);

    let mut candidates: Vec<Candidate> = nearby
        .into_iter()
        .filter_map(|env| {
            let edge = network.edge(env.edge_idx)?;

            // Slip roads are exempt, as in the decoder's LFRCNP filter: HERE gives a
            // ramp the class of the roads it joins (FRC0 between motorways), while
            // the loader puts every motorway_link at Frc3. The mismatch still costs
            // score.
            let frc_diff = (edge.frc as i8 - frc as i8).unsigned_abs();
            if frc_diff > config.frc_tolerance && edge.fow != Fow::SlipRoad {
                return None;
            }

            let proj = project_onto_line(coord, &edge.geometry)?;
            let distance_m = proj.distance_m;

            if distance_m > config.search_radius_m {
                return None;
            }

            let edge_bearing = bearing_along(&edge.geometry, proj.along_m, BEARDIST_M, direction);
            let bearing_diff = bearing_difference(bearing, edge_bearing);

            if bearing_diff > config.max_bearing_diff {
                return None;
            }

            // FOW only affects the score; it never filters.
            let fow_score = edge.fow.substitution_score(fow);

            let score = compute_score(distance_m, bearing_diff, frc_diff, fow_score, config);

            Some(Candidate {
                edge_idx: env.edge_idx,
                distance_m,
                bearing_diff,
                frc_diff,
                fow_score,
                score,
                projection_fraction: proj.fraction(),
            })
        })
        .collect();

    // Break score ties by edge index so the result doesn't depend on R-tree
    // traversal order, which changes whenever the tree is rebalanced.
    candidates.sort_by(|a, b| {
        a.score
            .total_cmp(&b.score)
            .then(a.edge_idx.cmp(&b.edge_idx))
    });
    candidates.truncate(config.max_candidates);

    candidates
}

/// Weighted sum of the four penalties, each scaled to 0–1 by its tolerance in
/// [`CandidateConfig`]. 0 is a perfect match; lower is better.
fn compute_score(
    distance_m: f64,
    bearing_diff: f64,
    frc_diff: u8,
    fow_score: f64,
    config: &CandidateConfig,
) -> f64 {
    // Each value is already filtered to `<= max`, so a zero max means a zero
    // value; score it as no penalty instead of 0/0 = NaN.
    let normalize = |value: f64, max: f64| if max > 0.0 { value / max } else { 0.0 };
    let distance_penalty = normalize(distance_m, config.search_radius_m);
    let bearing_penalty = normalize(bearing_diff, config.max_bearing_diff);
    let frc_penalty = normalize(frc_diff as f64, config.frc_tolerance as f64);
    let fow_penalty = 1.0 - fow_score;

    config.distance_weight * distance_penalty
        + config.bearing_weight * bearing_penalty
        + config.frc_weight * frc_penalty
        + config.fow_weight * fow_penalty
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_score_perfect_match() {
        let config = CandidateConfig::default();

        let score = compute_score(0.0, 0.0, 0, 1.0, &config);
        assert_eq!(score, 0.0);
    }

    #[test]
    fn test_compute_score_zero_tolerances_are_finite() {
        let config = CandidateConfig {
            frc_tolerance: 0,
            max_bearing_diff: 0.0,
            ..CandidateConfig::default()
        };
        let near = compute_score(1.0, 0.0, 0, 1.0, &config);
        let far = compute_score(10.0, 0.0, 0, 1.0, &config);
        assert!(near.is_finite() && far.is_finite());
        assert!(near < far);
    }

    #[test]
    fn test_compute_score_worse_match_higher_score() {
        let config = CandidateConfig::default();

        let perfect_score = compute_score(0.0, 0.0, 0, 1.0, &config);
        let worse_score = compute_score(50.0, 15.0, 1, 0.75, &config);
        assert!(worse_score > perfect_score);
    }

    #[test]
    fn test_compute_score_distance_variations() {
        let config = CandidateConfig::default();

        let score_0m = compute_score(0.0, 0.0, 0, 1.0, &config);
        let score_10m = compute_score(10.0, 0.0, 0, 1.0, &config);
        let score_50m = compute_score(50.0, 0.0, 0, 1.0, &config);
        let score_100m = compute_score(100.0, 0.0, 0, 1.0, &config);

        assert!(score_0m < score_10m);
        assert!(score_10m < score_50m);
        assert!(score_50m < score_100m);

        // At the search radius the distance penalty is the full weight.
        assert!((score_100m - 10.0).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_bearing_variations() {
        let config = CandidateConfig::default();

        let score_0deg = compute_score(0.0, 0.0, 0, 1.0, &config);
        let score_15deg = compute_score(0.0, 15.0, 0, 1.0, &config);
        let score_90deg = compute_score(0.0, 90.0, 0, 1.0, &config);

        assert!(score_0deg < score_15deg);
        assert!(score_15deg < score_90deg);

        assert!((score_90deg - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_frc_variations() {
        let config = CandidateConfig::default();

        let score_frc0 = compute_score(0.0, 0.0, 0, 1.0, &config);
        let score_frc1 = compute_score(0.0, 0.0, 1, 1.0, &config);
        let score_frc2 = compute_score(0.0, 0.0, 2, 1.0, &config);

        assert!(score_frc0 < score_frc1);
        assert!(score_frc1 < score_frc2);

        assert!((score_frc2 - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_fow_variations() {
        let config = CandidateConfig::default();

        let score_perfect_fow = compute_score(0.0, 0.0, 0, 1.0, &config);
        let score_good_fow = compute_score(0.0, 0.0, 0, 0.75, &config);
        let score_partial_fow = compute_score(0.0, 0.0, 0, 0.5, &config);
        let score_bad_fow = compute_score(0.0, 0.0, 0, 0.0, &config);

        assert!(score_perfect_fow < score_good_fow);
        assert!(score_good_fow < score_partial_fow);
        assert!(score_partial_fow < score_bad_fow);

        assert!((score_bad_fow - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_distance_dominates() {
        let config = CandidateConfig::default();

        // A close edge with poor bearing, FRC and FOW beats a far edge that matches
        // perfectly: about 1.3 vs 5.0.
        let close_bad = compute_score(10.0, 30.0, 3, 0.0, &config);
        let far_perfect = compute_score(50.0, 0.0, 0, 1.0, &config);

        assert!(close_bad < far_perfect);
    }

    #[test]
    fn test_compute_score_maximum_values() {
        let config = CandidateConfig::default();

        let max_score = compute_score(
            config.search_radius_m,
            config.max_bearing_diff,
            config.frc_tolerance,
            0.0,
            &config,
        );

        let expected =
            config.distance_weight + config.bearing_weight + config.frc_weight + config.fow_weight;
        assert!((max_score - expected).abs() < 0.001);
    }

    /// `find_candidates` returns best score first, and equal scores in edge
    /// index order.
    #[test]
    fn test_find_candidates_order() {
        use crate::test_utils::TestNetworkBuilder;

        // Edge 12 runs ~11 m north of the LRP and is added first, so it has the
        // lowest index but the worst score. Edges 11 and 10 coincide.
        let (network, spatial) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.002)
            .add_node(3, 0.0001, 0.0)
            .add_node(4, 0.0001, 0.002)
            .add_edge(12, 3, 4, 200.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(11, 1, 2, 200.0, Frc::Frc4, Fow::SingleCarriageway)
            .add_edge(10, 1, 2, 200.0, Frc::Frc4, Fow::SingleCarriageway)
            .build();
        let idx = |id| *network.edge_id_to_index.as_ref().unwrap().get(&id).unwrap();

        let candidates = find_candidates(
            Point::new(0.001, 0.00002),
            90.0,
            Frc::Frc4,
            Fow::SingleCarriageway,
            BearingDirection::Forward,
            &network,
            &spatial,
            &CandidateConfig::default(),
        );

        let order: Vec<EdgeIndex> = candidates.iter().map(|c| c.edge_idx).collect();
        assert_eq!(order, vec![idx(11), idx(10), idx(12)]);
        assert!(idx(11) < idx(10));
        assert_eq!(candidates[0].score, candidates[1].score);
        assert!(candidates[1].score < candidates[2].score);
    }

    #[test]
    fn test_slip_road_exempt_from_frc_filter() {
        use crate::test_utils::TestNetworkBuilder;

        // A HERE FRC0 ramp between motorways lands on an OSM motorway_link (Frc3).
        // An ordinary Frc3 road the same distance away is still filtered out.
        let (network, spatial) = TestNetworkBuilder::new()
            .add_node(1, 0.0, 0.0)
            .add_node(2, 0.0, 0.002)
            .add_edge(1, 1, 2, 200.0, Frc::Frc3, Fow::SlipRoad)
            .add_edge(2, 1, 2, 200.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();
        let idx = |id| *network.edge_id_to_index.as_ref().unwrap().get(&id).unwrap();

        let candidates = find_candidates(
            Point::new(0.001, 0.0),
            90.0,
            Frc::Frc0,
            Fow::Motorway,
            BearingDirection::Forward,
            &network,
            &spatial,
            &CandidateConfig::default(),
        );

        let found: Vec<EdgeIndex> = candidates.iter().map(|c| c.edge_idx).collect();
        assert_eq!(found, vec![idx(1)]);
    }

    #[test]
    fn test_score_monotonicity() {
        let config = CandidateConfig::default();

        let mut prev_score = 0.0;
        for distance in [0.0, 10.0, 20.0, 50.0, 100.0] {
            let score = compute_score(distance, 0.0, 0, 1.0, &config);
            assert!(
                score >= prev_score,
                "Score should increase with distance: {} at {}m vs {} before",
                score,
                distance,
                prev_score
            );
            prev_score = score;
        }

        prev_score = 0.0;
        for bearing in [0.0, 5.0, 10.0, 20.0, 30.0] {
            let score = compute_score(0.0, bearing, 0, 1.0, &config);
            assert!(
                score >= prev_score,
                "Score should increase with bearing diff: {} at {}° vs {} before",
                score,
                bearing,
                prev_score
            );
            prev_score = score;
        }

        prev_score = 0.0;
        for frc in [0, 1, 2] {
            let score = compute_score(0.0, 0.0, frc, 1.0, &config);
            assert!(
                score >= prev_score,
                "Score should increase with FRC diff: {} at frc_diff={} vs {} before",
                score,
                frc,
                prev_score
            );
            prev_score = score;
        }

        prev_score = 0.0;
        for fow_score in [1.0, 0.75, 0.5, 0.25, 0.0] {
            let score = compute_score(0.0, 0.0, 0, fow_score, &config);
            assert!(
                score >= prev_score,
                "Score should increase with worse FOW: {} at fow_score={} vs {} before",
                score,
                fow_score,
                prev_score
            );
            prev_score = score;
        }
    }

    #[test]
    fn test_custom_config_weights() {
        let custom_config = CandidateConfig {
            search_radius_m: 50.0,
            max_bearing_diff: 45.0,
            frc_tolerance: 3,
            max_candidates: 5,
            max_candidate_distance_m: 50.0,
            distance_weight: 1.0,
            bearing_weight: 1.0,
            frc_weight: 1.0,
            fow_weight: 1.0,
        };

        // With unit weights, each penalty at its limit contributes 1.0.
        let max_score = compute_score(50.0, 45.0, 3, 0.0, &custom_config);
        assert!((max_score - 4.0).abs() < 0.001);

        // Half of each limit, except FRC, which is 1 of 3.
        let half_score = compute_score(25.0, 22.5, 1, 0.5, &custom_config);
        let expected_half = 0.5 + 0.5 + (1.0 / 3.0) + 0.5;
        assert!((half_score - expected_half).abs() < 0.001);
    }
}
