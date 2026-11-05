use geo::{Closest, Distance, Haversine, HaversineClosestPoint, Point};
use petgraph::graph::EdgeIndex;

use crate::graph::{Fow, Frc, RoadNetwork};
use crate::spatial::{bearing_at_projection, bearing_difference, BearingDirection, SpatialIndex};

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

/// Up to `max_candidates` edges near an LRP, best score first. `direction` is
/// [`BearingDirection::Backward`] for the last LRP and `Forward` otherwise.
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
    // Pad the box because find_nearby's meters-to-degrees conversion is approximate.
    let nearby = spatial.find_nearby(coord, config.search_radius_m * 1.5);

    let mut candidates: Vec<Candidate> = nearby
        .into_iter()
        .filter_map(|env| {
            let edge = network.edge(env.edge_idx)?;

            let closest = match edge.geometry.haversine_closest_point(&coord) {
                Closest::SinglePoint(p) | Closest::Intersection(p) => p,
                Closest::Indeterminate => {
                    let first = edge.geometry.coords().next()?;
                    Point::new(first.x, first.y)
                }
            };
            let distance_m = Haversine.distance(coord, closest);

            if distance_m > config.search_radius_m {
                return None;
            }

            let edge_bearing = bearing_at_projection(coord, &edge.geometry, direction);
            let bearing_diff = bearing_difference(bearing, edge_bearing);

            if bearing_diff > config.max_bearing_diff {
                return None;
            }

            let frc_diff = (edge.frc as i8 - frc as i8).unsigned_abs();
            if frc_diff > config.frc_tolerance {
                return None;
            }

            // FOW only affects the score; it never filters.
            let fow_score = edge.fow.substitution_score(fow);

            let projection_fraction =
                crate::spatial::project_point_to_line_fraction(coord, &edge.geometry);

            let score = compute_score(distance_m, bearing_diff, frc_diff, fow_score, config);

            Some(Candidate {
                edge_idx: env.edge_idx,
                distance_m,
                bearing_diff,
                frc_diff,
                fow_score,
                score,
                projection_fraction,
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

    fn make_candidate(score: f64) -> Candidate {
        Candidate {
            edge_idx: EdgeIndex::new(0),
            distance_m: 0.0,
            bearing_diff: 0.0,
            frc_diff: 0,
            fow_score: 1.0,
            score,
            projection_fraction: 0.5,
        }
    }

    #[test]
    fn test_candidate_comparison_basic() {
        let better = make_candidate(0.5);
        let worse = make_candidate(1.0);

        assert!(better.score < worse.score);
    }

    #[test]
    fn test_candidate_comparison_equal_scores() {
        let a = make_candidate(0.5);
        let b = make_candidate(0.5);

        assert_eq!(a.score, b.score);
    }

    #[test]
    fn test_candidate_sorting() {
        // Port of the Java testComparison. Java ranks by a ratio where higher is
        // better, so its MAX_VALUE/MIN_VALUE cases become 0.0/f64::MAX here.
        let score_0 = make_candidate(0.0);
        let score_001 = make_candidate(0.001);
        let score_01 = make_candidate(0.01);
        let score_1 = make_candidate(1.0);
        let score_max = make_candidate(f64::MAX);

        let mut candidates = [
            score_01.clone(),
            score_max.clone(),
            score_0.clone(),
            score_001.clone(),
            score_1.clone(),
        ];

        candidates.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());

        let expected_scores = [0.0, 0.001, 0.01, 1.0, f64::MAX];

        for (i, expected) in expected_scores.iter().enumerate() {
            assert_eq!(
                candidates[i].score, *expected,
                "Unexpected score at index {}: got {}, expected {}",
                i, candidates[i].score, expected
            );
        }
    }

    #[test]
    fn test_candidate_sorting_edge_cases() {
        let score_zero = make_candidate(0.0);
        let score_tiny = make_candidate(f64::MIN_POSITIVE);
        let score_one = make_candidate(1.0);
        let score_large = make_candidate(1e10);
        let score_infinity = make_candidate(f64::INFINITY);

        let mut candidates = [
            score_infinity.clone(),
            score_zero.clone(),
            score_large.clone(),
            score_tiny.clone(),
            score_one.clone(),
        ];

        candidates.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());

        assert_eq!(candidates[0].score, 0.0);
        assert_eq!(candidates[1].score, f64::MIN_POSITIVE);
        assert_eq!(candidates[2].score, 1.0);
        assert_eq!(candidates[3].score, 1e10);
        assert_eq!(candidates[4].score, f64::INFINITY);
    }

    #[test]
    fn test_candidate_sorting_realistic_scores() {
        let config = CandidateConfig::default();

        let perfect = make_candidate(compute_score(0.0, 0.0, 0, 1.0, &config));
        let good = make_candidate(compute_score(5.0, 5.0, 0, 1.0, &config));
        let medium = make_candidate(compute_score(20.0, 10.0, 1, 0.75, &config));
        let poor = make_candidate(compute_score(50.0, 20.0, 2, 0.5, &config));
        let worst = make_candidate(compute_score(100.0, 30.0, 2, 0.0, &config));

        let mut candidates = [
            medium.clone(),
            worst.clone(),
            perfect.clone(),
            poor.clone(),
            good.clone(),
        ];

        candidates.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());

        assert_eq!(candidates[0].score, perfect.score);
        assert_eq!(candidates[1].score, good.score);
        assert_eq!(candidates[2].score, medium.score);
        assert_eq!(candidates[3].score, poor.score);
        assert_eq!(candidates[4].score, worst.score);

        for i in 0..candidates.len() - 1 {
            assert!(
                candidates[i].score < candidates[i + 1].score,
                "Score at index {} ({}) should be less than score at index {} ({})",
                i,
                candidates[i].score,
                i + 1,
                candidates[i + 1].score
            );
        }
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
