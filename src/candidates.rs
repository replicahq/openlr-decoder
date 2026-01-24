use geo::Point;
use petgraph::graph::EdgeIndex;

use crate::graph::{Fow, Frc, RoadNetwork};
use crate::spatial::{bearing_at_projection, bearing_difference, SpatialIndex};

/// A candidate edge for matching an LRP
#[derive(Debug, Clone)]
pub struct Candidate {
    pub edge_idx: EdgeIndex,
    pub distance_m: f64,          // Distance from LRP to edge
    pub bearing_diff: f64,        // Bearing difference in degrees
    pub frc_diff: u8,             // FRC difference (0 = exact match)
    pub fow_compatible: bool,     // FOW compatibility
    pub score: f64,               // Combined score (lower is better)
    pub projection_fraction: f64, // Where on the edge the LRP projects (0-1)
}

/// Configuration for candidate scoring
#[derive(Debug, Clone)]
pub struct CandidateConfig {
    pub search_radius_m: f64,
    pub max_bearing_diff: f64,
    pub frc_tolerance: u8,
    pub max_candidates: usize,
    // Scoring weights
    pub distance_weight: f64,
    pub bearing_weight: f64,
    pub frc_weight: f64,
}

impl Default for CandidateConfig {
    fn default() -> Self {
        CandidateConfig {
            search_radius_m: 100.0, // 100m search radius
            max_bearing_diff: 30.0, // ±30 degrees bearing tolerance
            frc_tolerance: 2,       // Allow ±2 FRC classes
            max_candidates: 10,     // Keep top 10 candidates
            // Scoring weights for cross-provider decoding (HERE → OSM):
            // Distance strongly dominates - LRP should be on top of the correct road
            // Bearing helps distinguish parallel roads going different directions
            // FRC is just a tiebreaker since mappings between providers differ
            distance_weight: 4.0, // Primary - closest road strongly wins
            bearing_weight: 0.2,  // Minor tiebreaker for parallel roads
            frc_weight: 0.1,      // Minimal - FRC mappings are very approximate
        }
    }
}

/// Find and score candidate edges for an LRP
pub fn find_candidates(
    coord: Point<f64>,
    bearing: f64,
    frc: Frc,
    fow: Fow,
    network: &RoadNetwork,
    spatial: &SpatialIndex,
    config: &CandidateConfig,
) -> Vec<Candidate> {
    // Find nearby edges
    let nearby = spatial.find_within_radius(coord, config.search_radius_m);

    let mut candidates: Vec<Candidate> = nearby
        .into_iter()
        .filter_map(|(env, distance_m)| {
            let edge = network.edge(env.edge_idx)?;

            // Compute bearing at the projection point (where LRP projects onto edge)
            let edge_bearing = bearing_at_projection(coord, &env.geometry);
            let bearing_diff = bearing_difference(bearing, edge_bearing);

            // Check bearing tolerance
            if bearing_diff > config.max_bearing_diff {
                return None;
            }

            // Check FRC compatibility
            let frc_diff = (edge.frc as i8 - frc as i8).unsigned_abs();
            if frc_diff > config.frc_tolerance {
                return None;
            }

            // Check FOW compatibility
            let fow_compatible = edge.fow.is_compatible(fow);
            if !fow_compatible {
                return None;
            }

            // Compute projection fraction
            let projection_fraction =
                crate::spatial::project_point_to_line_fraction(coord, &env.geometry);

            // Compute score (lower is better)
            let score = compute_score(distance_m, bearing_diff, frc_diff, config);

            Some(Candidate {
                edge_idx: env.edge_idx,
                distance_m,
                bearing_diff,
                frc_diff,
                fow_compatible,
                score,
                projection_fraction,
            })
        })
        .collect();

    // Sort by score and take top N
    candidates.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());
    candidates.truncate(config.max_candidates);

    candidates
}

/// Compute combined score for a candidate (lower is better)
fn compute_score(
    distance_m: f64,
    bearing_diff: f64,
    frc_diff: u8,
    config: &CandidateConfig,
) -> f64 {
    // Normalize each component to 0-1 range
    let distance_score = distance_m / config.search_radius_m;
    let bearing_score = bearing_diff / config.max_bearing_diff;
    let frc_score = frc_diff as f64 / config.frc_tolerance as f64;

    // Weighted sum
    config.distance_weight * distance_score
        + config.bearing_weight * bearing_score
        + config.frc_weight * frc_score
}

/// Find candidates for the start of a line (uses start bearing)
pub fn find_start_candidates(
    coord: Point<f64>,
    bearing: f64,
    frc: Frc,
    fow: Fow,
    network: &RoadNetwork,
    spatial: &SpatialIndex,
    config: &CandidateConfig,
) -> Vec<Candidate> {
    find_candidates(coord, bearing, frc, fow, network, spatial, config)
}

/// Find candidates for an intermediate or end LRP
/// These need to match the incoming bearing (edge end bearing)
pub fn find_end_candidates(
    coord: Point<f64>,
    bearing: f64,
    frc: Frc,
    fow: Fow,
    network: &RoadNetwork,
    spatial: &SpatialIndex,
    config: &CandidateConfig,
) -> Vec<Candidate> {
    // For end points, we need to check the bearing at the end of edges
    // leading into this point, which is slightly different logic
    // For now, use the same function but this could be enhanced
    find_candidates(coord, bearing, frc, fow, network, spatial, config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_score_perfect_match() {
        let config = CandidateConfig::default();

        // Perfect match should have score 0
        let score = compute_score(0.0, 0.0, 0, &config);
        assert_eq!(score, 0.0);
    }

    #[test]
    fn test_compute_score_worse_match_higher_score() {
        let config = CandidateConfig::default();

        let perfect_score = compute_score(0.0, 0.0, 0, &config);
        let worse_score = compute_score(50.0, 15.0, 1, &config);
        assert!(worse_score > perfect_score);
    }

    #[test]
    fn test_compute_score_distance_variations() {
        let config = CandidateConfig::default();

        // Test various distance values with zero bearing/frc diff
        let score_0m = compute_score(0.0, 0.0, 0, &config);
        let score_10m = compute_score(10.0, 0.0, 0, &config);
        let score_50m = compute_score(50.0, 0.0, 0, &config);
        let score_100m = compute_score(100.0, 0.0, 0, &config);

        // Scores should increase with distance
        assert!(score_0m < score_10m);
        assert!(score_10m < score_50m);
        assert!(score_50m < score_100m);

        // At max search radius (100m), distance component should contribute
        // distance_weight * (100 / 100) = 4.0 * 1.0 = 4.0
        assert!((score_100m - 4.0).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_bearing_variations() {
        let config = CandidateConfig::default();

        // Test various bearing differences with zero distance/frc diff
        let score_0deg = compute_score(0.0, 0.0, 0, &config);
        let score_15deg = compute_score(0.0, 15.0, 0, &config);
        let score_30deg = compute_score(0.0, 30.0, 0, &config);

        // Scores should increase with bearing difference
        assert!(score_0deg < score_15deg);
        assert!(score_15deg < score_30deg);

        // At max bearing diff (30°), bearing component should contribute
        // bearing_weight * (30 / 30) = 0.2 * 1.0 = 0.2
        assert!((score_30deg - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_frc_variations() {
        let config = CandidateConfig::default();

        // Test various FRC differences with zero distance/bearing diff
        let score_frc0 = compute_score(0.0, 0.0, 0, &config);
        let score_frc1 = compute_score(0.0, 0.0, 1, &config);
        let score_frc2 = compute_score(0.0, 0.0, 2, &config);

        // Scores should increase with FRC difference
        assert!(score_frc0 < score_frc1);
        assert!(score_frc1 < score_frc2);

        // At max FRC diff (2), FRC component should contribute
        // frc_weight * (2 / 2) = 0.1 * 1.0 = 0.1
        assert!((score_frc2 - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_distance_dominates() {
        let config = CandidateConfig::default();

        // Distance should dominate scoring (weight 4.0 vs 0.2 and 0.1)
        // A closer edge with worse bearing/frc should beat a farther edge with perfect bearing/frc
        let close_bad_bearing = compute_score(10.0, 30.0, 2, &config);
        let far_perfect = compute_score(50.0, 0.0, 0, &config);

        // close_bad_bearing = 4.0 * 0.1 + 0.2 * 1.0 + 0.1 * 1.0 = 0.4 + 0.2 + 0.1 = 0.7
        // far_perfect = 4.0 * 0.5 + 0 + 0 = 2.0
        assert!(close_bad_bearing < far_perfect);
    }

    #[test]
    fn test_compute_score_maximum_values() {
        let config = CandidateConfig::default();

        // Maximum everything should give maximum score
        let max_score = compute_score(
            config.search_radius_m,
            config.max_bearing_diff,
            config.frc_tolerance,
            &config,
        );

        // max_score = 4.0 * 1.0 + 0.2 * 1.0 + 0.1 * 1.0 = 4.3
        let expected = config.distance_weight + config.bearing_weight + config.frc_weight;
        assert!((max_score - expected).abs() < 0.001);
    }

    /// Helper to create a test candidate with a specific score
    fn make_candidate(score: f64) -> Candidate {
        Candidate {
            edge_idx: EdgeIndex::new(0),
            distance_m: 0.0,
            bearing_diff: 0.0,
            frc_diff: 0,
            fow_compatible: true,
            score,
            projection_fraction: 0.5,
        }
    }

    #[test]
    fn test_candidate_comparison_basic() {
        // Lower score is better, so lower scores should come first when sorted
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
        // Port of Java testComparison - verify candidates sort correctly by score
        // In Rust, lower score is better (opposite of Java's higher ratio is better)

        let score_0 = make_candidate(0.0); // Best - like Java's MAX_VALUE ratio
        let score_001 = make_candidate(0.001);
        let score_01 = make_candidate(0.01);
        let score_1 = make_candidate(1.0);
        let score_max = make_candidate(f64::MAX); // Worst - like Java's MIN_VALUE ratio

        // Create unsorted list (similar to Java test setup)
        let mut candidates = vec![
            score_01.clone(),
            score_max.clone(),
            score_0.clone(),
            score_001.clone(),
            score_1.clone(),
        ];

        // Sort by score ascending (lower is better)
        candidates.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());

        // Expected order: best to worst (0.0, 0.001, 0.01, 1.0, MAX)
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
        // Test edge cases similar to Java test with MAX_VALUE, MIN_VALUE, zero
        let score_zero = make_candidate(0.0);
        let score_tiny = make_candidate(f64::MIN_POSITIVE);
        let score_one = make_candidate(1.0);
        let score_large = make_candidate(1e10);
        let score_infinity = make_candidate(f64::INFINITY);

        let mut candidates = vec![
            score_infinity.clone(),
            score_zero.clone(),
            score_large.clone(),
            score_tiny.clone(),
            score_one.clone(),
        ];

        candidates.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());

        // Expected order: 0.0, MIN_POSITIVE, 1.0, 1e10, INFINITY
        assert_eq!(candidates[0].score, 0.0);
        assert_eq!(candidates[1].score, f64::MIN_POSITIVE);
        assert_eq!(candidates[2].score, 1.0);
        assert_eq!(candidates[3].score, 1e10);
        assert_eq!(candidates[4].score, f64::INFINITY);
    }

    #[test]
    fn test_candidate_sorting_realistic_scores() {
        let config = CandidateConfig::default();

        // Create candidates with realistic computed scores
        let perfect = make_candidate(compute_score(0.0, 0.0, 0, &config));
        let good = make_candidate(compute_score(5.0, 5.0, 0, &config));
        let medium = make_candidate(compute_score(20.0, 10.0, 1, &config));
        let poor = make_candidate(compute_score(50.0, 20.0, 2, &config));
        let worst = make_candidate(compute_score(100.0, 30.0, 2, &config));

        let mut candidates = vec![
            medium.clone(),
            worst.clone(),
            perfect.clone(),
            poor.clone(),
            good.clone(),
        ];

        candidates.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());

        // Verify sorted order
        assert_eq!(candidates[0].score, perfect.score);
        assert_eq!(candidates[1].score, good.score);
        assert_eq!(candidates[2].score, medium.score);
        assert_eq!(candidates[3].score, poor.score);
        assert_eq!(candidates[4].score, worst.score);

        // Also verify the scores are in strictly increasing order
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
        // Verify score increases monotonically with each parameter
        let config = CandidateConfig::default();

        // Distance monotonicity
        let mut prev_score = 0.0;
        for distance in [0.0, 10.0, 20.0, 50.0, 100.0] {
            let score = compute_score(distance, 0.0, 0, &config);
            assert!(
                score >= prev_score,
                "Score should increase with distance: {} at {}m vs {} before",
                score,
                distance,
                prev_score
            );
            prev_score = score;
        }

        // Bearing monotonicity
        prev_score = 0.0;
        for bearing in [0.0, 5.0, 10.0, 20.0, 30.0] {
            let score = compute_score(0.0, bearing, 0, &config);
            assert!(
                score >= prev_score,
                "Score should increase with bearing diff: {} at {}° vs {} before",
                score,
                bearing,
                prev_score
            );
            prev_score = score;
        }

        // FRC monotonicity
        prev_score = 0.0;
        for frc in [0, 1, 2] {
            let score = compute_score(0.0, 0.0, frc, &config);
            assert!(
                score >= prev_score,
                "Score should increase with FRC diff: {} at frc_diff={} vs {} before",
                score,
                frc,
                prev_score
            );
            prev_score = score;
        }
    }

    #[test]
    fn test_custom_config_weights() {
        // Test that custom weights are respected
        let custom_config = CandidateConfig {
            search_radius_m: 50.0,
            max_bearing_diff: 45.0,
            frc_tolerance: 3,
            max_candidates: 5,
            distance_weight: 1.0,
            bearing_weight: 1.0,
            frc_weight: 1.0,
        };

        // With equal weights, all components contribute equally at their max values
        let max_score = compute_score(50.0, 45.0, 3, &custom_config);
        // Each component: 1.0 * 1.0 = 1.0, total = 3.0
        assert!((max_score - 3.0).abs() < 0.001);

        // Half values should give half score
        let half_score = compute_score(25.0, 22.5, 1, &custom_config);
        // distance: 1.0 * 0.5 = 0.5, bearing: 1.0 * 0.5 = 0.5, frc: 1.0 * (1/3) ≈ 0.333
        let expected_half = 0.5 + 0.5 + (1.0 / 3.0);
        assert!((half_score - expected_half).abs() < 0.001);
    }
}
