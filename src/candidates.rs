use geo::{Closest, HaversineClosestPoint, HaversineDistance, Point};
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
    pub fow_score: f64,           // FOW substitution score (0.0 = incompatible, 1.0 = exact match)
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
    /// Maximum distance from LRP to edge for a valid candidate (meters).
    /// Candidates beyond this distance are rejected even if they pass other filters.
    /// This prevents matching to far-away roads when the correct road is missing.
    pub max_candidate_distance_m: f64,
    // Scoring weights (lower score is better)
    pub distance_weight: f64,
    pub bearing_weight: f64,
    pub frc_weight: f64,
    pub fow_weight: f64,
}

impl Default for CandidateConfig {
    fn default() -> Self {
        CandidateConfig {
            search_radius_m: 100.0,         // 100m search radius for spatial index query
            max_bearing_diff: 90.0, // ±90 degrees bearing tolerance (generous for cross-provider curving roads)
            frc_tolerance: 2,       // Allow ±2 FRC classes
            max_candidates: 10,     // Keep top 10 candidates
            max_candidate_distance_m: 35.0, // Reject candidates > 35m from LRP
            // Scoring weights for cross-provider decoding (lower score is better):
            // Distance heavily dominates - a spatially close match with wrong FRC/FOW
            // is almost always better than a farther match with correct attributes.
            // Bearing is a minor tiebreaker for parallel roads.
            // FRC/FOW are nearly ignored since cross-provider mappings are unreliable.
            distance_weight: 10.0, // Dominant - closest road almost always wins
            bearing_weight: 0.2,   // Minor tiebreaker for parallel roads
            frc_weight: 0.1,       // Small factor - FRC mappings unreliable across providers
            fow_weight: 0.1,       // Small factor - FOW mappings unreliable across providers
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
    // Find nearby edges using bbox query (with buffer for bbox approximation)
    let nearby = spatial.find_nearby(coord, config.search_radius_m * 1.5);

    let mut candidates: Vec<Candidate> = nearby
        .into_iter()
        .filter_map(|env| {
            let edge = network.edge(env.edge_idx)?;

            // Compute precise distance using geometry from the graph
            let closest = match edge.geometry.haversine_closest_point(&coord) {
                Closest::SinglePoint(p) | Closest::Intersection(p) => p,
                Closest::Indeterminate => {
                    // Fallback to first coordinate of the geometry
                    let first = edge.geometry.coords().next()?;
                    Point::new(first.x, first.y)
                }
            };
            let distance_m = coord.haversine_distance(&closest);

            // Filter by actual radius
            if distance_m > config.search_radius_m {
                return None;
            }

            // Compute bearing at the projection point (where LRP projects onto edge)
            let edge_bearing = bearing_at_projection(coord, &edge.geometry);
            let bearing_diff = bearing_difference(bearing, edge_bearing);

            // Check bearing tolerance (hard filter - bearing must be reasonably close)
            if bearing_diff > config.max_bearing_diff {
                return None;
            }

            // Check FRC compatibility (hard filter with tolerance)
            let frc_diff = (edge.frc as i8 - frc as i8).unsigned_abs();
            if frc_diff > config.frc_tolerance {
                return None;
            }

            // FOW is a soft scoring factor, not a hard filter
            // Uses substitution score matrix from official OpenLR spec
            let fow_score = edge.fow.substitution_score(fow);

            // Compute projection fraction
            let projection_fraction =
                crate::spatial::project_point_to_line_fraction(coord, &edge.geometry);

            // Compute score (lower is better)
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
    fow_score: f64,
    config: &CandidateConfig,
) -> f64 {
    // Normalize each component to 0-1 range (for penalty calculation)
    let distance_penalty = distance_m / config.search_radius_m;
    let bearing_penalty = bearing_diff / config.max_bearing_diff;
    let frc_penalty = frc_diff as f64 / config.frc_tolerance as f64;
    // FOW score is already 0-1, but higher is better, so invert it for penalty
    // 1.0 (exact match) -> 0.0 penalty, 0.0 (incompatible) -> 1.0 penalty
    let fow_penalty = 1.0 - fow_score;

    // Weighted sum of penalties
    config.distance_weight * distance_penalty
        + config.bearing_weight * bearing_penalty
        + config.frc_weight * frc_penalty
        + config.fow_weight * fow_penalty
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

        // Perfect match (fow_score=1.0 means exact FOW match) should have score 0
        let score = compute_score(0.0, 0.0, 0, 1.0, &config);
        assert_eq!(score, 0.0);
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

        // Test various distance values with zero bearing/frc diff and perfect FOW
        let score_0m = compute_score(0.0, 0.0, 0, 1.0, &config);
        let score_10m = compute_score(10.0, 0.0, 0, 1.0, &config);
        let score_50m = compute_score(50.0, 0.0, 0, 1.0, &config);
        let score_100m = compute_score(100.0, 0.0, 0, 1.0, &config);

        // Scores should increase with distance
        assert!(score_0m < score_10m);
        assert!(score_10m < score_50m);
        assert!(score_50m < score_100m);

        // At max search radius (100m), distance component should contribute
        // distance_weight * (100 / 100) = 10.0 * 1.0 = 10.0
        assert!((score_100m - 10.0).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_bearing_variations() {
        let config = CandidateConfig::default();

        // Test various bearing differences with zero distance/frc diff and perfect FOW
        let score_0deg = compute_score(0.0, 0.0, 0, 1.0, &config);
        let score_15deg = compute_score(0.0, 15.0, 0, 1.0, &config);
        let score_90deg = compute_score(0.0, 90.0, 0, 1.0, &config);

        // Scores should increase with bearing difference
        assert!(score_0deg < score_15deg);
        assert!(score_15deg < score_90deg);

        // At max bearing diff (90°), bearing component should contribute
        // bearing_weight * (90 / 90) = 0.2 * 1.0 = 0.2
        assert!((score_90deg - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_frc_variations() {
        let config = CandidateConfig::default();

        // Test various FRC differences with zero distance/bearing diff and perfect FOW
        let score_frc0 = compute_score(0.0, 0.0, 0, 1.0, &config);
        let score_frc1 = compute_score(0.0, 0.0, 1, 1.0, &config);
        let score_frc2 = compute_score(0.0, 0.0, 2, 1.0, &config);

        // Scores should increase with FRC difference
        assert!(score_frc0 < score_frc1);
        assert!(score_frc1 < score_frc2);

        // At max FRC diff (2), FRC component should contribute
        // frc_weight * (2 / 2) = 0.1 * 1.0 = 0.1
        assert!((score_frc2 - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_fow_variations() {
        let config = CandidateConfig::default();

        // Test various FOW scores with zero distance/bearing/frc diff
        let score_perfect_fow = compute_score(0.0, 0.0, 0, 1.0, &config); // exact match
        let score_good_fow = compute_score(0.0, 0.0, 0, 0.75, &config); // compatible
        let score_partial_fow = compute_score(0.0, 0.0, 0, 0.5, &config); // partially compatible
        let score_bad_fow = compute_score(0.0, 0.0, 0, 0.0, &config); // incompatible

        // Scores should increase as FOW match gets worse
        assert!(score_perfect_fow < score_good_fow);
        assert!(score_good_fow < score_partial_fow);
        assert!(score_partial_fow < score_bad_fow);

        // At worst FOW (0.0), FOW component should contribute
        // fow_weight * (1.0 - 0.0) = 0.1 * 1.0 = 0.1
        assert!((score_bad_fow - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_compute_score_distance_dominates() {
        let config = CandidateConfig::default();

        // Distance should dominate scoring (weight 10.0 vs 0.2, 0.1, 0.1)
        // A closer edge with worse bearing/frc/fow should beat a farther edge with perfect attributes
        let close_bad = compute_score(10.0, 30.0, 3, 0.0, &config);
        let far_perfect = compute_score(50.0, 0.0, 0, 1.0, &config);

        // close_bad = 10.0 * 0.1 + 0.2 * 1.0 + 0.1 * 1.0 + 0.1 * 1.0 = 1.0 + 0.2 + 0.1 + 0.1 = 1.4
        // far_perfect = 10.0 * 0.5 + 0 + 0 + 0 = 5.0
        assert!(close_bad < far_perfect);
    }

    #[test]
    fn test_compute_score_maximum_values() {
        let config = CandidateConfig::default();

        // Maximum everything (worst possible) should give maximum score
        let max_score = compute_score(
            config.search_radius_m,
            config.max_bearing_diff,
            config.frc_tolerance,
            0.0, // worst FOW score
            &config,
        );

        let expected =
            config.distance_weight + config.bearing_weight + config.frc_weight + config.fow_weight;
        assert!((max_score - expected).abs() < 0.001);
    }

    /// Helper to create a test candidate with a specific score
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
        let mut candidates = [
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

        let mut candidates = [
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

        // Create candidates with realistic computed scores (fow_score=1.0 for all)
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

        // Distance monotonicity (with perfect FOW)
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

        // Bearing monotonicity
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

        // FRC monotonicity
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

        // FOW monotonicity (lower fow_score = worse match = higher penalty)
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
        // Test that custom weights are respected
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

        // With equal weights, all components contribute equally at their max values
        // fow_score=0.0 gives max penalty of 1.0
        let max_score = compute_score(50.0, 45.0, 3, 0.0, &custom_config);
        // Each component: 1.0 * 1.0 = 1.0, total = 4.0
        assert!((max_score - 4.0).abs() < 0.001);

        // Half values should give half score
        // fow_score=0.5 gives penalty of 0.5
        let half_score = compute_score(25.0, 22.5, 1, 0.5, &custom_config);
        // distance: 1.0 * 0.5 = 0.5, bearing: 1.0 * 0.5 = 0.5, frc: 1.0 * (1/3) ≈ 0.333, fow: 1.0 * 0.5 = 0.5
        let expected_half = 0.5 + 0.5 + (1.0 / 3.0) + 0.5;
        assert!((half_score - expected_half).abs() < 0.001);
    }
}
