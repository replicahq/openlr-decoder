use geo::Point;
use petgraph::graph::EdgeIndex;

use crate::graph::{Frc, Fow, RoadNetwork};
use crate::spatial::{bearing_at_projection, bearing_difference, SpatialIndex};

/// A candidate edge for matching an LRP
#[derive(Debug, Clone)]
pub struct Candidate {
    pub edge_idx: EdgeIndex,
    pub distance_m: f64,           // Distance from LRP to edge
    pub bearing_diff: f64,         // Bearing difference in degrees
    pub frc_diff: u8,              // FRC difference (0 = exact match)
    pub fow_compatible: bool,      // FOW compatibility
    pub score: f64,                // Combined score (lower is better)
    pub projection_fraction: f64,  // Where on the edge the LRP projects (0-1)
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
            search_radius_m: 100.0,     // 100m search radius
            max_bearing_diff: 30.0,      // ±30 degrees bearing tolerance
            frc_tolerance: 2,            // Allow ±2 FRC classes
            max_candidates: 10,          // Keep top 10 candidates
            // Scoring weights for cross-provider decoding (HERE → OSM):
            // Distance strongly dominates - LRP should be on top of the correct road
            // Bearing helps distinguish parallel roads going different directions
            // FRC is just a tiebreaker since mappings between providers differ
            distance_weight: 4.0,        // Primary - closest road strongly wins
            bearing_weight: 0.2,         // Minor tiebreaker for parallel roads
            frc_weight: 0.1,             // Minimal - FRC mappings are very approximate
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
            let projection_fraction = crate::spatial::project_point_to_line_fraction(coord, &env.geometry);

            // Compute score (lower is better)
            let score = compute_score(
                distance_m,
                bearing_diff,
                frc_diff,
                config,
            );

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
    fn test_compute_score() {
        let config = CandidateConfig::default();

        // Perfect match should have score 0
        let score = compute_score(0.0, 0.0, 0, &config);
        assert_eq!(score, 0.0);

        // Worse match should have higher score
        let worse_score = compute_score(50.0, 15.0, 1, &config);
        assert!(worse_score > score);
    }
}
