//! HMM map matching of GPS traces onto a [`RoadNetwork`], after Newson &
//! Krumm (2009), "Hidden Markov Map Matching Through Noise and Sparseness".
//!
//! Each ping's hidden state is a point on a nearby edge. The emission score
//! is a Gaussian in the ping's distance to that point, plus a von Mises term
//! in the heading difference when the ping has a heading. The transition score
//! between consecutive pings is exponential in the difference between the
//! route distance and the straight-line distance, plus a Gaussian in the
//! difference between the route distance and the distance implied by the
//! pings' speeds when both have one. Viterbi picks the best sequence; where no
//! route connects consecutive pings, the chain breaks and a new segment starts.
//!
//! Distances are meters, times seconds, speeds m/s and headings degrees
//! clockwise from north.

mod candidates;
mod viterbi;

use geo::Point;
use petgraph::graph::EdgeIndex;
use road_network::{Frc, RoadNetwork, SpatialIndex};

/// One GPS fix.
#[derive(Debug, Clone, Copy)]
pub struct Ping {
    pub point: Point<f64>,
    /// Seconds from any fixed origin. Must not decrease along a trace.
    pub time_s: f64,
    pub speed_mps: Option<f64>,
    /// Course over ground, degrees clockwise from north.
    pub heading_deg: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct MatchConfig {
    /// Standard deviation of GPS position error.
    pub sigma_m: f64,
    /// Edges further than this from a ping are not candidates.
    pub search_radius_m: f64,
    /// Candidates kept per ping, best emission first.
    pub max_candidates: usize,
    /// Scale of the transition penalty on |route distance − straight-line
    /// distance|. Larger values tolerate more detour between pings.
    pub beta_m: f64,
    /// Score candidates by heading difference when a ping has a heading.
    pub use_heading: bool,
    /// Concentration of the heading term: the score drops by `kappa·(1 − cos Δ)`.
    pub heading_kappa: f64,
    /// Ignore a ping's heading below this speed, where GPS course is noise.
    /// Pings without a speed always use their heading.
    pub heading_min_speed_mps: f64,
    /// Score transitions by how well the route distance matches the pings'
    /// average speed times the time between them, when both pings have a speed.
    pub use_speed: bool,
    /// Standard deviation of that speed, so the route distance tolerance grows
    /// with the time between pings.
    pub speed_sigma_mps: f64,
    /// Routes longer than `max_speed_mps` × the time between pings (or the
    /// straight-line distance + 200 m, if more) are not considered.
    pub max_speed_mps: f64,
    /// A later ping may project this far behind an earlier one on the same
    /// edge and still count as not moving, instead of having looped around.
    pub backward_slack_m: f64,
    /// Only edges of this road class or more important are matched.
    pub max_frc: Frc,
}

impl MatchConfig {
    /// Defaults scaled to a GPS error of `sigma_m`: a search radius of 4σ (at
    /// least 50 m) and a backward slack of 3σ.
    pub fn with_sigma(sigma_m: f64) -> Self {
        MatchConfig {
            sigma_m,
            search_radius_m: (4.0 * sigma_m).max(50.0),
            max_candidates: 8,
            beta_m: 50.0,
            use_heading: true,
            heading_kappa: 4.0,
            heading_min_speed_mps: 3.0,
            use_speed: true,
            speed_sigma_mps: 4.0,
            max_speed_mps: 40.0,
            backward_slack_m: 3.0 * sigma_m,
            max_frc: Frc::Frc4,
        }
    }
}

impl Default for MatchConfig {
    fn default() -> Self {
        Self::with_sigma(10.0)
    }
}

/// Where one ping was matched.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchedPing {
    /// Index into the input pings.
    pub index: usize,
    pub edge: EdgeIndex,
    /// Meters along `edge` from its start.
    pub offset_m: f64,
    /// Meters from the ping to its matched point.
    pub distance_m: f64,
}

/// A stretch of the trace matched as one connected path.
#[derive(Debug, Clone, PartialEq)]
pub struct MatchedSegment {
    /// The path, in travel order, with no edge repeated back to back.
    pub edges: Vec<EdgeIndex>,
    /// Meters along the first edge where the path starts.
    pub start_offset_m: f64,
    /// Meters along the last edge where the path ends.
    pub end_offset_m: f64,
    /// The matched pings, in order. Pings with no candidates are skipped and
    /// listed in [`MatchResult::unmatched`] instead.
    pub pings: Vec<MatchedPing>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct MatchResult {
    /// One segment per unbroken chain. A new segment starts where no route
    /// within the limits connects a ping to the previous matched one.
    pub segments: Vec<MatchedSegment>,
    /// Pings with no candidate edge within the search radius.
    pub unmatched: Vec<usize>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MatchError {
    /// `pings[index]` is earlier than the ping before it.
    TimeDecreases { index: usize },
}

impl std::fmt::Display for MatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MatchError::TimeDecreases { index } => {
                write!(f, "ping {index} is earlier than the ping before it")
            }
        }
    }
}

impl std::error::Error for MatchError {}

pub struct Matcher<'a> {
    network: &'a RoadNetwork,
    spatial: &'a SpatialIndex,
    config: MatchConfig,
}

impl<'a> Matcher<'a> {
    pub fn new(network: &'a RoadNetwork, spatial: &'a SpatialIndex, config: MatchConfig) -> Self {
        Matcher {
            network,
            spatial,
            config,
        }
    }

    pub fn config(&self) -> &MatchConfig {
        &self.config
    }

    /// Match one trace. Safe to call from many threads at once.
    pub fn match_trace(&self, pings: &[Ping]) -> Result<MatchResult, MatchError> {
        if let Some(i) = (1..pings.len()).find(|&i| pings[i].time_s < pings[i - 1].time_s) {
            return Err(MatchError::TimeDecreases { index: i });
        }
        Ok(viterbi::run(self, pings))
    }
}
