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

#[cfg(feature = "python")]
mod python;

use geo::Point;
use petgraph::graph::EdgeIndex;
use road_network::{Frc, RoadNetwork, SpatialIndex};

/// One GPS fix.
#[derive(Debug, Clone, Copy)]
pub struct Ping {
    pub point: Point<f64>,
    /// Seconds from any fixed origin. Must not decrease along a trace.
    pub time_s: f64,
    /// Treated as missing if negative (some devices report -1 for unknown) or
    /// not finite.
    pub speed_mps: Option<f64>,
    /// Course over ground, degrees clockwise from north. Treated as missing if
    /// not finite.
    pub heading_deg: Option<f64>,
}

impl Ping {
    fn speed(&self) -> Option<f64> {
        self.speed_mps.filter(|v| v.is_finite() && *v >= 0.0)
    }

    fn heading(&self) -> Option<f64> {
        self.heading_deg.filter(|h| h.is_finite())
    }
}

#[derive(Debug, Clone)]
pub struct MatchConfig {
    /// Standard deviation of GPS position error. Real errors have a heavy tail,
    /// so set this above the typical error: on Arity pings (median 2.9 m, p95
    /// 16 m) 10 m gave half the detours at stops of 5 m, and on synthetic
    /// traces with a 3–5 m error it matched as many traces entirely correctly
    /// as any other setting.
    pub sigma_m: f64,
    /// Edges further than this from a ping are not candidates. On Arity pings,
    /// three quarters of those 50–80 m from any road were parked, so the 50 m
    /// minimum leaves them unmatched rather than placing them on a road.
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
    /// with the time between pings. Routes more than four of these standard
    /// deviations longer than the expected distance are not searched, unless
    /// they are within the straight-line distance + 200 m.
    pub speed_sigma_mps: f64,
    /// The route search between two pings stops at `max_speed_mps` × the time
    /// between them (or the straight-line distance + 200 m, if more). The cap
    /// applies to reaching the start of the target edge.
    pub max_speed_mps: f64,
    /// Routes more than this much longer than the straight line between pings
    /// are not considered either. This bounds the route search after a long
    /// gap in the trace, where `max_speed_mps` alone would allow a search of
    /// the whole network.
    pub max_detour_m: f64,
    /// A later ping may project this far behind an earlier one on the same
    /// edge and still count as not moving, instead of having looped around.
    pub backward_slack_m: f64,
    /// Log-score cost of a route that turns from an edge straight onto its
    /// reverse. Without it, jitter at a stop pulls the match onto a nearby side
    /// road and back. On real 15 s pings the default 2.0 cut matched u-turns
    /// from 847 to 454 and kept 39 of the 42 where the heading reverses. On
    /// synthetic traces it doubled the share matched entirely correctly.
    pub uturn_penalty: f64,
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
            max_detour_m: 2000.0,
            backward_slack_m: 3.0 * sigma_m,
            uturn_penalty: 2.0,
            max_frc: Frc::Frc4,
        }
    }
}

impl MatchConfig {
    /// Checks that every distance, speed and scale is a positive (or for
    /// `heading_kappa`, `heading_min_speed_mps`, `backward_slack_m` and
    /// `uturn_penalty`, non-negative) finite number and that `max_candidates`
    /// is at least 1.
    pub fn validate(&self) -> Result<(), String> {
        let positive = [
            ("sigma_m", self.sigma_m),
            ("search_radius_m", self.search_radius_m),
            ("beta_m", self.beta_m),
            ("speed_sigma_mps", self.speed_sigma_mps),
            ("max_speed_mps", self.max_speed_mps),
            ("max_detour_m", self.max_detour_m),
        ];
        let non_negative = [
            ("heading_kappa", self.heading_kappa),
            ("heading_min_speed_mps", self.heading_min_speed_mps),
            ("backward_slack_m", self.backward_slack_m),
            ("uturn_penalty", self.uturn_penalty),
        ];
        for (name, v) in positive {
            if !(v.is_finite() && v > 0.0) {
                return Err(format!("{name} must be a positive number, got {v}"));
            }
        }
        for (name, v) in non_negative {
            if !(v.is_finite() && v >= 0.0) {
                return Err(format!("{name} must be a non-negative number, got {v}"));
            }
        }
        if self.max_candidates == 0 {
            return Err("max_candidates must be at least 1".into());
        }
        Ok(())
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
    /// `pings[index].time_s` is NaN or infinite.
    TimeNotFinite { index: usize },
    /// The [`MatchConfig`] failed [`MatchConfig::validate`].
    InvalidConfig(String),
}

impl std::fmt::Display for MatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MatchError::TimeDecreases { index } => {
                write!(f, "ping {index} is earlier than the ping before it")
            }
            MatchError::TimeNotFinite { index } => write!(f, "ping {index} has a non-finite time"),
            MatchError::InvalidConfig(reason) => write!(f, "invalid MatchConfig: {reason}"),
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
        self.config.validate().map_err(MatchError::InvalidConfig)?;
        if let Some(i) = pings.iter().position(|p| !p.time_s.is_finite()) {
            return Err(MatchError::TimeNotFinite { index: i });
        }
        if let Some(i) = (1..pings.len()).find(|&i| pings[i].time_s < pings[i - 1].time_s) {
            return Err(MatchError::TimeDecreases { index: i });
        }
        Ok(viterbi::run(self, pings))
    }
}
