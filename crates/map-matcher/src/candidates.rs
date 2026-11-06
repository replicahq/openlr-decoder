//! Candidate edges for a ping and the routes between candidates.

use std::cell::RefCell;

use geo::Haversine;
use petgraph::graph::{EdgeIndex, NodeIndex};
use road_network::spatial::{bearing_along, bearing_difference, project_onto_line};
use road_network::{BearingDirection, Edge, ShortestPaths};

use crate::{Matcher, Ping};

/// Meters of edge over which a candidate's bearing is measured.
const BEARING_CHORD_M: f64 = 20.0;

/// A point on an edge that a ping might have come from.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Candidate {
    pub edge: EdgeIndex,
    pub offset_m: f64,
    pub distance_m: f64,
    /// Log emission probability, up to a constant.
    pub emission: f64,
}

/// The best `max_candidates` candidates within the search radius, best
/// emission first.
pub(crate) fn candidates(m: &Matcher, ping: &Ping) -> Vec<Candidate> {
    let c = &m.config;
    let heading = ping
        .heading()
        .filter(|_| c.use_heading && ping.speed().is_none_or(|v| v >= c.heading_min_speed_mps));

    let mut out: Vec<Candidate> = m
        .spatial
        .find_nearby(ping.point, c.search_radius_m)
        .into_iter()
        .filter_map(|env| {
            let edge = m.network.edge(env.edge_idx)?;
            if !matchable(edge, m) {
                return None;
            }
            let proj = project_onto_line(ping.point, &edge.geometry)?;
            if proj.distance_m > c.search_radius_m {
                return None;
            }
            let mut emission = -0.5 * (proj.distance_m / c.sigma_m).powi(2);
            if let Some(h) = heading {
                let bearing = bearing_along(
                    &edge.geometry,
                    proj.along_m,
                    BEARING_CHORD_M,
                    BearingDirection::Forward,
                );
                let diff = bearing_difference(h.rem_euclid(360.0), bearing);
                emission += c.heading_kappa * (diff.to_radians().cos() - 1.0);
            }
            Some(Candidate {
                edge: env.edge_idx,
                offset_m: proj.along_m,
                distance_m: proj.distance_m,
                emission,
            })
        })
        .collect();

    // Ties go to the lower edge index so the result doesn't depend on R-tree order.
    out.sort_by(|a, b| b.emission.total_cmp(&a.emission).then(a.edge.cmp(&b.edge)));
    out.truncate(c.max_candidates);
    out
}

fn matchable(edge: &Edge, m: &Matcher) -> bool {
    edge.frc <= m.config.max_frc
}

/// A route from one candidate to another.
#[derive(Debug, Clone)]
pub(crate) struct Route {
    pub len_m: f64,
    /// The edges strictly between the two candidates' edges.
    pub between: Vec<EdgeIndex>,
    /// Both candidates are on the same edge and the route doesn't leave it:
    /// the vehicle moved forward along it, or jittered back within the
    /// backward slack.
    pub same_edge: bool,
}

thread_local! {
    static SEARCH: RefCell<ShortestPaths> = RefCell::new(ShortestPaths::new());
}

/// Routes from `from` to each of `to` whose target edge starts within `max_m`.
/// The distance along the target edge is not capped: enforcing the cap at the
/// target point only splits more traces, since the transition score already
/// penalises a long route. Moving forward along the same edge, or back by up
/// to the backward slack, costs the distance moved (zero when moving back).
pub(crate) fn routes(
    m: &Matcher,
    from: &Candidate,
    to: &[Candidate],
    max_m: f64,
) -> Vec<Option<Route>> {
    let net = m.network;
    let mut out: Vec<Option<Route>> = vec![None; to.len()];

    // Each target edge's source node, with the candidates on that edge.
    let mut targets: Vec<(NodeIndex, Vec<usize>)> = Vec::new();
    for (j, b) in to.iter().enumerate() {
        if b.edge == from.edge && b.offset_m >= from.offset_m - m.config.backward_slack_m {
            out[j] = Some(Route {
                len_m: (b.offset_m - from.offset_m).max(0.0),
                between: Vec::new(),
                same_edge: true,
            });
            continue;
        }
        let Some(node) = net.edge_source(b.edge) else {
            continue;
        };
        match targets.iter_mut().find(|t| t.0 == node) {
            Some(t) => t.1.push(j),
            None => targets.push((node, vec![j])),
        }
    }
    if targets.is_empty() {
        return out;
    }

    let (Some(from_edge), Some(start)) = (net.edge(from.edge), net.edge_target(from.edge)) else {
        return out;
    };
    let to_end = (from_edge.length_m - from.offset_m).max(0.0);
    if to_end > max_m {
        return out;
    }

    let nodes: Vec<NodeIndex> = targets.iter().map(|t| t.0).collect();
    SEARCH.with_borrow_mut(|sp| {
        sp.run(net, start, to_end, max_m, &nodes, |e| {
            matchable(e, m).then_some(e.length_m)
        });
        for (node, js) in &targets {
            let (Some(cost), Some(edges)) = (sp.cost(*node), sp.path_to(net, *node)) else {
                continue;
            };
            for &j in js {
                out[j] = Some(Route {
                    len_m: cost + to[j].offset_m,
                    between: edges.clone(),
                    same_edge: false,
                });
            }
        }
    });
    out
}

/// Great-circle distance between two pings.
pub(crate) fn straight_line_m(a: &Ping, b: &Ping) -> f64 {
    use geo::Distance;
    Haversine.distance(a.point, b.point)
}
