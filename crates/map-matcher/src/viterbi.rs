//! Viterbi over the candidates of consecutive pings, and the segments that
//! result.

use petgraph::graph::EdgeIndex;

use crate::candidates::{candidates, routes, straight_line_m, Candidate, Route};
use crate::{MatchResult, MatchedPing, MatchedSegment, Matcher, Ping};

/// The candidates of one ping that had any.
struct Layer {
    ping: usize,
    cands: Vec<Candidate>,
    /// Best log probability of a path ending at each candidate, shifted so the
    /// best is 0.
    score: Vec<f64>,
    /// For each candidate, the previous layer's candidate on its best path and
    /// the route from it. `None` for every candidate when this layer starts a
    /// new chain.
    back: Vec<Option<(usize, Route)>>,
}

pub(crate) fn run(m: &Matcher, pings: &[Ping]) -> MatchResult {
    let c = &m.config;
    let mut layers: Vec<Layer> = Vec::new();
    let mut unmatched = Vec::new();

    for (t, ping) in pings.iter().enumerate() {
        let cands = candidates(m, ping);
        if cands.is_empty() {
            unmatched.push(t);
            continue;
        }
        let emission: Vec<f64> = cands.iter().map(|c| c.emission).collect();
        let mut score = vec![f64::NEG_INFINITY; cands.len()];
        let mut back: Vec<Option<(usize, Route)>> = vec![None; cands.len()];

        if let Some(prev) = layers.last() {
            let p0 = &pings[prev.ping];
            let dt = ping.time_s - p0.time_s;
            let straight = straight_line_m(p0, ping);
            let max_route = (c.max_speed_mps * dt)
                .min(straight + c.max_detour_m)
                .max(straight + 200.0);
            let speed = match (p0.speed(), ping.speed()) {
                (Some(v0), Some(v1)) if c.use_speed && dt > 0.0 => {
                    Some((0.5 * (v0 + v1) * dt, c.speed_sigma_mps * dt))
                }
                _ => None,
            };

            for (i, a) in prev.cands.iter().enumerate() {
                if prev.score[i] == f64::NEG_INFINITY {
                    continue;
                }
                for (j, route) in routes(m, a, &cands, max_route).into_iter().enumerate() {
                    let Some(route) = route else { continue };
                    let len = route.0;
                    let mut transition = -(len - straight).abs() / c.beta_m;
                    if let Some((expected, sigma)) = speed {
                        transition -= 0.5 * ((len - expected) / sigma).powi(2);
                    }
                    let s = prev.score[i] + transition + emission[j];
                    if s > score[j] {
                        score[j] = s;
                        back[j] = Some((i, route));
                    }
                }
            }
        }

        if score.iter().all(|s| *s == f64::NEG_INFINITY) {
            // First ping, or nothing reachable: start a new chain.
            score = emission;
            back = vec![None; cands.len()];
        }
        let best = score.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        score.iter_mut().for_each(|s| *s -= best);
        layers.push(Layer {
            ping: t,
            cands,
            score,
            back,
        });
    }

    MatchResult {
        segments: segments(&layers, &backtrack(&layers)),
        unmatched,
    }
}

/// The chosen candidate in each layer. Each chain ends at its last layer's
/// best candidate and follows the back-pointers to its start.
fn backtrack(layers: &[Layer]) -> Vec<usize> {
    let mut pick = vec![0; layers.len()];
    let mut next: Option<usize> = None;
    for li in (0..layers.len()).rev() {
        let j = next.unwrap_or_else(|| argmax(&layers[li].score));
        pick[li] = j;
        next = layers[li].back[j].as_ref().map(|(i, _)| *i);
    }
    pick
}

fn segments(layers: &[Layer], pick: &[usize]) -> Vec<MatchedSegment> {
    let mut out: Vec<MatchedSegment> = Vec::new();
    for (layer, &j) in layers.iter().zip(pick) {
        let cand = &layer.cands[j];
        let matched = MatchedPing {
            index: layer.ping,
            edge: cand.edge,
            offset_m: cand.offset_m,
            distance_m: cand.distance_m,
        };
        match (&layer.back[j], out.last_mut()) {
            (Some((_, (_, between))), Some(seg)) => {
                // Jitter back along the same edge doesn't undo progress.
                let stayed = between.is_empty() && seg.edges.last() == Some(&cand.edge);
                seg.end_offset_m = if stayed {
                    seg.end_offset_m.max(cand.offset_m)
                } else {
                    cand.offset_m
                };
                for &e in between.iter().chain([&cand.edge]) {
                    push_edge(&mut seg.edges, e);
                }
                seg.pings.push(matched);
            }
            _ => out.push(MatchedSegment {
                edges: vec![cand.edge],
                start_offset_m: cand.offset_m,
                end_offset_m: cand.offset_m,
                pings: vec![matched],
            }),
        }
    }
    out
}

fn push_edge(edges: &mut Vec<EdgeIndex>, e: EdgeIndex) {
    if edges.last() != Some(&e) {
        edges.push(e);
    }
}

fn argmax(v: &[f64]) -> usize {
    (1..v.len()).fold(0, |best, i| if v[i] > v[best] { i } else { best })
}
