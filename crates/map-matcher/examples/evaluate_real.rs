//! Quality proxies for matching real GPS pings, which have no ground truth.
//!
//! ```text
//! cargo run --release -p map-matcher --example evaluate_real -- \
//!     NETWORK.parquet PINGS.csv [FLAGGED.jsonl | --trace TRACE_ID]
//! ```
//!
//! PINGS.csv has a header row and the columns
//! `trace_id,time_ms,lat,lon,speed_kph,heading`. Rows of a trace may be in any
//! order. Empty or non-numeric speed and heading count as unknown.
//!
//! For each matcher config it reports:
//! - breaks: segments beyond the first per trace.
//! - res50, res95: distance from each matched ping to its matched point.
//! - heading: share of moving pings (at least 5 m/s) whose heading is within
//!   45° of the matched road's direction.
//! - stop det.: transitions between two stopped pings (both under 1 m/s and at
//!   most 30 m apart) whose matched route is longer than 100 m.
//! - moving det.: other transitions whose route is longer than
//!   1.5 × the straight line + 150 m.
//! - u-turns: transitions whose route turns from an edge straight onto its
//!   reverse. "reversed" means the vehicle's heading before and after (from
//!   the nearest moving pings within a minute) differs by more than 135°, so
//!   the u-turn is probably real; "kept" means it didn't, so the u-turn is
//!   probably a matching artifact; "unknown" means no moving ping on one side.
//!
//! With FLAGGED.jsonl, writes every break, detour and u-turn of the first config as
//! one JSON object per line, for inspection on a map. With `--trace`, prints
//! one trace's match ping by ping instead.

use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;
use std::time::Instant;

use geo::{Distance, Haversine, Point};
use map_matcher::{MatchConfig, MatchResult, MatchedPing, MatchedSegment, Matcher, Ping};
use petgraph::graph::EdgeIndex;
use rayon::prelude::*;
use road_network::spatial::{bearing_along, bearing_difference};
use road_network::{load_network_from_parquet, BearingDirection, RoadNetwork};

const STOP_MPS: f64 = 1.0;
const STOP_CHORD_M: f64 = 30.0;
const STOP_DETOUR_M: f64 = 100.0;
const MOVING_MPS: f64 = 5.0;
const HEADING_OK_DEG: f64 = 45.0;

struct Trace {
    id: String,
    pings: Vec<Ping>,
}

impl Trace {
    fn speed(&self, i: usize) -> Option<f64> {
        self.pings[i].speed_mps
    }

    fn stopped(&self, i: usize) -> bool {
        self.speed(i).is_some_and(|v| v < STOP_MPS)
    }

    fn moving(&self, i: usize) -> bool {
        self.speed(i).is_some_and(|v| v >= MOVING_MPS)
    }

    /// Heading of the nearest moving ping from `i` stepping by `dir` (±1),
    /// within a minute of ping `i`.
    fn moving_heading(&self, i: usize, dir: isize) -> Option<f64> {
        let t0 = self.pings[i].time_s;
        let mut k = i as isize;
        while k >= 0 && (k as usize) < self.pings.len() {
            let p = &self.pings[k as usize];
            if (p.time_s - t0).abs() > 60.0 {
                break;
            }
            if self.moving(k as usize) && p.heading_deg.is_some() {
                return p.heading_deg;
            }
            k += dir;
        }
        None
    }
}

fn read_pings(path: &str) -> anyhow::Result<Vec<Trace>> {
    let mut by_id: HashMap<String, Vec<Ping>> = HashMap::new();
    for line in BufReader::new(File::open(path)?).lines().skip(1) {
        let line = line?;
        let f: Vec<&str> = line.split(',').collect();
        anyhow::ensure!(f.len() >= 6, "expected 6 columns: {line}");
        let num = |s: &str| s.parse::<f64>().ok().filter(|v| v.is_finite());
        let (Some(t), Some(lat), Some(lon)) = (num(f[1]), num(f[2]), num(f[3])) else {
            anyhow::bail!("bad time or position: {line}");
        };
        by_id.entry(f[0].to_string()).or_default().push(Ping {
            point: Point::new(lon, lat),
            time_s: t / 1000.0,
            speed_mps: num(f[4]).map(|kph| kph / 3.6),
            heading_deg: num(f[5]),
        });
    }
    let mut traces: Vec<Trace> = by_id
        .into_iter()
        .map(|(id, mut pings)| {
            pings.sort_by(|a, b| a.time_s.total_cmp(&b.time_s));
            Trace { id, pings }
        })
        .collect();
    traces.sort_by(|a, b| a.id.cmp(&b.id));
    Ok(traces)
}

#[derive(Default)]
struct Eval {
    pings: usize,
    unmatched: usize,
    breaks: usize,
    residuals: Vec<f64>,
    heading_n: usize,
    heading_ok: usize,
    stopped_transitions: usize,
    stop_detours: usize,
    moving_transitions: usize,
    moving_detours: usize,
    uturns: BTreeMap<&'static str, usize>,
    flagged: Vec<String>,
}

/// For each matched ping of the segment, the index of its edge in the path
/// and its distance along the path. A ping more than `slack_m` behind the
/// previous one on the same edge was reached by looping back onto that edge,
/// as the matcher's routes do.
fn path_positions(net: &RoadNetwork, seg: &MatchedSegment, slack_m: f64) -> Vec<(usize, f64)> {
    let mut out = Vec::with_capacity(seg.pings.len());
    let (mut k, mut before) = (0, 0.0);
    let mut prev: Option<&MatchedPing> = None;
    for p in &seg.pings {
        let looped = prev.is_some_and(|q| q.edge == p.edge && p.offset_m < q.offset_m - slack_m);
        if looped && k + 1 < seg.edges.len() {
            before += net.edge(seg.edges[k]).unwrap().length_m;
            k += 1;
        }
        while k + 1 < seg.edges.len() && seg.edges[k] != p.edge {
            before += net.edge(seg.edges[k]).unwrap().length_m;
            k += 1;
        }
        out.push((k, before + p.offset_m));
        prev = Some(p);
    }
    out
}

/// Classifies a u-turn between pings `i` and `j` by where it happens (a dead
/// end, a stop, or while moving) and whether the heading reverses.
fn uturn_class(
    net: &RoadNetwork,
    trace: &Trace,
    i: usize,
    j: usize,
    edge: EdgeIndex,
) -> &'static str {
    let node = net.edge_target(edge).unwrap();
    let back = net.edge_source(edge);
    let dead_end = net.outgoing_edges(node).all(|e| net.edge_target(e) == back);
    let reversed = match (trace.moving_heading(i, -1), trace.moving_heading(j, 1)) {
        (Some(a), Some(b)) => Some(bearing_difference(a, b) > 135.0),
        _ => None,
    };
    let at = if dead_end {
        0
    } else if trace.stopped(i) || trace.stopped(j) {
        1
    } else {
        2
    };
    let names = [
        ["dead end reversed", "dead end kept", "dead end unknown"],
        ["stop reversed", "stop kept", "stop unknown"],
        ["moving reversed", "moving kept", "moving unknown"],
    ];
    names[at][match reversed {
        Some(true) => 0,
        Some(false) => 1,
        None => 2,
    }]
}

fn json_flag(
    net: &RoadNetwork,
    kind: &str,
    trace: &Trace,
    w: &[MatchedPing],
    route_m: Option<f64>,
    edges: &[EdgeIndex],
) -> String {
    let (from, to) = (w[0].index, w[1].index);
    let pings: Vec<String> = trace.pings[from..=to]
        .iter()
        .map(|p| {
            let v = p.speed_mps.map_or("null".into(), |s| format!("{s:.1}"));
            format!(
                "[{:.6},{:.6},{:.1},{v}]",
                p.point.x(),
                p.point.y(),
                p.time_s
            )
        })
        .collect();
    let matched: Vec<String> = w
        .iter()
        .map(|p| {
            let e = net.edge(p.edge).unwrap();
            format!(
                "[{},{:.1},{:.1},{:.1}]",
                e.id, p.offset_m, e.length_m, p.distance_m
            )
        })
        .collect();
    let ids: Vec<u64> = edges.iter().map(|&e| net.edge(e).unwrap().id).collect();
    format!(
        "{{\"kind\":\"{kind}\",\"trace_id\":\"{}\",\"from\":{from},\"to\":{to},\"route_m\":{},\
         \"edges\":{ids:?},\"matched\":[{}],\"pings\":[{}]}}",
        trace.id,
        route_m.map_or("null".into(), |m| format!("{m:.0}")),
        matched.join(","),
        pings.join(",")
    )
}

fn evaluate(
    net: &RoadNetwork,
    trace: &Trace,
    res: &MatchResult,
    slack_m: f64,
    ev: &mut Eval,
    flag: bool,
) {
    ev.pings += trace.pings.len();
    ev.unmatched += res.unmatched.len();
    ev.breaks += res.segments.len().saturating_sub(1);
    if flag {
        for pair in res.segments.windows(2) {
            let w = [*pair[0].pings.last().unwrap(), pair[1].pings[0]];
            ev.flagged
                .push(json_flag(net, "break", trace, &w, None, &[]));
        }
    }
    for seg in &res.segments {
        for p in &seg.pings {
            ev.residuals.push(p.distance_m);
            if let (Some(h), true) = (trace.pings[p.index].heading_deg, trace.moving(p.index)) {
                let edge = net.edge(p.edge).unwrap();
                let b = bearing_along(&edge.geometry, p.offset_m, 10.0, BearingDirection::Forward);
                ev.heading_n += 1;
                ev.heading_ok += (bearing_difference(b, h) <= HEADING_OK_DEG) as usize;
            }
        }
        let pos = path_positions(net, seg, slack_m);
        for (w, d) in seg.pings.windows(2).zip(pos.windows(2)) {
            let (i, j) = (w[0].index, w[1].index);
            let ((ka, da), (kb, db)) = (d[0], d[1]);
            let route_m = db - da;
            let edges = &seg.edges[ka..=kb];
            let mut flagged = |kind: &str| {
                if flag {
                    ev.flagged
                        .push(json_flag(net, kind, trace, w, Some(route_m), edges));
                }
            };
            if let Some(k) = (ka..kb).find(|&k| net.is_reverse_of(seg.edges[k], seg.edges[k + 1])) {
                let class = uturn_class(net, trace, i, j, seg.edges[k]);
                flagged(&format!("u-turn {class}"));
                *ev.uturns.entry(class).or_default() += 1;
            }
            let chord_m = Haversine.distance(trace.pings[i].point, trace.pings[j].point);
            if trace.stopped(i) && trace.stopped(j) && chord_m <= STOP_CHORD_M {
                ev.stopped_transitions += 1;
                if route_m > STOP_DETOUR_M {
                    ev.stop_detours += 1;
                    flagged("stop detour");
                }
            } else {
                ev.moving_transitions += 1;
                if route_m > 1.5 * chord_m + 150.0 {
                    ev.moving_detours += 1;
                    flagged("moving detour");
                }
            }
        }
    }
}

fn print_trace(net: &RoadNetwork, trace: &Trace, res: &MatchResult) {
    let node = |n| net.node(n).unwrap().id;
    for (s, seg) in res.segments.iter().enumerate() {
        println!("segment {s}: {} edges", seg.edges.len());
        for &e in &seg.edges {
            let edge = net.edge(e).unwrap();
            println!(
                "  edge {:>20} {:>12} -> {:>12} len {:6.1} {:?}",
                edge.id,
                node(net.edge_source(e).unwrap()),
                node(net.edge_target(e).unwrap()),
                edge.length_m,
                edge.frc
            );
        }
        for p in &seg.pings {
            let ping = &trace.pings[p.index];
            let edge = net.edge(p.edge).unwrap();
            println!(
                "  ping {:3} t={:5.0} v={:5.1} h={:5.0}  edge {:>20} at {:6.1}/{:6.1} d {:5.1}  {:.6},{:.6}",
                p.index,
                ping.time_s - trace.pings[0].time_s,
                ping.speed_mps.unwrap_or(f64::NAN),
                ping.heading_deg.unwrap_or(f64::NAN),
                edge.id,
                p.offset_m,
                edge.length_m,
                p.distance_m,
                ping.point.y(),
                ping.point.x()
            );
        }
    }
    println!("unmatched: {:?}", res.unmatched);
}

fn pct(v: &mut [f64], q: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    let k = ((v.len() - 1) as f64 * q).round() as usize;
    *v.select_nth_unstable_by(k, |a, b| a.total_cmp(b)).1
}

fn run(net: &RoadNetwork, m: &Matcher, traces: &[Trace], flag: bool) -> (Eval, f64) {
    let t0 = Instant::now();
    let results: Vec<MatchResult> = traces
        .par_iter()
        .map(|t| m.match_trace(&t.pings).unwrap())
        .collect();
    let secs = t0.elapsed().as_secs_f64();
    let mut ev = Eval::default();
    for (t, r) in traces.iter().zip(&results) {
        evaluate(net, t, r, m.config().backward_slack_m, &mut ev, flag);
    }
    (ev, secs)
}

fn print_row(label: &str, ev: &mut Eval, secs: f64) {
    let rate = |n: usize, d: usize| 100.0 * n as f64 / d.max(1) as f64;
    let uturns: usize = ev.uturns.values().sum();
    println!(
        "{:<26} {:>6} {:>9} {:>6.1} {:>6.1} {:>8.1} {:>5} {:>4.1}% {:>6} {:>5.2}% {:>7} {:>11.0}",
        label,
        ev.breaks,
        ev.unmatched,
        pct(&mut ev.residuals, 0.5),
        pct(&mut ev.residuals, 0.95),
        rate(ev.heading_ok, ev.heading_n),
        ev.stop_detours,
        rate(ev.stop_detours, ev.stopped_transitions),
        ev.moving_detours,
        rate(ev.moving_detours, ev.moving_transitions),
        uturns,
        ev.pings as f64 / secs,
    );
    let classes: Vec<String> = ev.uturns.iter().map(|(k, v)| format!("{k} {v}")).collect();
    println!("    u-turns: {}", classes.join(", "));
}

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let usage = "usage: evaluate_real NETWORK.parquet PINGS.csv [FLAGGED.jsonl | --trace TRACE_ID]";
    let (Some(net_path), Some(pings_path)) = (args.get(1), args.get(2)) else {
        anyhow::bail!(usage);
    };

    let t0 = Instant::now();
    let (net, spatial) = load_network_from_parquet(Path::new(net_path))?;
    eprintln!(
        "loaded {} edges in {:.1}s",
        net.edge_count(),
        t0.elapsed().as_secs_f64()
    );
    let traces = read_pings(pings_path)?;

    if args.get(3).map(String::as_str) == Some("--trace") {
        let id = args.get(4).ok_or_else(|| anyhow::anyhow!(usage))?;
        let Some(t) = traces.iter().find(|t| &t.id == id) else {
            anyhow::bail!("no trace {id}");
        };
        let m = Matcher::new(&net, &spatial, MatchConfig::default());
        print_trace(&net, t, &m.match_trace(&t.pings)?);
        return Ok(());
    }
    let flag_path = args.get(3);

    let n: usize = traces.iter().map(|t| t.pings.len()).sum();
    let stopped: usize = traces
        .iter()
        .map(|t| (0..t.pings.len()).filter(|&i| t.stopped(i)).count())
        .sum();
    println!(
        "{} traces, {} pings ({:.0}% under {STOP_MPS} m/s)\n",
        traces.len(),
        n,
        100.0 * stopped as f64 / n as f64
    );

    let default = MatchConfig::default;
    let configs = [
        ("default (sigma=10)", default()),
        (
            "uturn_penalty=0",
            MatchConfig {
                uturn_penalty: 0.0,
                ..default()
            },
        ),
        (
            "no heading or speed",
            MatchConfig {
                use_heading: false,
                use_speed: false,
                ..default()
            },
        ),
        ("sigma=5", MatchConfig::with_sigma(5.0)),
        ("sigma=20", MatchConfig::with_sigma(20.0)),
        (
            "search_radius_m=80",
            MatchConfig {
                search_radius_m: 80.0,
                ..default()
            },
        ),
    ];
    println!(
        "{:<26} {:>6} {:>9} {:>6} {:>6} {:>8} {:>11} {:>13} {:>7} {:>11}",
        "config",
        "breaks",
        "unmatched",
        "res50",
        "res95",
        "heading%",
        "stop det.",
        "moving det.",
        "u-turns",
        "pings/s"
    );
    for (k, (label, config)) in configs.into_iter().enumerate() {
        let m = Matcher::new(&net, &spatial, config);
        let flag = k == 0 && flag_path.is_some();
        let (mut ev, secs) = run(&net, &m, &traces, flag);
        print_row(label, &mut ev, secs);
        if let (true, Some(path)) = (flag, flag_path) {
            let mut w = BufWriter::new(File::create(path)?);
            for line in &ev.flagged {
                writeln!(w, "{line}")?;
            }
            eprintln!("wrote {} flagged cases to {path}", ev.flagged.len());
        }
    }
    Ok(())
}
