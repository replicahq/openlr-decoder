//! Accuracy and speed of the matcher on synthetic traces with known ground
//! truth: random fastest-time routes on a real network, driven at road-class
//! speeds with random stops, sampled with Gaussian position, speed and heading
//! noise.
//!
//! ```text
//! cargo run --release -p map-matcher --example evaluate -- NETWORK.parquet [n_traces] [seed]
//! ```
//!
//! Columns: `edge_ok` is the share of pings matched to their true edge,
//! `on_route` the share matched within three edges of it along the true route,
//! `trace_ok` the share of traces with every ping on route, and `recall` /
//! `precis` the overlap of matched and true edges by length.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::path::PathBuf;
use std::time::Instant;

use geo::{Bearing, Distance, Geodesic, Haversine, LineString, Point};
use map_matcher::{MatchConfig, MatchResult, Matcher, Ping};
use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use rayon::prelude::*;
use road_network::{load_network_from_parquet, Edge, Fow, Frc, RoadNetwork};

/// SplitMix64, so runs are reproducible without a dependency.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn f(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.f()
    }
    fn normal(&mut self) -> f64 {
        let u1 = self.f().max(1e-12);
        let u2 = self.f();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
    fn below(&mut self, n: usize) -> usize {
        ((self.f() * n as f64) as usize).min(n - 1)
    }
}

fn cum_lengths(g: &LineString<f64>) -> Vec<f64> {
    let mut cum = vec![0.0];
    for w in g.0.windows(2) {
        let d = Haversine.distance(Point::from(w[0]), Point::from(w[1]));
        cum.push(cum.last().unwrap() + d);
    }
    cum
}

fn point_along(g: &LineString<f64>, cum: &[f64], s: f64) -> Point<f64> {
    let s = s.clamp(0.0, *cum.last().unwrap());
    let pts = &g.0;
    for i in 0..pts.len() - 1 {
        if s <= cum[i + 1] || i == pts.len() - 2 {
            let seg = cum[i + 1] - cum[i];
            let f = if seg > 0.0 { (s - cum[i]) / seg } else { 0.0 };
            return Point::new(
                pts[i].x + f * (pts[i + 1].x - pts[i].x),
                pts[i].y + f * (pts[i + 1].y - pts[i].y),
            );
        }
    }
    Point::from(pts[pts.len() - 1])
}

fn bearing_along(g: &LineString<f64>, cum: &[f64], s: f64) -> f64 {
    let total = *cum.last().unwrap();
    let a = point_along(g, cum, (s - 5.0).max(0.0));
    let b = point_along(g, cum, (s + 5.0).min(total));
    Geodesic.bearing(a, b)
}

fn offset_m(p: Point<f64>, dx: f64, dy: f64) -> Point<f64> {
    Point::new(
        p.x() + dx / (111_320.0 * p.y().to_radians().cos()),
        p.y() + dy / 111_320.0,
    )
}

fn drivable(e: &Edge) -> bool {
    e.frc <= Frc::Frc4
}

/// Free-flow speed used by the simulator, m/s.
fn edge_speed(e: &Edge) -> f64 {
    if e.fow == Fow::SlipRoad {
        15.0
    } else if e.is_access_road {
        9.0
    } else {
        match e.frc {
            Frc::Frc0 => 29.0,
            Frc::Frc1 => 24.0,
            Frc::Frc2 => 18.0,
            Frc::Frc3 => 15.0,
            _ => 12.0,
        }
    }
}

#[derive(Clone, Copy)]
struct HeapItem {
    cost: f64,
    node: NodeIndex,
}
impl PartialEq for HeapItem {
    fn eq(&self, o: &Self) -> bool {
        self.cost == o.cost
    }
}
impl Eq for HeapItem {}
impl PartialOrd for HeapItem {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for HeapItem {
    fn cmp(&self, o: &Self) -> Ordering {
        o.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
    }
}

/// Fastest free-flow route between two nodes over drivable edges, searching
/// up to an hour out.
fn fastest_route(net: &RoadNetwork, from: NodeIndex, to: NodeIndex) -> Option<Vec<EdgeIndex>> {
    let mut dist: HashMap<NodeIndex, f64> = HashMap::new();
    let mut prev: HashMap<NodeIndex, EdgeIndex> = HashMap::new();
    let mut heap = BinaryHeap::new();
    dist.insert(from, 0.0);
    heap.push(HeapItem {
        cost: 0.0,
        node: from,
    });
    while let Some(HeapItem { cost, node }) = heap.pop() {
        if node == to {
            let mut edges = Vec::new();
            let mut n = to;
            while let Some(&e) = prev.get(&n) {
                edges.push(e);
                n = net.edge_source(e).unwrap();
            }
            edges.reverse();
            return Some(edges);
        }
        if cost > dist[&node] || cost > 3600.0 {
            continue;
        }
        for er in net.graph.edges(node) {
            let e = er.weight();
            if !drivable(e) {
                continue;
            }
            let nc = cost + e.length_m / edge_speed(e);
            if dist.get(&er.target()).is_none_or(|&d| nc < d) {
                dist.insert(er.target(), nc);
                prev.insert(er.target(), er.id());
                heap.push(HeapItem {
                    cost: nc,
                    node: er.target(),
                });
            }
        }
    }
    None
}

/// `n` fastest routes between random nodes 2–15 km apart, each at least 5 edges.
fn generate_routes(net: &RoadNetwork, n: usize, rng: &mut Rng) -> Vec<Vec<EdgeIndex>> {
    let nn = net.graph.node_count();
    let has_drivable_out = |v: NodeIndex| net.graph.edges(v).any(|e| drivable(e.weight()));
    let mut routes = Vec::with_capacity(n);
    while routes.len() < n {
        let a = NodeIndex::new(rng.below(nn));
        if !has_drivable_out(a) {
            continue;
        }
        let pa = net.node(a).unwrap().coord;
        let mut b = None;
        for _ in 0..200 {
            let c = NodeIndex::new(rng.below(nn));
            let d = Haversine.distance(pa, net.node(c).unwrap().coord);
            if (2_000.0..15_000.0).contains(&d) {
                b = Some(c);
                break;
            }
        }
        let Some(b) = b else { continue };
        if let Some(r) = fastest_route(net, a, b) {
            if r.len() >= 5 {
                routes.push(r);
            }
        }
    }
    routes
}

/// Where the vehicle really was at a ping.
#[derive(Clone, Copy)]
struct Truth {
    edge: EdgeIndex,
    route_pos: usize,
}

struct Trace {
    route: Vec<EdgeIndex>,
    pings: Vec<Ping>,
    truth: Vec<Truth>,
}

#[derive(Clone, Copy)]
struct SimParams {
    dt: f64,
    sigma_pos: f64,
    sigma_speed: f64,
    sigma_heading: f64,
    stop_prob: f64,
}

/// One stretch of the timeline: driving edge `pos` at speed `v`, or stopped
/// at its end when `v` is 0.
struct Stretch {
    pos: usize,
    t0: f64,
    dur: f64,
    v: f64,
}

fn simulate(net: &RoadNetwork, route: &[EdgeIndex], sp: SimParams, rng: &mut Rng) -> Trace {
    let mult = rng.range(0.85, 1.10);
    let geoms: Vec<(&LineString<f64>, Vec<f64>)> = route
        .iter()
        .map(|&e| {
            let g = &net.edge(e).unwrap().geometry;
            (g, cum_lengths(g))
        })
        .collect();

    // Drive each edge at a constant speed, with random stops at junctions onto
    // non-freeway roads (traffic lights, stop signs).
    let mut timeline = Vec::new();
    let mut t = 0.0;
    for (i, &e) in route.iter().enumerate() {
        let len = *geoms[i].1.last().unwrap();
        let v = edge_speed(net.edge(e).unwrap()) * mult;
        timeline.push(Stretch {
            pos: i,
            t0: t,
            dur: len / v,
            v,
        });
        t += len / v;
        if i + 1 < route.len() {
            let next = net.edge(route[i + 1]).unwrap();
            if next.frc >= Frc::Frc2 && rng.f() < sp.stop_prob {
                let d = rng.range(5.0, 45.0);
                timeline.push(Stretch {
                    pos: i,
                    t0: t,
                    dur: d,
                    v: 0.0,
                });
                t += d;
            }
        }
    }
    let t_end = t;

    let mut pings = Vec::new();
    let mut truth = Vec::new();
    let mut ts = rng.range(0.0, sp.dt);
    let mut si = 0;
    while ts < t_end {
        while si + 1 < timeline.len() && ts >= timeline[si].t0 + timeline[si].dur {
            si += 1;
        }
        let st = &timeline[si];
        let (g, cum) = &geoms[st.pos];
        let len = *cum.last().unwrap();
        let s = if st.v == 0.0 {
            len
        } else {
            ((ts - st.t0) * st.v).min(len)
        };
        let true_pt = point_along(g, cum, s);
        let true_heading = bearing_along(g, cum, s);

        let point = offset_m(
            true_pt,
            rng.normal() * sp.sigma_pos,
            rng.normal() * sp.sigma_pos,
        );
        let speed = (st.v + rng.normal() * sp.sigma_speed).max(0.0);
        let heading = if st.v > 1.0 {
            (true_heading + rng.normal() * sp.sigma_heading + 360.0) % 360.0
        } else {
            rng.range(0.0, 360.0) // GPS course is noise when stationary
        };
        pings.push(Ping {
            point,
            time_s: ts,
            speed_mps: Some(speed),
            heading_deg: Some(heading),
        });
        truth.push(Truth {
            edge: route[st.pos],
            route_pos: st.pos,
        });
        ts += sp.dt;
    }
    Trace {
        route: route.to_vec(),
        pings,
        truth,
    }
}

#[derive(Default)]
struct Eval {
    pings: u64,
    ping_correct: u64,
    ping_on_route: u64,
    traces: u64,
    traces_perfect: u64,
    true_len: f64,
    matched_len: f64,
    overlap_len: f64,
    breaks: usize,
    secs: f64,
}

fn evaluate(net: &RoadNetwork, trace: &Trace, res: &MatchResult, ev: &mut Eval) {
    let mut chosen = vec![None; trace.pings.len()];
    for p in res.segments.iter().flat_map(|s| &s.pings) {
        chosen[p.index] = Some(p.edge);
    }
    let mut all_ok = true;
    for (t, tr) in trace.truth.iter().enumerate() {
        ev.pings += 1;
        if chosen[t] == Some(tr.edge) {
            ev.ping_correct += 1;
        }
        // Edges are short, so a ping near a junction can legitimately land on
        // the neighbouring route edge.
        let lo = tr.route_pos.saturating_sub(3);
        let hi = (tr.route_pos + 3).min(trace.route.len() - 1);
        if chosen[t].is_some_and(|e| trace.route[lo..=hi].contains(&e)) {
            ev.ping_on_route += 1;
        } else {
            all_ok = false;
        }
    }
    ev.traces += 1;
    if all_ok {
        ev.traces_perfect += 1;
    }
    // Overlap over the part of the route the pings cover.
    let lo = trace.truth.first().map_or(0, |t| t.route_pos);
    let hi = trace.truth.last().map_or(0, |t| t.route_pos);
    let truth: HashSet<EdgeIndex> = trace.route[lo..=hi].iter().copied().collect();
    let matched: HashSet<EdgeIndex> = res
        .segments
        .iter()
        .flat_map(|s| s.edges.iter().copied())
        .collect();
    let len = |e: &EdgeIndex| net.edge(*e).unwrap().length_m;
    ev.true_len += truth.iter().map(len).sum::<f64>();
    ev.matched_len += matched.iter().map(len).sum::<f64>();
    ev.overlap_len += truth.intersection(&matched).map(len).sum::<f64>();
    ev.breaks += res.segments.len().saturating_sub(1);
}

fn run(net: &RoadNetwork, m: &Matcher, traces: &[Trace]) -> Eval {
    let mut ev = Eval::default();
    let t0 = Instant::now();
    let results: Vec<MatchResult> = traces
        .iter()
        .map(|tr| m.match_trace(&tr.pings).unwrap())
        .collect();
    ev.secs = t0.elapsed().as_secs_f64();
    for (tr, res) in traces.iter().zip(&results) {
        evaluate(net, tr, res, &mut ev);
    }
    ev
}

fn header() {
    println!(
        "{:<26} {:>8} {:>8} {:>8} {:>8} {:>8} {:>6} {:>9} {:>9}",
        "config",
        "edge_ok",
        "on_route",
        "trace_ok",
        "recall",
        "precis",
        "breaks",
        "ms/ping",
        "pings/s"
    );
}

fn print_row(label: &str, ev: &Eval) {
    let pct = |a: f64, b: f64| if b > 0.0 { 100.0 * a / b } else { 0.0 };
    println!(
        "{:<26} {:>7.1}% {:>7.1}% {:>7.1}% {:>7.1}% {:>7.1}% {:>6} {:>9.3} {:>9.0}",
        label,
        pct(ev.ping_correct as f64, ev.pings as f64),
        pct(ev.ping_on_route as f64, ev.pings as f64),
        pct(ev.traces_perfect as f64, ev.traces as f64),
        pct(ev.overlap_len, ev.true_len),
        pct(ev.overlap_len, ev.matched_len),
        ev.breaks,
        1e3 * ev.secs / ev.pings as f64,
        ev.pings as f64 / ev.secs,
    );
}

/// The model variants the prototype compared: with and without the heading
/// and speed terms.
fn variants(sigma: f64) -> Vec<(&'static str, MatchConfig)> {
    let base = MatchConfig::with_sigma(sigma);
    let with = |use_heading, use_speed| MatchConfig {
        use_heading,
        use_speed,
        ..base.clone()
    };
    vec![
        ("hmm", with(false, false)),
        ("hmm + heading", with(true, false)),
        ("hmm + speed", with(false, true)),
        ("hmm + heading + speed", with(true, true)),
    ]
}

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let path = PathBuf::from(
        args.get(1)
            .expect("usage: evaluate NETWORK.parquet [n_traces] [seed]"),
    );
    let n_traces: usize = args.get(2).map_or(300, |s| s.parse().unwrap());
    let seed: u64 = args.get(3).map_or(42, |s| s.parse().unwrap());

    let t0 = Instant::now();
    let (net, spatial) = load_network_from_parquet(&path)?;
    eprintln!(
        "loaded {} edges in {:.1}s",
        net.edge_count(),
        t0.elapsed().as_secs_f64()
    );

    let mut rng = Rng(seed);
    let t0 = Instant::now();
    let routes = generate_routes(&net, n_traces, &mut rng);
    let km: f64 = routes
        .iter()
        .flatten()
        .map(|&e| net.edge(e).unwrap().length_m)
        .sum::<f64>()
        / 1000.0;
    eprintln!(
        "generated {} routes ({:.0} km total, {:.1} km avg) in {:.1}s",
        routes.len(),
        km,
        km / routes.len() as f64,
        t0.elapsed().as_secs_f64()
    );

    let sim = |dt: f64, sigma: f64, seed: u64| -> Vec<Trace> {
        let mut r = Rng(seed);
        let sp = SimParams {
            dt,
            sigma_pos: sigma,
            sigma_speed: 0.5,
            sigma_heading: 5.0,
            stop_prob: 0.08,
        };
        routes
            .iter()
            .map(|rt| simulate(&net, rt, sp, &mut r))
            .collect()
    };
    let matcher = |config| Matcher::new(&net, &spatial, config);

    println!("\n== beta sweep: dt=15s sigma=10m, heading + speed ==");
    header();
    let traces = sim(15.0, 10.0, seed + 1);
    for beta_m in [10.0, 25.0, 50.0, 100.0, 200.0] {
        let config = MatchConfig {
            beta_m,
            ..MatchConfig::with_sigma(10.0)
        };
        print_row(
            &format!("beta={beta_m}"),
            &run(&net, &matcher(config), &traces),
        );
    }

    for sigma in [5.0, 10.0, 20.0] {
        println!("\n== dt=15s sigma={sigma}m ==");
        header();
        let traces = sim(15.0, sigma, seed + 2);
        for (label, config) in variants(sigma) {
            print_row(label, &run(&net, &matcher(config), &traces));
        }
    }

    println!("\n== sampling interval, sigma=10m, heading + speed ==");
    header();
    for dt in [1.0, 5.0, 15.0, 30.0, 60.0] {
        let traces = sim(dt, 10.0, seed + 3);
        let m = matcher(MatchConfig::with_sigma(10.0));
        print_row(&format!("dt={dt}s"), &run(&net, &m, &traces));
    }

    let traces = sim(15.0, 10.0, seed + 4);
    let m = matcher(MatchConfig::with_sigma(10.0));
    let reps = 20;
    let work: Vec<&Trace> = (0..reps).flat_map(|_| traces.iter()).collect();
    let n_pings: usize = work.iter().map(|t| t.pings.len()).sum();
    let t0 = Instant::now();
    let _results: Vec<MatchResult> = work
        .par_iter()
        .map(|tr| m.match_trace(&tr.pings).unwrap())
        .collect();
    let secs = t0.elapsed().as_secs_f64();
    println!(
        "\n== parallel ({} threads), dt=15s: {} traces, {} pings in {:.2}s -> {:.0} pings/s ==",
        rayon::current_num_threads(),
        work.len(),
        n_pings,
        secs,
        n_pings as f64 / secs,
    );
    Ok(())
}
