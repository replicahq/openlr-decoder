use geo::Point;
use map_matcher::{MatchConfig, MatchError, Matcher, Ping};
use road_network::test_utils::TestNetworkBuilder;
use road_network::{Fow, Frc, RoadNetwork, SpatialIndex};

const LAT: f64 = 39.0;
/// Degrees of longitude per meter at `LAT`.
const DEG_PER_M: f64 = 1.0 / 86_500.0;

fn lon(m: f64) -> f64 {
    -94.5 + m * DEG_PER_M
}

/// A two-way east-west road: nodes every 100 m from 0 to 300 m, eastbound
/// edges 1–3 and westbound edges 11–13.
fn two_way_road() -> (RoadNetwork, SpatialIndex) {
    let mut b = TestNetworkBuilder::new();
    for i in 0..4 {
        b = b.add_node(i, LAT, lon(100.0 * i as f64));
    }
    for i in 0..3 {
        let (a, z) = (i as i64, i as i64 + 1);
        b = b
            .add_edge_auto_length(1 + i, a, z, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge_auto_length(11 + i, z, a, Frc::Frc3, Fow::SingleCarriageway);
    }
    b.build()
}

/// A ping `x_m` east of the road start and `y_m` north of the road.
fn ping(x_m: f64, y_m: f64, time_s: f64, heading_deg: Option<f64>) -> Ping {
    Ping {
        point: Point::new(lon(x_m), LAT + y_m / 111_000.0),
        time_s,
        speed_mps: None,
        heading_deg,
    }
}

fn ids(net: &RoadNetwork, edges: &[petgraph::graph::EdgeIndex]) -> Vec<u64> {
    edges.iter().map(|&e| net.edge(e).unwrap().id).collect()
}

#[test]
fn test_follows_road_in_heading_direction() {
    let (net, sp) = two_way_road();
    let m = Matcher::new(&net, &sp, MatchConfig::with_sigma(5.0));
    let east: Vec<Ping> = [20.0, 120.0, 220.0, 280.0]
        .iter()
        .enumerate()
        .map(|(i, &x)| ping(x, 3.0, 10.0 * i as f64, Some(90.0)))
        .collect();
    let r = m.match_trace(&east).unwrap();
    assert_eq!(r.segments.len(), 1);
    assert!(r.unmatched.is_empty());
    let seg = &r.segments[0];
    assert_eq!(ids(&net, &seg.edges), vec![1, 2, 3]);
    assert!((seg.start_offset_m - 20.0).abs() < 1.0, "{seg:?}");
    assert!((seg.end_offset_m - 80.0).abs() < 1.0, "{seg:?}");
    assert_eq!(seg.pings.len(), 4);

    let west: Vec<Ping> = [280.0, 180.0, 80.0]
        .iter()
        .enumerate()
        .map(|(i, &x)| ping(x, 3.0, 10.0 * i as f64, Some(270.0)))
        .collect();
    let r = m.match_trace(&west).unwrap();
    assert_eq!(ids(&net, &r.segments[0].edges), vec![13, 12, 11]);
}

#[test]
fn test_without_heading_order_of_pings_decides_direction() {
    let (net, sp) = two_way_road();
    let m = Matcher::new(&net, &sp, MatchConfig::with_sigma(5.0));
    let pings: Vec<Ping> = [20.0, 120.0, 220.0]
        .iter()
        .enumerate()
        .map(|(i, &x)| ping(x, 0.0, 10.0 * i as f64, None))
        .collect();
    let r = m.match_trace(&pings).unwrap();
    assert_eq!(ids(&net, &r.segments[0].edges), vec![1, 2, 3]);
}

#[test]
fn test_ping_far_from_roads_is_unmatched_and_chain_continues() {
    let (net, sp) = two_way_road();
    let m = Matcher::new(&net, &sp, MatchConfig::with_sigma(5.0));
    let pings = vec![
        ping(20.0, 0.0, 0.0, Some(90.0)),
        ping(120.0, 500.0, 10.0, Some(90.0)),
        ping(220.0, 0.0, 20.0, Some(90.0)),
    ];
    let r = m.match_trace(&pings).unwrap();
    assert_eq!(r.unmatched, vec![1]);
    assert_eq!(r.segments.len(), 1);
    assert_eq!(ids(&net, &r.segments[0].edges), vec![1, 2, 3]);
    let idx: Vec<usize> = r.segments[0].pings.iter().map(|p| p.index).collect();
    assert_eq!(idx, vec![0, 2]);
}

#[test]
fn test_disconnected_roads_split_into_segments() {
    // Two one-way roads 2 km apart with no connection.
    let (net, sp) = TestNetworkBuilder::new()
        .add_node(1, LAT, lon(0.0))
        .add_node(2, LAT, lon(100.0))
        .add_node(3, LAT, lon(2000.0))
        .add_node(4, LAT, lon(2100.0))
        .add_edge_auto_length(1, 1, 2, Frc::Frc3, Fow::SingleCarriageway)
        .add_edge_auto_length(2, 3, 4, Frc::Frc3, Fow::SingleCarriageway)
        .build();
    let m = Matcher::new(&net, &sp, MatchConfig::with_sigma(5.0));
    let pings = vec![
        ping(20.0, 0.0, 0.0, None),
        ping(80.0, 0.0, 5.0, None),
        ping(2020.0, 0.0, 60.0, None),
        ping(2080.0, 0.0, 65.0, None),
    ];
    let r = m.match_trace(&pings).unwrap();
    assert_eq!(r.segments.len(), 2);
    assert_eq!(ids(&net, &r.segments[0].edges), vec![1]);
    assert_eq!(ids(&net, &r.segments[1].edges), vec![2]);
    assert_eq!(r.segments[1].pings[0].index, 2);
}

#[test]
fn test_jitter_while_stopped_stays_on_edge() {
    let (net, sp) = two_way_road();
    let m = Matcher::new(&net, &sp, MatchConfig::with_sigma(5.0));
    // Stopped around 150 m, the fix wandering a few meters back and forth.
    let pings: Vec<Ping> = [150.0, 147.0, 152.0, 146.0]
        .iter()
        .enumerate()
        .map(|(i, &x)| Ping {
            speed_mps: Some(0.0),
            ..ping(x, 2.0, 15.0 * i as f64, Some(90.0))
        })
        .collect();
    let r = m.match_trace(&pings).unwrap();
    assert_eq!(r.segments.len(), 1);
    let seg = &r.segments[0];
    assert_eq!(seg.edges.len(), 1, "looped instead of staying: {seg:?}");
    assert!(seg.end_offset_m >= seg.start_offset_m);
}

#[test]
fn test_rejects_decreasing_time() {
    let (net, sp) = two_way_road();
    let m = Matcher::new(&net, &sp, MatchConfig::default());
    let pings = vec![ping(0.0, 0.0, 10.0, None), ping(10.0, 0.0, 5.0, None)];
    assert_eq!(
        m.match_trace(&pings),
        Err(MatchError::TimeDecreases { index: 1 })
    );
    assert_eq!(m.match_trace(&[]).unwrap().segments.len(), 0);
}
