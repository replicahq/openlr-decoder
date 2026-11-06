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

/// A one-way loop: east along y=0 from 0 to 300 m (edge 1), then north
/// 1500 m, west 300 m and south 1500 m back to the start (edges 2–4).
fn one_way_loop() -> (RoadNetwork, SpatialIndex) {
    let north = LAT + 1500.0 / 111_000.0;
    TestNetworkBuilder::new()
        .add_node(1, LAT, lon(0.0))
        .add_node(2, LAT, lon(300.0))
        .add_node(3, north, lon(300.0))
        .add_node(4, north, lon(0.0))
        .add_edge_auto_length(1, 1, 2, Frc::Frc3, Fow::SingleCarriageway)
        .add_edge_auto_length(2, 2, 3, Frc::Frc3, Fow::SingleCarriageway)
        .add_edge_auto_length(3, 3, 4, Frc::Frc3, Fow::SingleCarriageway)
        .add_edge_auto_length(4, 4, 1, Frc::Frc3, Fow::SingleCarriageway)
        .build()
}

#[test]
fn test_max_detour_bounds_routes_after_a_long_gap() {
    let (net, sp) = one_way_loop();
    // An hour apart, the second ping 80 m behind the first on the one-way
    // edge and 100 m from the loop's other legs: the only route is the 3.3 km
    // loop.
    let pings = vec![ping(280.0, 0.0, 0.0, None), ping(200.0, 0.0, 3600.0, None)];
    let segments = |max_detour_m| {
        let config = MatchConfig {
            max_detour_m,
            ..MatchConfig::with_sigma(5.0)
        };
        Matcher::new(&net, &sp, config)
            .match_trace(&pings)
            .unwrap()
            .segments
    };
    assert_eq!(segments(2000.0).len(), 2, "loop exceeds the detour cap");
    let joined = segments(5000.0);
    assert_eq!(joined.len(), 1);
    assert_eq!(ids(&net, &joined[0].edges), vec![1, 2, 3, 4, 1]);
}

#[test]
fn test_non_finite_speed_and_heading_count_as_missing() {
    let (net, sp) = two_way_road();
    let m = Matcher::new(&net, &sp, MatchConfig::with_sigma(5.0));
    let pings: Vec<Ping> = [20.0, 120.0, 220.0]
        .iter()
        .enumerate()
        .map(|(i, &x)| Ping {
            speed_mps: Some(if i == 1 { f64::NAN } else { 10.0 }),
            heading_deg: Some(if i == 2 { f64::NAN } else { 90.0 }),
            ..ping(x, 0.0, 10.0 * i as f64, None)
        })
        .collect();
    let r = m.match_trace(&pings).unwrap();
    assert_eq!(r.segments.len(), 1);
    assert_eq!(ids(&net, &r.segments[0].edges), vec![1, 2, 3]);

    let mut bad = pings.clone();
    bad[2].time_s = f64::NAN;
    assert_eq!(
        m.match_trace(&bad),
        Err(MatchError::TimeNotFinite { index: 2 })
    );
}

#[test]
fn test_end_offset_keeps_farthest_point_on_last_edge() {
    let (net, sp) = two_way_road();
    let m = Matcher::new(&net, &sp, MatchConfig::with_sigma(5.0));
    // Drives onto edge 2, then the last fix jitters 4 m back.
    let pings: Vec<Ping> = [50.0, 150.0, 146.0]
        .iter()
        .enumerate()
        .map(|(i, &x)| ping(x, 0.0, 10.0 * i as f64, Some(90.0)))
        .collect();
    let seg = &m.match_trace(&pings).unwrap().segments[0];
    assert_eq!(ids(&net, &seg.edges), vec![1, 2]);
    assert!((seg.end_offset_m - 50.0).abs() < 1.0, "{seg:?}");
}

#[test]
fn test_validate() {
    assert!(MatchConfig::default().validate().is_ok());
    let bad = MatchConfig {
        beta_m: 0.0,
        ..MatchConfig::default()
    };
    assert!(bad.validate().unwrap_err().contains("beta_m"));
    let bad = MatchConfig {
        backward_slack_m: f64::NAN,
        ..MatchConfig::default()
    };
    assert!(bad.validate().unwrap_err().contains("backward_slack_m"));
    let bad = MatchConfig {
        max_candidates: 0,
        ..MatchConfig::default()
    };
    assert!(bad.validate().is_err());
}

#[test]
fn test_impossible_jump_breaks_the_chain() {
    let (net, sp) = two_way_road();
    let m = Matcher::new(&net, &sp, MatchConfig::with_sigma(5.0));
    // 260 m in 1 s is far beyond 40 m/s plus GPS slack, though the road connects.
    let jump = vec![ping(20.0, 0.0, 0.0, None), ping(280.0, 0.0, 1.0, None)];
    assert_eq!(m.match_trace(&jump).unwrap().segments.len(), 2);
    // The same pings 10 s apart are drivable.
    let drive = vec![ping(20.0, 0.0, 0.0, None), ping(280.0, 0.0, 10.0, None)];
    assert_eq!(m.match_trace(&drive).unwrap().segments.len(), 1);
}

#[test]
fn test_match_trace_rejects_invalid_config() {
    let (net, sp) = two_way_road();
    let config = MatchConfig {
        sigma_m: 0.0,
        ..MatchConfig::default()
    };
    let err = Matcher::new(&net, &sp, config)
        .match_trace(&[ping(20.0, 0.0, 0.0, None)])
        .unwrap_err();
    assert!(matches!(err, MatchError::InvalidConfig(ref r) if r.contains("sigma_m")));
}
