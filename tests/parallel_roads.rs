//! Port of the Java reference decoder's ParallelRoadsTest
//! (openlr/decoder/src/test/java/openlr/decoder/ParallelRoadsTest.java).
//!
//! ```text
//!        ____
//! N1---N2____N3---N4
//! ```
//!
//! Every line is drivable both ways. Lines 2 (750 m) and 4 (800 m) both join N2
//! and N3, and each test expects line 2.
//!
//! The LRPs go through binary serialization and [`Decoder::decode`], so the
//! decoder sees quantized values: DNPs in 58.6 m buckets and bearings in 11.25°
//! sectors. Only `test_parallel_roads` is decided by the DNP. In the other two
//! the quantized DNP slightly favors line 4, and line 2 wins on start bearing
//! because line 4 leaves N2 toward an offset midpoint.

use geo::{Bearing, Geodesic, Point};
use openlr::{Coordinate, Line, LineAttributes, LocationReference, Offsets, PathAttributes};
use openlr_decoder::test_utils::TestNetworkBuilder;
use openlr_decoder::{DecodedPath, Decoder, Fow, Frc, RoadNetwork, SpatialIndex};

/// Node coordinates from ParallelRoadsMap.xml, in (lon, lat) order.
const NODE_1: (f64, f64) = (-73.96526, 40.75629);
const NODE_2: (f64, f64) = (-73.96696, 40.75702);
const NODE_3: (f64, f64) = (-73.96922, 40.75797);
const NODE_4: (f64, f64) = (-73.9708, 40.75862);

/// Edge lengths from ParallelRoadsMap.xml, in meters.
const LINE_1_LENGTH: f64 = 500.0; // N1 -> N2
const LINE_2_LENGTH: f64 = 750.0; // N2 -> N3, shorter parallel
const LINE_3_LENGTH: f64 = 550.0; // N3 -> N4
const LINE_4_LENGTH: f64 = 800.0; // N2 -> N3, longer parallel

/// Every LRP and edge uses these, so FRC and FOW never separate candidates.
const FRC: Frc = Frc::Frc5;
const FOW: Fow = Fow::MultipleCarriageway;

/// Geodesic bearing in [0, 360), from (lon, lat) points.
fn compute_bearing(from: (f64, f64), to: (f64, f64)) -> f64 {
    let bearing = Geodesic.bearing(Point::new(from.0, from.1), Point::new(to.0, to.1));
    bearing.rem_euclid(360.0)
}

/// Reverse edges get id + 100. Line 4 detours through an offset midpoint so the
/// two parallels have distinct geometry; its length still comes from
/// `LINE_4_LENGTH`.
fn build_parallel_roads_network() -> (RoadNetwork, SpatialIndex) {
    let mid_4 = (
        (NODE_2.1 + NODE_3.1) / 2.0 + 0.0003,
        (NODE_2.0 + NODE_3.0) / 2.0 - 0.0005,
    ); // (lat, lon)

    let mut builder = TestNetworkBuilder::new();
    for (id, (lon, lat)) in [(1, NODE_1), (2, NODE_2), (3, NODE_3), (4, NODE_4)] {
        builder = builder.add_node(id, lat, lon);
    }
    builder
        .add_edge(1, 1, 2, LINE_1_LENGTH, FRC, FOW)
        .add_edge(101, 2, 1, LINE_1_LENGTH, FRC, FOW)
        .add_edge(2, 2, 3, LINE_2_LENGTH, FRC, FOW)
        .add_edge(102, 3, 2, LINE_2_LENGTH, FRC, FOW)
        .add_edge(3, 3, 4, LINE_3_LENGTH, FRC, FOW)
        .add_edge(103, 4, 3, LINE_3_LENGTH, FRC, FOW)
        .add_edge_via(4, 2, 3, &[mid_4], LINE_4_LENGTH, FRC, FOW)
        .add_edge_via(104, 3, 2, &[mid_4], LINE_4_LENGTH, FRC, FOW)
        .build()
}

/// An LRP at `node` with `bearing` in degrees. `dnp` is `None` for the last LRP.
fn lrp(node: (f64, f64), bearing: f64, dnp: Option<f64>) -> openlr::Point {
    let frc = openlr::Frc::from_value(FRC as i8).unwrap();
    openlr::Point {
        coordinate: Coordinate::new(node.0, node.1).unwrap(),
        line: LineAttributes {
            frc,
            fow: openlr::Fow::from_value(FOW as i8).unwrap(),
            bearing: openlr::Bearing::from_degrees(bearing.round() as u16 % 360),
        },
        path: dnp.map(|meters| PathAttributes {
            lfrcnp: frc,
            dnp: openlr::Length::from_meters(meters),
        }),
    }
}

/// Serializes the LRPs and decodes them on the parallel roads network.
fn decode(points: Vec<openlr::Point>) -> DecodedPath {
    let (network, spatial) = build_parallel_roads_network();
    let code = openlr::serialize_base64_openlr(&LocationReference::Line(Line {
        points,
        offsets: Offsets::ZERO,
    }))
    .unwrap();
    Decoder::new(&network, &spatial)
        .decode(&code)
        .unwrap_or_else(|e| panic!("decoding {code} failed: {e}"))
}

/// N1 -> N4 with the parallel pair in the middle. Expects edges [1, 2, 3].
#[test]
fn test_parallel_roads() {
    let decoded = decode(vec![
        lrp(NODE_1, compute_bearing(NODE_1, NODE_2), Some(1800.0)),
        lrp(NODE_4, compute_bearing(NODE_3, NODE_4), None),
    ]);

    // Via line 2: 500 + 750 + 550 = 1800 m, the DNP. Via line 4: 1850 m.
    assert_eq!(decoded.edge_ids, vec![1, 2, 3]);
}

/// N2 -> N3, covering only the parallel pair. Expects edge [2].
#[test]
fn test_parallel_roads_only() {
    let bearing_n2_n3 = compute_bearing(NODE_2, NODE_3);
    let decoded = decode(vec![
        lrp(NODE_2, bearing_n2_n3, Some(750.0)),
        lrp(NODE_3, bearing_n2_n3, None),
    ]);

    assert_eq!(decoded.edge_ids, vec![2]);
}

/// N1 -> N3, ending on the parallel pair. Expects edges [1, 2].
#[test]
fn test_parallel_roads_at_end() {
    let decoded = decode(vec![
        lrp(NODE_1, compute_bearing(NODE_1, NODE_2), Some(1250.0)),
        lrp(NODE_3, compute_bearing(NODE_2, NODE_3), None),
    ]);

    // Via line 2: 500 + 750 = 1250 m, the DNP. Via line 4: 1300 m.
    assert_eq!(decoded.edge_ids, vec![1, 2]);
}
