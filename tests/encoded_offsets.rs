//! Encoded positive/negative offsets (whitepaper §12.10) trim the decoded path.
//!
//! ```text
//! N1 --e1-- N2 --e2-- N3 --e3-- N4     each edge 100 m, one way, eastbound
//! ```
//!
//! Arity's `geoid_portion` codes are slices of one parent: the same two LRPs
//! (N1 and N4 here) with different offsets. Each slice must decode to its own
//! edges.

use openlr::{
    Coordinate, Line, LineAttributes, LocationReference, Offset, Offsets, PathAttributes,
};
use openlr_decoder::test_utils::TestNetworkBuilder;
use openlr_decoder::{DecodedPath, Decoder, Fow, Frc, RoadNetwork, SpatialIndex};

const FRC: Frc = Frc::Frc4;
const FOW: Fow = Fow::SingleCarriageway;
const EDGE_M: f64 = 100.0;
/// Degrees of longitude for 100 m at the equator.
const STEP_DEG: f64 = 0.000_898_3;

fn network() -> (RoadNetwork, SpatialIndex) {
    let mut builder = TestNetworkBuilder::new();
    for id in 1..=4 {
        builder = builder.add_node(id, 0.0, (id - 1) as f64 * STEP_DEG);
    }
    builder
        .add_edge(1, 1, 2, EDGE_M, FRC, FOW)
        .add_edge(2, 2, 3, EDGE_M, FRC, FOW)
        .add_edge(3, 3, 4, EDGE_M, FRC, FOW)
        .build()
}

fn lrp(lon: f64, dnp: Option<f64>) -> openlr::Point {
    let frc = openlr::Frc::from_value(FRC as i8).unwrap();
    openlr::Point {
        coordinate: Coordinate::new(lon, 0.0).unwrap(),
        line: LineAttributes {
            frc,
            fow: openlr::Fow::from_value(FOW as i8).unwrap(),
            bearing: openlr::Bearing::from_degrees(90),
        },
        path: dnp.map(|meters| PathAttributes {
            lfrcnp: frc,
            dnp: openlr::Length::from_meters(meters),
        }),
    }
}

/// Decode N1 -> N4 with offsets given as fractions of the 300 m line.
fn decode(positive: f64, negative: f64) -> DecodedPath {
    let (network, spatial) = network();
    let code = openlr::serialize_base64_openlr(&LocationReference::Line(Line {
        points: vec![lrp(0.0, Some(3.0 * EDGE_M)), lrp(3.0 * STEP_DEG, None)],
        offsets: Offsets {
            pos: Offset::from_range(positive),
            neg: Offset::from_range(negative),
        },
    }))
    .unwrap();
    Decoder::new(&network, &spatial)
        .decode(&code)
        .unwrap_or_else(|e| panic!("decoding {code} failed: {e}"))
}

/// Offsets quantize to 1/256 of the line, about 1.2 m here.
fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 2.0,
        "expected about {expected}, got {actual}"
    );
}

#[test]
fn no_offsets_keep_the_whole_line() {
    let decoded = decode(0.0, 0.0);
    assert_eq!(decoded.edge_ids, vec![1, 2, 3]);
    assert_close(decoded.length_m, 300.0);
    assert_close(decoded.positive_offset_m, 0.0);
    assert_close(decoded.negative_offset_m, 0.0);
}

#[test]
fn positive_offset_drops_leading_edges() {
    // The location starts 150 m in: e1 is dropped and 50 m of e2 trimmed.
    let decoded = decode(0.5, 0.0);
    assert_eq!(decoded.edge_ids, vec![2, 3]);
    assert_close(decoded.positive_offset_m, 50.0);
    assert_close(decoded.negative_offset_m, 0.0);
    assert_close(decoded.length_m, 150.0);
    assert_eq!(decoded.primary_edge_id, 3);
}

#[test]
fn negative_offset_drops_trailing_edges() {
    // The location ends 150 m before N4: e3 is dropped and 50 m of e2 trimmed.
    let decoded = decode(0.0, 0.5);
    assert_eq!(decoded.edge_ids, vec![1, 2]);
    assert_close(decoded.positive_offset_m, 0.0);
    assert_close(decoded.negative_offset_m, 50.0);
    assert_close(decoded.length_m, 150.0);
    assert_eq!(decoded.primary_edge_id, 1);
}

#[test]
fn both_offsets_within_one_edge() {
    // 110 m to 190 m: all on e2.
    let decoded = decode(110.0 / 300.0, 110.0 / 300.0);
    assert_eq!(decoded.edge_ids, vec![2]);
    assert_close(decoded.positive_offset_m, 10.0);
    assert_close(decoded.negative_offset_m, 10.0);
    assert_close(decoded.length_m, 80.0);
    assert_eq!(decoded.primary_edge_id, 2);
}

#[test]
fn sibling_portions_split_the_parent() {
    let first = decode(0.0, 2.0 / 3.0);
    let rest = decode(1.0 / 3.0, 0.0);
    assert_eq!(first.edge_ids, vec![1]);
    assert_eq!(rest.edge_ids, vec![2, 3]);
    assert_close(first.length_m + rest.length_m, 300.0);
}
