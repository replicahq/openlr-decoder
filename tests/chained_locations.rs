//! Junction completion (#65) end to end, and how it meets encoded offsets.
//!
//! ```text
//! N1 --e1-- N2 --e2-- N3 --e3-- N4 -e4- N5 --e5-- N6    eastbound, one way
//!                               |        |
//!                               e6       e7             side roads
//! ```
//!
//! e1 to e3 and e5 are about 100 m, e4 under 1 m: one HERE junction that OSM
//! splits into N4 and N5. The last LRP sits 0.33 m past N4, under half of e4,
//! so path finding stops at N4. A location starting at that LRP would start on
//! e5, so a location ending there should own e4.

use openlr::{
    Coordinate, Line, LineAttributes, LocationReference, Offset, Offsets, PathAttributes,
};
use openlr_decoder::test_utils::TestNetworkBuilder;
use openlr_decoder::{DecodedPath, Decoder, DecoderConfig, Fow, Frc, RoadNetwork, SpatialIndex};

const FRC: Frc = Frc::Frc4;
const FOW: Fow = Fow::SingleCarriageway;
/// OpenLR coordinates are quantized to 1e-5 degrees (about 1.1 m), so
/// positions are given in those units and the LRPs sit on whole ones.
const UNIT_DEG: f64 = 1e-5;

fn network() -> (RoadNetwork, SpatialIndex) {
    let x = |units: f64| units * UNIT_DEG;
    TestNetworkBuilder::new()
        .add_node(1, 0.0, x(0.0))
        .add_node(2, 0.0, x(90.0))
        .add_node(3, 0.0, x(180.0))
        .add_node(4, 0.0, x(269.7))
        .add_node(5, 0.0, x(270.5))
        .add_node(6, 0.0, x(360.0))
        .add_node(7, x(90.0), x(269.7))
        .add_node(8, -x(90.0), x(270.5))
        .add_edge_auto_length(1, 1, 2, FRC, FOW)
        .add_edge_auto_length(2, 2, 3, FRC, FOW)
        .add_edge_auto_length(3, 3, 4, FRC, FOW)
        .add_edge_auto_length(4, 4, 5, FRC, FOW)
        .add_edge_auto_length(5, 5, 6, FRC, FOW)
        .add_edge_auto_length(6, 4, 7, FRC, FOW)
        .add_edge_auto_length(7, 5, 8, FRC, FOW)
        .build()
}

fn lrp(units: f64, dnp: Option<f64>) -> openlr::Point {
    let frc = openlr::Frc::from_value(FRC as i8).unwrap();
    openlr::Point {
        coordinate: Coordinate::new(units * UNIT_DEG, 0.0).unwrap(),
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

/// Decode N1 to just past N4, with offsets as fractions of the line.
fn decode(positive: f64, negative: f64) -> DecodedPath {
    decode_with(
        positive,
        negative,
        DecoderConfig::default().max_junction_completion_m,
    )
}

fn decode_with(positive: f64, negative: f64, max_junction_completion_m: f64) -> DecodedPath {
    let (network, spatial) = network();
    let code = openlr::serialize_base64_openlr(&LocationReference::Line(Line {
        points: vec![lrp(0.0, Some(300.0)), lrp(270.0, None)],
        offsets: Offsets {
            pos: Offset::from_range(positive),
            neg: Offset::from_range(negative),
        },
    }))
    .unwrap();
    Decoder::new(&network, &spatial)
        .with_config(DecoderConfig {
            max_junction_completion_m,
            ..DecoderConfig::default()
        })
        .decode(&code)
        .unwrap_or_else(|e| panic!("decoding {code} failed: {e}"))
}

#[test]
fn location_ending_at_split_junction_owns_the_junction_edge() {
    let decoded = decode(0.0, 0.0);
    assert_eq!(decoded.edge_ids, vec![1, 2, 3, 4]);
    assert!(decoded.negative_offset_m.abs() < 1e-6);
}

/// A negative offset is measured back from the LRP. Carrying the path end
/// past it would move the cut downstream with it, so a location with one
/// decodes as if completion were off.
#[test]
fn negative_offset_is_not_shifted_by_completion() {
    // Cut halfway along e2, away from a junction, where a shift would show.
    let with = decode_with(0.0, 0.5, 15.0);
    let without = decode_with(0.0, 0.5, 0.0);
    assert_eq!(with.edge_ids, vec![1, 2]);
    assert_eq!(with.edge_ids, without.edge_ids);
    assert!(
        (with.negative_offset_m - without.negative_offset_m).abs() < 1e-6,
        "negative offset {} vs {}",
        with.negative_offset_m,
        without.negative_offset_m
    );
    assert!((with.length_m - without.length_m).abs() < 1e-6);
}
