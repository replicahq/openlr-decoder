//! Build a [`RoadNetwork`] and [`SpatialIndex`] from Arrow record batches or a
//! parquet file. [`road_network_schema`] documents the expected columns.

use ahash::AHashMap;
use std::fs::File;
use std::path::Path;

use anyhow::{anyhow, Context, Result};
use arrow::array::{
    Array, ArrayRef, BinaryArray, BinaryViewArray, Float64Array, Int64Array, LargeBinaryArray,
    LargeStringArray, RecordBatch, StringArray, StringViewArray, UInt64Array,
};
use arrow::datatypes::{DataType, Field, Schema};
use geo::{Bearing, Geodesic, LineString, Point};
use geoarrow_array::array::LineStringArray;
use geoarrow_array::{GeoArrowArray, GeoArrowArrayAccessor};
use geoarrow_schema::GeoArrowType;
use geozero::{wkb, wkt, ToGeo};
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use rayon::prelude::*;

use crate::graph::{Edge, Fow, Frc, Node, RoadNetwork};
use crate::spatial::{EdgeEnvelope, SpatialIndex};

enum GeometryFormat<'a> {
    Wkb(WkbSource<'a>),
    Wkt(WktSource<'a>),
    GeoArrow(LineStringArray),
    None,
}

enum WkbSource<'a> {
    Binary(&'a BinaryArray),
    LargeBinary(&'a LargeBinaryArray),
    BinaryView(&'a BinaryViewArray),
}

enum WktSource<'a> {
    String(&'a StringArray),
    LargeString(&'a LargeStringArray),
    StringView(&'a StringViewArray),
}

struct GeometryWithMetrics {
    geometry: LineString<f64>,
    length_m: f64,
    bearing_start: f64,
    bearing_end: f64,
}

/// The Arrow schema the loader reads. Extra columns are ignored.
///
/// - `stableEdgeId` (UInt64), `startOsmNode`/`endOsmNode` (Int64): rows with a
///   null in any of these are skipped. Node ids are OSM nodes, so edges split
///   at barriers still connect.
/// - `startLat`, `startLon`, `endLat`, `endLon` (Float64), degrees.
/// - `highway`: OSM tag, mapped by [`Frc::from_osm_highway`] and
///   [`Fow::from_osm_tags`]. Any string type (String, LargeString, StringView).
/// - `lanes` (Int64, optional): only used for FOW. Other types are ignored.
/// - `geometry` (optional): WKB as any binary type, WKT as any string type, or
///   GeoArrow `geoarrow.linestring`. Only the first part of a MultiLineString
///   is kept. Missing or unparseable geometry falls back to the straight line
///   between the endpoints.
pub fn road_network_schema() -> Schema {
    Schema::new(vec![
        Field::new("stableEdgeId", DataType::UInt64, false),
        Field::new("startOsmNode", DataType::Int64, false),
        Field::new("endOsmNode", DataType::Int64, false),
        Field::new("startLat", DataType::Float64, false),
        Field::new("startLon", DataType::Float64, false),
        Field::new("endLat", DataType::Float64, false),
        Field::new("endLon", DataType::Float64, false),
        Field::new("highway", DataType::Utf8, false),
        Field::new("lanes", DataType::Int64, true),
        Field::new("geometry", DataType::Binary, true),
    ])
}

/// Load a parquet file with the columns described in [`road_network_schema`].
pub fn load_network_from_parquet(path: &Path) -> Result<(RoadNetwork, SpatialIndex)> {
    let file = File::open(path).context("Failed to open parquet file")?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;

    let edge_count_hint = builder
        .metadata()
        .file_metadata()
        .num_rows()
        .try_into()
        .unwrap_or(0usize);

    let reader = builder.build()?;

    build_network_from_batches_with_hint(reader.map(|r| r.map_err(|e| anyhow!(e))), edge_count_hint)
}

/// Build a network from batches with the columns described in
/// [`road_network_schema`].
pub fn build_network_from_batches<I>(batches: I) -> Result<(RoadNetwork, SpatialIndex)>
where
    I: Iterator<Item = Result<RecordBatch>>,
{
    build_network_from_batches_with_hint(batches, 0)
}

fn build_network_from_batches_with_hint<I>(
    batches: I,
    edge_count_hint: usize,
) -> Result<(RoadNetwork, SpatialIndex)>
where
    I: Iterator<Item = Result<RecordBatch>>,
{
    let mut builder = NetworkBuilder::with_capacity(edge_count_hint);

    for batch_result in batches {
        let batch = batch_result?;
        builder.push_batch(&batch)?;
    }

    Ok(builder.finish())
}

/// [`build_network_from_batches`] one batch at a time. The Python bindings
/// fetch each batch from an Arrow stream under the GIL, release the GIL to
/// process it, and drop it before fetching the next, so the whole stream is
/// never held in memory.
pub struct NetworkBuilder {
    network: RoadNetwork,
    edge_envelopes: Vec<EdgeEnvelope>,
    // Node ids already added to `network`.
    seen_nodes: AHashMap<i64, Point<f64>>,
}

impl NetworkBuilder {
    /// Pre-allocate for `edge_count_hint` edges; 0 if unknown.
    pub fn with_capacity(edge_count_hint: usize) -> Self {
        // Road networks have roughly as many nodes as edges.
        let node_hint = edge_count_hint;

        let network = if edge_count_hint > 0 {
            RoadNetwork::new_with_capacity(node_hint, edge_count_hint)
        } else {
            RoadNetwork::new()
        };

        NetworkBuilder {
            network,
            edge_envelopes: Vec::with_capacity(edge_count_hint),
            seen_nodes: AHashMap::with_capacity(node_hint),
        }
    }

    pub fn push_batch(&mut self, batch: &RecordBatch) -> Result<()> {
        process_batch(
            batch,
            &mut self.network,
            &mut self.edge_envelopes,
            &mut self.seen_nodes,
        )
    }

    pub fn finish(mut self) -> (RoadNetwork, SpatialIndex) {
        // Free this before the R-tree bulk load to lower peak memory.
        drop(self.seen_nodes);

        let spatial_index = SpatialIndex::new(self.edge_envelopes);

        self.network.compact();

        (self.network, spatial_index)
    }
}

impl Default for NetworkBuilder {
    fn default() -> Self {
        Self::with_capacity(0)
    }
}

/// An edge built in the parallel phase of [`process_batch`], waiting for
/// sequential insertion into the graph.
struct PendingEdge {
    sv_id: i64,
    ev_id: i64,
    edge: Edge,
}

fn process_batch(
    batch: &RecordBatch,
    network: &mut RoadNetwork,
    edge_envelopes: &mut Vec<EdgeEnvelope>,
    seen_nodes: &mut AHashMap<i64, Point<f64>>,
) -> Result<()> {
    fn require_column<'a, T: 'static>(batch: &'a RecordBatch, name: &str) -> Result<&'a T> {
        let col = batch
            .column_by_name(name)
            .ok_or_else(|| anyhow!("Missing required column '{name}'"))?;
        col.as_any().downcast_ref::<T>().ok_or_else(|| {
            anyhow!(
                "Column '{name}' has type {:?}, expected {:?}",
                col.data_type(),
                std::any::type_name::<T>()
            )
        })
    }

    let stable_edge_id = require_column::<UInt64Array>(batch, "stableEdgeId")?;
    let start_vertex = require_column::<Int64Array>(batch, "startOsmNode")?;
    let end_vertex = require_column::<Int64Array>(batch, "endOsmNode")?;
    let start_lat = require_column::<Float64Array>(batch, "startLat")?;
    let start_lon = require_column::<Float64Array>(batch, "startLon")?;
    let end_lat = require_column::<Float64Array>(batch, "endLat")?;
    let end_lon = require_column::<Float64Array>(batch, "endLon")?;

    // Polars exports strings as StringView.
    let highway_col = batch
        .column_by_name("highway")
        .ok_or_else(|| anyhow!("Missing required column 'highway'"))?;
    let highway_string = highway_col.as_any().downcast_ref::<StringArray>();
    let highway_large = highway_col.as_any().downcast_ref::<LargeStringArray>();
    let highway_view = highway_col.as_any().downcast_ref::<StringViewArray>();
    if highway_string.is_none() && highway_large.is_none() && highway_view.is_none() {
        return Err(anyhow!(
            "Column 'highway' has type {:?}, expected String, LargeString, or StringView",
            highway_col.data_type()
        ));
    }

    let lanes = batch
        .column_by_name("lanes")
        .and_then(|c| c.as_any().downcast_ref::<Int64Array>());

    let geometry_format = detect_geometry_format(batch);

    // Dedupe the batch's nodes locally first so the long-lived `seen_nodes` map
    // is probed once per unique node rather than twice per edge. The first
    // coordinate seen for a node id wins.
    let mut batch_nodes: AHashMap<i64, Point<f64>> = AHashMap::new();
    for row in 0..batch.num_rows() {
        if stable_edge_id.is_null(row) || start_vertex.is_null(row) || end_vertex.is_null(row) {
            continue;
        }
        let sv_id = start_vertex.value(row);
        let ev_id = end_vertex.value(row);
        let sv_lat = start_lat.value(row);
        let sv_lon = start_lon.value(row);
        let ev_lat = end_lat.value(row);
        let ev_lon = end_lon.value(row);

        batch_nodes
            .entry(sv_id)
            .or_insert_with(|| Point::new(sv_lon, sv_lat));
        batch_nodes
            .entry(ev_id)
            .or_insert_with(|| Point::new(ev_lon, ev_lat));
    }

    for (node_id, coord) in batch_nodes {
        if let std::collections::hash_map::Entry::Vacant(e) = seen_nodes.entry(node_id) {
            e.insert(coord);
            network.add_node(Node { id: node_id, coord });
        }
    }

    // Build edges in parallel; only graph insertion has to be sequential.
    let pending_edges: Vec<Option<PendingEdge>> = (0..batch.num_rows())
        .into_par_iter()
        .map(|row| {
            if stable_edge_id.is_null(row) || start_vertex.is_null(row) || end_vertex.is_null(row) {
                return None;
            }

            let edge_id = stable_edge_id.value(row);
            let sv_id = start_vertex.value(row);
            let ev_id = end_vertex.value(row);
            let sv_lat = start_lat.value(row);
            let sv_lon = start_lon.value(row);
            let ev_lat = end_lat.value(row);
            let ev_lon = end_lon.value(row);

            let hw_tag: &str = if let Some(hw) = highway_string {
                if hw.is_null(row) {
                    ""
                } else {
                    hw.value(row)
                }
            } else if let Some(hw) = highway_large {
                if hw.is_null(row) {
                    ""
                } else {
                    hw.value(row)
                }
            } else if let Some(hw) = highway_view {
                if hw.is_null(row) {
                    ""
                } else {
                    hw.value(row)
                }
            } else {
                ""
            };
            let frc = Frc::from_osm_highway(hw_tag);

            let lane_count = lanes.and_then(|l| {
                if l.is_null(row) {
                    None
                } else {
                    Some(l.value(row) as u8)
                }
            });
            let fow = Fow::from_osm_tags(hw_tag, None, None, lane_count);

            let fallback = || LineString::from(vec![(sv_lon, sv_lat), (ev_lon, ev_lat)]);

            let geom: LineString<f64> = match &geometry_format {
                GeometryFormat::Wkb(source) => {
                    let bytes = match source {
                        WkbSource::Binary(arr) => {
                            if arr.is_null(row) {
                                None
                            } else {
                                Some(arr.value(row))
                            }
                        }
                        WkbSource::LargeBinary(arr) => {
                            if arr.is_null(row) {
                                None
                            } else {
                                Some(arr.value(row))
                            }
                        }
                        WkbSource::BinaryView(arr) => {
                            if arr.is_null(row) {
                                None
                            } else {
                                Some(arr.value(row))
                            }
                        }
                    };
                    bytes
                        .and_then(parse_wkb_linestring)
                        .unwrap_or_else(fallback)
                }
                GeometryFormat::Wkt(source) => {
                    let text = match source {
                        WktSource::String(arr) => {
                            if arr.is_null(row) {
                                None
                            } else {
                                Some(arr.value(row))
                            }
                        }
                        WktSource::LargeString(arr) => {
                            if arr.is_null(row) {
                                None
                            } else {
                                Some(arr.value(row))
                            }
                        }
                        WktSource::StringView(arr) => {
                            if arr.is_null(row) {
                                None
                            } else {
                                Some(arr.value(row))
                            }
                        }
                    };
                    text.and_then(parse_wkt_linestring).unwrap_or_else(fallback)
                }
                GeometryFormat::GeoArrow(arr) => {
                    if arr.is_null(row) {
                        fallback()
                    } else {
                        parse_geoarrow_linestring(arr, row).unwrap_or_else(fallback)
                    }
                }
                GeometryFormat::None => fallback(),
            };

            let geom_with_metrics = linestring_with_metrics(geom);

            let edge = Edge {
                id: edge_id,
                geometry: geom_with_metrics.geometry,
                frc,
                fow,
                length_m: geom_with_metrics.length_m,
                bearing_start: geom_with_metrics.bearing_start,
                bearing_end: geom_with_metrics.bearing_end,
                is_access_road: Frc::is_access_road(hw_tag),
            };

            Some(PendingEdge { sv_id, ev_id, edge })
        })
        .collect();

    for pending in pending_edges.into_iter().flatten() {
        let edge_idx = network.add_edge(pending.sv_id, pending.ev_id, pending.edge);

        let edge_geom = &network.edge(edge_idx).unwrap().geometry;
        edge_envelopes.push(EdgeEnvelope::new(edge_idx, edge_geom));
    }

    Ok(())
}

/// A `geoarrow.linestring` extension wins; otherwise any binary type is WKB and
/// any string type is WKT.
fn detect_geometry_format<'a>(batch: &'a RecordBatch) -> GeometryFormat<'a> {
    let schema = batch.schema();
    let Some((idx, field)) = schema.column_with_name("geometry") else {
        return GeometryFormat::None;
    };
    let col = batch.column(idx);

    if let Some(ext_name) = field.metadata().get("ARROW:extension:name") {
        if ext_name.starts_with("geoarrow.linestring") {
            if let Some(arr) = try_parse_geoarrow_linestring(col.clone(), field) {
                return GeometryFormat::GeoArrow(arr);
            }
        }
    }

    if let Some(arr) = col.as_any().downcast_ref::<BinaryArray>() {
        return GeometryFormat::Wkb(WkbSource::Binary(arr));
    }
    if let Some(arr) = col.as_any().downcast_ref::<LargeBinaryArray>() {
        return GeometryFormat::Wkb(WkbSource::LargeBinary(arr));
    }
    if let Some(arr) = col.as_any().downcast_ref::<BinaryViewArray>() {
        return GeometryFormat::Wkb(WkbSource::BinaryView(arr));
    }

    if let Some(arr) = col.as_any().downcast_ref::<StringArray>() {
        return GeometryFormat::Wkt(WktSource::String(arr));
    }
    if let Some(arr) = col.as_any().downcast_ref::<LargeStringArray>() {
        return GeometryFormat::Wkt(WktSource::LargeString(arr));
    }
    if let Some(arr) = col.as_any().downcast_ref::<StringViewArray>() {
        return GeometryFormat::Wkt(WktSource::StringView(arr));
    }

    GeometryFormat::None
}

fn try_parse_geoarrow_linestring(col: ArrayRef, field: &Field) -> Option<LineStringArray> {
    let geo_type = GeoArrowType::try_from(field).ok()?;
    let line_type = match geo_type {
        GeoArrowType::LineString(t) => t,
        _ => return None,
    };

    LineStringArray::try_from((col.as_ref(), line_type)).ok()
}

fn parse_wkb_linestring(wkb_bytes: &[u8]) -> Option<LineString<f64>> {
    let geom = wkb::Wkb(wkb_bytes.to_vec()).to_geo().ok()?;
    extract_linestring(geom)
}

fn parse_wkt_linestring(wkt_str: &str) -> Option<LineString<f64>> {
    let geom = wkt::Wkt(wkt_str).to_geo().ok()?;
    extract_linestring(geom)
}

fn parse_geoarrow_linestring(arr: &LineStringArray, row: usize) -> Option<LineString<f64>> {
    use geo_traits::{CoordTrait, LineStringTrait};

    let ls = arr.value(row).ok()?;
    let coords: Vec<_> = ls.coords().map(|c| (c.x(), c.y())).collect();
    Some(LineString::from(coords))
}

/// Keeps only the first part of a MultiLineString; other geometry types give `None`.
fn extract_linestring(geom: geo::Geometry<f64>) -> Option<LineString<f64>> {
    match geom {
        geo::Geometry::LineString(ls) => Some(ls),
        geo::Geometry::MultiLineString(mls) => mls.0.into_iter().next(),
        _ => None,
    }
}

/// Length in meters, then start and end bearings in degrees. A geometry with
/// fewer than two points gives all zeros.
fn compute_metrics_from_linestring(geom: &LineString<f64>) -> (f64, f64, f64) {
    let coords: Vec<_> = geom.coords().collect();
    if coords.len() < 2 {
        return (0.0, 0.0, 0.0);
    }

    let mut total_length = 0.0;
    for i in 0..coords.len() - 1 {
        let p1 = Point::new(coords[i].x, coords[i].y);
        let p2 = Point::new(coords[i + 1].x, coords[i + 1].y);
        total_length += haversine_distance(p1, p2);
    }

    let start_bearing = Geodesic.bearing(
        Point::new(coords[0].x, coords[0].y),
        Point::new(coords[1].x, coords[1].y),
    );

    let n = coords.len();
    let end_bearing = Geodesic.bearing(
        Point::new(coords[n - 2].x, coords[n - 2].y),
        Point::new(coords[n - 1].x, coords[n - 1].y),
    );

    let normalize = |b: f64| ((b % 360.0) + 360.0) % 360.0;

    (
        total_length,
        normalize(start_bearing),
        normalize(end_bearing),
    )
}

/// Great-circle distance in meters on a spherical earth. Cheaper than
/// `Geodesic`, and its error (under 0.5%) is small next to the decoder's
/// length tolerance.
#[inline]
fn haversine_distance(p1: Point<f64>, p2: Point<f64>) -> f64 {
    const EARTH_RADIUS_M: f64 = 6371000.0;

    let lat1 = p1.y().to_radians();
    let lat2 = p2.y().to_radians();
    let delta_lat = (p2.y() - p1.y()).to_radians();
    let delta_lon = (p2.x() - p1.x()).to_radians();

    let a =
        (delta_lat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (delta_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    EARTH_RADIUS_M * c
}

fn linestring_with_metrics(geom: LineString<f64>) -> GeometryWithMetrics {
    let (length_m, bearing_start, bearing_end) = compute_metrics_from_linestring(&geom);
    GeometryWithMetrics {
        geometry: geom,
        length_m,
        bearing_start,
        bearing_end,
    }
}

/// Read base64 OpenLR codes from the `openlr` column of a parquet file,
/// skipping nulls. Returns an empty list if the column is missing or is not
/// a plain String array.
pub fn load_openlr_codes(path: &Path) -> Result<Vec<String>> {
    let file = File::open(path).context("Failed to open parquet file")?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;
    let reader = builder.build()?;

    let mut codes = Vec::new();

    for batch_result in reader {
        let batch = batch_result?;

        let openlr_col = batch
            .column_by_name("openlr")
            .and_then(|c| c.as_any().downcast_ref::<StringArray>());

        if let Some(col) = openlr_col {
            for i in 0..col.len() {
                if !col.is_null(i) {
                    codes.push(col.value(i).to_string());
                }
            }
        }
    }

    Ok(codes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn test_frc_highway_mapping() {
        assert_eq!(Frc::from_osm_highway("motorway"), Frc::Frc0);
        assert_eq!(Frc::from_osm_highway("tertiary"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("residential"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("service"), Frc::Frc4);
    }

    #[test]
    fn test_parse_wkb_linestring() {
        // LINESTRING(0 0, 1 1, 2 2), little-endian
        let wkb: Vec<u8> = vec![
            0x01, // Little endian
            0x02, 0x00, 0x00, 0x00, // Type: LineString (2)
            0x03, 0x00, 0x00, 0x00, // Number of points: 3
            // Point 1: (0, 0)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // x = 0.0
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // y = 0.0
            // Point 2: (1, 1)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f, // x = 1.0
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f, // y = 1.0
            // Point 3: (2, 2)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, // x = 2.0
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, // y = 2.0
        ];

        let result = parse_wkb_linestring(&wkb);
        assert!(result.is_some());

        let ls = result.unwrap();
        assert_eq!(ls.0.len(), 3);
        assert_eq!(ls.0[0].x, 0.0);
        assert_eq!(ls.0[0].y, 0.0);
        assert_eq!(ls.0[1].x, 1.0);
        assert_eq!(ls.0[1].y, 1.0);
        assert_eq!(ls.0[2].x, 2.0);
        assert_eq!(ls.0[2].y, 2.0);
    }

    #[test]
    fn test_parse_wkb_invalid() {
        let result = parse_wkb_linestring(&[0x00, 0x01, 0x02]);
        assert!(result.is_none());
    }

    #[test]
    fn test_parse_wkt_linestring() {
        let wkt = "LINESTRING(0 0, 1 1, 2 2)";
        let result = parse_wkt_linestring(wkt);
        assert!(result.is_some());

        let ls = result.unwrap();
        assert_eq!(ls.0.len(), 3);
        assert_eq!(ls.0[0].x, 0.0);
        assert_eq!(ls.0[0].y, 0.0);
        assert_eq!(ls.0[1].x, 1.0);
        assert_eq!(ls.0[1].y, 1.0);
        assert_eq!(ls.0[2].x, 2.0);
        assert_eq!(ls.0[2].y, 2.0);
    }

    #[test]
    fn test_parse_wkt_linestring_with_decimals() {
        let wkt = "LINESTRING(-122.4194 37.7749, -122.4089 37.7849)";
        let result = parse_wkt_linestring(wkt);
        assert!(result.is_some());

        let ls = result.unwrap();
        assert_eq!(ls.0.len(), 2);
        assert!((ls.0[0].x - (-122.4194)).abs() < 1e-10);
        assert!((ls.0[0].y - 37.7749).abs() < 1e-10);
    }

    #[test]
    fn test_parse_wkt_invalid() {
        let result = parse_wkt_linestring("not valid wkt");
        assert!(result.is_none());
    }

    #[test]
    fn test_parse_wkt_wrong_geometry_type() {
        let result = parse_wkt_linestring("POINT(0 0)");
        assert!(result.is_none());
    }

    #[test]
    fn test_parse_geoarrow_linestring() {
        use arrow::array::StructArray;
        use arrow::datatypes::{DataType, Field, Fields};
        use geoarrow_schema::{Dimension, LineStringType, Metadata};

        // GeoArrow LineString is List<Struct<x: f64, y: f64>>.
        let x_field = Field::new("x", DataType::Float64, false);
        let y_field = Field::new("y", DataType::Float64, false);
        let coord_fields = Fields::from(vec![x_field.clone(), y_field.clone()]);

        let x_values: Vec<f64> = vec![0.0, 1.0, 2.0];
        let y_values: Vec<f64> = vec![0.0, 1.0, 2.0];

        let x_array =
            Arc::new(arrow::array::Float64Array::from(x_values)) as arrow::array::ArrayRef;
        let y_array =
            Arc::new(arrow::array::Float64Array::from(y_values)) as arrow::array::ArrayRef;

        let coord_struct = StructArray::new(coord_fields, vec![x_array, y_array], None);

        let offsets = arrow::buffer::OffsetBuffer::new(vec![0i32, 3].into());
        let list_array = arrow::array::ListArray::new(
            Arc::new(Field::new_struct(
                "vertices",
                coord_struct.fields().clone(),
                false,
            )),
            offsets,
            Arc::new(coord_struct),
            None,
        );

        let line_type = LineStringType::new(Dimension::XY, Arc::new(Metadata::default()))
            .with_coord_type(geoarrow_schema::CoordType::Separated);
        let geoarrow_arr = LineStringArray::try_from((&list_array, line_type)).unwrap();

        let result = parse_geoarrow_linestring(&geoarrow_arr, 0);
        assert!(result.is_some());

        let ls = result.unwrap();
        assert_eq!(ls.0.len(), 3);
        assert_eq!(ls.0[0].x, 0.0);
        assert_eq!(ls.0[0].y, 0.0);
        assert_eq!(ls.0[1].x, 1.0);
        assert_eq!(ls.0[1].y, 1.0);
        assert_eq!(ls.0[2].x, 2.0);
        assert_eq!(ls.0[2].y, 2.0);
    }

    #[test]
    fn test_detect_geometry_format_wkb() {
        use arrow::array::BinaryArray;
        use arrow::datatypes::{Field, Schema};

        let wkb_data: Vec<Option<&[u8]>> = vec![Some(&[0x01, 0x02])];
        let geom_array = BinaryArray::from(wkb_data);

        let schema = Schema::new(vec![Field::new("geometry", DataType::Binary, true)]);

        let batch = RecordBatch::try_new(Arc::new(schema), vec![Arc::new(geom_array)]).unwrap();

        let format = detect_geometry_format(&batch);
        assert!(matches!(format, GeometryFormat::Wkb(WkbSource::Binary(_))));
    }

    #[test]
    fn test_detect_geometry_format_wkt() {
        use arrow::datatypes::{Field, Schema};

        let wkt_data = vec![Some("LINESTRING(0 0, 1 1)")];
        let geom_array = StringArray::from(wkt_data);

        let schema = Schema::new(vec![Field::new("geometry", DataType::Utf8, true)]);

        let batch = RecordBatch::try_new(Arc::new(schema), vec![Arc::new(geom_array)]).unwrap();

        let format = detect_geometry_format(&batch);
        assert!(matches!(format, GeometryFormat::Wkt(WktSource::String(_))));
    }

    #[test]
    fn test_detect_geometry_format_none() {
        use arrow::datatypes::{Field, Schema};

        let data = vec![Some(1i64), Some(2i64)];
        let array = Int64Array::from(data);

        let schema = Schema::new(vec![Field::new("other_column", DataType::Int64, true)]);

        let batch = RecordBatch::try_new(Arc::new(schema), vec![Arc::new(array)]).unwrap();

        let format = detect_geometry_format(&batch);
        assert!(matches!(format, GeometryFormat::None));
    }

    #[test]
    fn test_wrong_column_type_errors() {
        use arrow::datatypes::{Field, Schema};

        // startOsmNode/endOsmNode as strings instead of Int64.
        let schema = Schema::new(vec![
            Field::new("stableEdgeId", DataType::UInt64, false),
            Field::new("startOsmNode", DataType::Utf8, false),
            Field::new("endOsmNode", DataType::Utf8, false),
            Field::new("startLat", DataType::Float64, false),
            Field::new("startLon", DataType::Float64, false),
            Field::new("endLat", DataType::Float64, false),
            Field::new("endLon", DataType::Float64, false),
            Field::new("highway", DataType::Utf8, false),
        ]);

        let batch = RecordBatch::try_new(
            Arc::new(schema),
            vec![
                Arc::new(UInt64Array::from(vec![1u64])),
                Arc::new(StringArray::from(vec!["100"])),
                Arc::new(StringArray::from(vec!["200"])),
                Arc::new(Float64Array::from(vec![39.0])),
                Arc::new(Float64Array::from(vec![-94.0])),
                Arc::new(Float64Array::from(vec![39.1])),
                Arc::new(Float64Array::from(vec![-94.1])),
                Arc::new(StringArray::from(vec!["secondary"])),
            ],
        )
        .unwrap();

        let mut network = RoadNetwork::new();
        let mut edge_envelopes = Vec::new();
        let mut seen_nodes = AHashMap::new();

        let result = process_batch(&batch, &mut network, &mut edge_envelopes, &mut seen_nodes);
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("startOsmNode"),
            "Error should mention the bad column, got: {err_msg}"
        );
    }
}
