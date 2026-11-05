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
use geo::{Haversine, Length, LineString, Point};
use geoarrow_array::array::LineStringArray;
use geoarrow_array::{GeoArrowArray, GeoArrowArrayAccessor};
use geoarrow_schema::GeoArrowType;
use geozero::{wkb, wkt, ToGeo};
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use rayon::prelude::*;

use crate::graph::{Edge, Fow, Frc, RoadNetwork};
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

/// The Arrow schema the loader reads. Extra columns are ignored.
///
/// - `stableEdgeId` (UInt64), `startOsmNode`/`endOsmNode` (Int64): node ids are
///   OSM nodes, so edges split at barriers still connect.
/// - `startLat`, `startLon`, `endLat`, `endLon` (Float64), degrees.
///
/// Rows with a null in any of those seven columns are skipped.
/// - `highway`: OSM tag, mapped by [`Frc::from_osm_highway`] and
///   [`Fow::from_osm_tags`]. Any string type (String, LargeString, StringView).
/// - `lanes` (optional): only used for FOW. Any type Arrow can cast to Int64;
///   values that don't cast, or fall outside 0–255, count as unknown.
/// - `geometry` (optional): WKB as any binary type, WKT as any string type, or
///   GeoArrow `geoarrow.linestring`. A MultiLineString's parts are joined head
///   to tail. Missing or unparseable geometry, or a MultiLineString whose
///   parts don't connect, falls back to the straight line between the
///   endpoints.
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
        }
    }

    pub fn push_batch(&mut self, batch: &RecordBatch) -> Result<()> {
        process_batch(batch, &mut self.network, &mut self.edge_envelopes)
    }

    pub fn finish(mut self) -> (RoadNetwork, SpatialIndex) {
        // Drop the id maps before the R-tree bulk load to lower peak memory.
        self.network.compact();
        let spatial_index = SpatialIndex::new(self.edge_envelopes);

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

    // Cast so Int32 or string lane columns work too; values that don't parse
    // become null.
    let lanes_i64 = batch
        .column_by_name("lanes")
        .and_then(|c| arrow::compute::cast(c, &DataType::Int64).ok());
    let lanes = lanes_i64
        .as_ref()
        .and_then(|c| c.as_any().downcast_ref::<Int64Array>());

    let skip_row = |row: usize| {
        stable_edge_id.is_null(row)
            || start_vertex.is_null(row)
            || end_vertex.is_null(row)
            || start_lat.is_null(row)
            || start_lon.is_null(row)
            || end_lat.is_null(row)
            || end_lon.is_null(row)
    };

    let geometry_format = detect_geometry_format(batch);

    // Dedupe the batch's nodes locally first so the network's id map is probed
    // once per unique node rather than twice per edge. The first coordinate
    // seen for a node id wins.
    let mut batch_nodes: AHashMap<i64, Point<f64>> = AHashMap::new();
    for row in 0..batch.num_rows() {
        if skip_row(row) {
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
        network.get_or_add_node(node_id, coord);
    }

    // Build edges in parallel; only graph insertion has to be sequential.
    let pending_edges: Vec<Option<PendingEdge>> = (0..batch.num_rows())
        .into_par_iter()
        .map(|row| {
            if skip_row(row) {
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

            let lane_count = lanes
                .filter(|l| !l.is_null(row))
                .and_then(|l| u8::try_from(l.value(row)).ok());
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

            let edge = Edge {
                id: edge_id,
                length_m: Haversine.length(&geom),
                geometry: geom,
                frc,
                fow,
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

/// Joins a MultiLineString's parts head to tail, in storage order, reversing
/// a part when that makes it connect. `None` if the parts don't chain that
/// way, or for any other geometry type, so the caller falls back to a straight
/// line rather than a geometry that jumps or doubles back.
fn extract_linestring(geom: geo::Geometry<f64>) -> Option<LineString<f64>> {
    match geom {
        geo::Geometry::LineString(ls) => Some(ls),
        geo::Geometry::MultiLineString(mls) => {
            let mut parts = mls.into_iter().map(|p| p.0).filter(|p| !p.is_empty());
            let mut coords = parts.next()?;
            let mut first_join = true;
            for mut part in parts {
                let end = *coords.last().unwrap();
                let (head, tail) = (part[0], *part.last().unwrap());
                if head == end {
                } else if tail == end {
                    part.reverse();
                } else if first_join && coords[0] == head {
                    coords.reverse();
                } else if first_join && coords[0] == tail {
                    coords.reverse();
                    part.reverse();
                } else {
                    return None;
                }
                coords.extend(part.into_iter().skip(1));
                first_join = false;
            }
            Some(LineString::new(coords))
        }
        _ => None,
    }
}

/// Read base64 OpenLR codes from the `openlr` column (any string type) of a
/// parquet file, skipping nulls. Dictionary-encoded strings work too. Errors if
/// the column is missing or not a string.
pub fn load_openlr_codes(path: &Path) -> Result<Vec<String>> {
    let file = File::open(path).context("Failed to open parquet file")?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;

    // Check the schema, not each batch, so a file with no rows errors too.
    let field = builder
        .schema()
        .field_with_name("openlr")
        .map_err(|_| anyhow!("Missing column 'openlr'"))?;
    let is_string =
        |t: &DataType| matches!(t, DataType::Utf8 | DataType::LargeUtf8 | DataType::Utf8View);
    let ok = match field.data_type() {
        DataType::Dictionary(_, values) => is_string(values),
        t => is_string(t),
    };
    if !ok {
        return Err(anyhow!(
            "Column 'openlr' has type {:?}, expected a string type",
            field.data_type()
        ));
    }

    let mut codes = Vec::new();
    for batch_result in builder.build()? {
        let batch = batch_result?;
        let col = batch.column_by_name("openlr").unwrap();
        let col = arrow::compute::cast(col, &DataType::Utf8)?;
        let col = col.as_any().downcast_ref::<StringArray>().unwrap();
        codes.extend(col.iter().flatten().map(str::to_string));
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

        let result = process_batch(&batch, &mut network, &mut edge_envelopes);
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("startOsmNode"),
            "Error should mention the bad column, got: {err_msg}"
        );
    }

    /// A two-row batch of `primary` edges, with `lanes` appended when given.
    fn edge_batch(start_lat: Vec<Option<f64>>, lanes: Option<ArrayRef>) -> RecordBatch {
        use arrow::datatypes::{Field, Schema};

        let mut fields = vec![
            Field::new("stableEdgeId", DataType::UInt64, false),
            Field::new("startOsmNode", DataType::Int64, false),
            Field::new("endOsmNode", DataType::Int64, false),
            Field::new("startLat", DataType::Float64, true),
            Field::new("startLon", DataType::Float64, false),
            Field::new("endLat", DataType::Float64, false),
            Field::new("endLon", DataType::Float64, false),
            Field::new("highway", DataType::Utf8, false),
        ];
        let mut columns: Vec<ArrayRef> = vec![
            Arc::new(UInt64Array::from(vec![1u64, 2])),
            Arc::new(Int64Array::from(vec![10i64, 20])),
            Arc::new(Int64Array::from(vec![11i64, 21])),
            Arc::new(Float64Array::from(start_lat)),
            Arc::new(Float64Array::from(vec![-94.0, -94.1])),
            Arc::new(Float64Array::from(vec![39.001, 39.101])),
            Arc::new(Float64Array::from(vec![-94.0, -94.1])),
            Arc::new(StringArray::from(vec!["primary", "primary"])),
        ];
        if let Some(lanes) = lanes {
            fields.push(Field::new("lanes", lanes.data_type().clone(), true));
            columns.push(lanes);
        }
        RecordBatch::try_new(Arc::new(Schema::new(fields)), columns).unwrap()
    }

    fn load(batch: &RecordBatch) -> RoadNetwork {
        let mut network = RoadNetwork::new();
        process_batch(batch, &mut network, &mut Vec::new()).unwrap();
        network
    }

    #[test]
    fn test_null_coordinate_row_skipped() {
        let network = load(&edge_batch(vec![Some(39.0), None], None));
        assert_eq!(network.graph.edge_count(), 1);
        assert_eq!(network.graph.node_count(), 2);
    }

    #[test]
    fn test_lanes_any_integer_type_and_out_of_range() {
        use arrow::array::Int32Array;

        // 4 lanes on a primary road means a dual carriageway; -1 must not wrap
        // to 255 and do the same.
        let lanes: ArrayRef = Arc::new(Int32Array::from(vec![4, -1]));
        let network = load(&edge_batch(vec![Some(39.0), Some(39.1)], Some(lanes)));
        let fow = |id| {
            let idx = network.edge_id_to_index.as_ref().unwrap()[&id];
            network.edge(idx).unwrap().fow
        };
        assert_eq!(fow(1), Fow::MultipleCarriageway);
        assert_eq!(fow(2), Fow::SingleCarriageway);
    }

    #[test]
    fn test_parse_wkt_multilinestring_joins_parts() {
        let xs = |wkt| {
            parse_wkt_linestring(wkt).map(|ls| ls.coords().map(|c| c.x).collect::<Vec<f64>>())
        };
        let joined = Some(vec![0.0, 1.0, 2.0, 3.0]);
        assert_eq!(
            xs("MULTILINESTRING((0 0, 1 1), (1 1, 2 2), (2 2, 3 3))"),
            joined
        );
        // Reversed parts, including the first.
        assert_eq!(
            xs("MULTILINESTRING((0 0, 1 1), (2 2, 1 1), (2 2, 3 3))"),
            joined
        );
        assert_eq!(
            xs("MULTILINESTRING((1 1, 0 0), (1 1, 2 2), (3 3, 2 2))"),
            joined
        );
        // A gap or out-of-order parts: no geometry, so the loader uses a straight line.
        assert_eq!(xs("MULTILINESTRING((0 0, 1 1), (2 2, 3 3))"), None);
        assert_eq!(
            xs("MULTILINESTRING((0 0, 1 1), (2 2, 3 3), (1 1, 2 2))"),
            None
        );
    }

    #[test]
    fn test_load_openlr_codes_string_types() {
        use arrow::array::StringViewArray;
        use arrow::datatypes::{Field, Schema};
        use parquet::arrow::ArrowWriter;

        let path =
            std::env::temp_dir().join(format!("openlr_codes_{}.parquet", std::process::id()));
        let write = |col: ArrayRef| {
            let schema = Schema::new(vec![Field::new("openlr", col.data_type().clone(), true)]);
            let batch = RecordBatch::try_new(Arc::new(schema), vec![col]).unwrap();
            let mut writer =
                ArrowWriter::try_new(File::create(&path).unwrap(), batch.schema(), None).unwrap();
            writer.write(&batch).unwrap();
            writer.close().unwrap();
        };

        let dict: arrow::array::DictionaryArray<arrow::datatypes::Int32Type> =
            vec![Some("a"), None, Some("b")].into_iter().collect();
        for col in [
            Arc::new(StringArray::from(vec![Some("a"), None, Some("b")])) as ArrayRef,
            Arc::new(LargeStringArray::from(vec![Some("a"), None, Some("b")])),
            Arc::new(StringViewArray::from(vec![Some("a"), None, Some("b")])),
            Arc::new(dict),
        ] {
            let ty = col.data_type().clone();
            write(col);
            assert_eq!(load_openlr_codes(&path).unwrap(), vec!["a", "b"], "{ty:?}");
        }

        write(Arc::new(Int64Array::from(vec![1i64])));
        assert!(load_openlr_codes(&path).is_err());
        // No rows and no openlr column: still an error.
        let empty = RecordBatch::try_new(
            Arc::new(Schema::new(vec![Field::new(
                "other",
                DataType::Int64,
                true,
            )])),
            vec![Arc::new(Int64Array::from(Vec::<i64>::new()))],
        )
        .unwrap();
        let mut writer =
            ArrowWriter::try_new(File::create(&path).unwrap(), empty.schema(), None).unwrap();
        writer.write(&empty).unwrap();
        writer.close().unwrap();
        assert!(load_openlr_codes(&path).is_err());
        std::fs::remove_file(&path).ok();
    }
}
