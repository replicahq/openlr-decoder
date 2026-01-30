//! Load road network from parquet files matching the BigQuery export schema

use ahash::AHashMap;
use std::fs::File;
use std::path::Path;

use anyhow::{anyhow, Context, Result};
use arrow::array::{
    Array, ArrayRef, BinaryArray, BinaryViewArray, Float64Array, Int64Array, LargeBinaryArray,
    LargeStringArray, RecordBatch, StringArray, StringViewArray, UInt64Array,
};
use arrow::datatypes::{DataType, Field, Schema};
use geo::{GeodesicBearing, LineString, Point};
use geoarrow_array::array::LineStringArray;
use geoarrow_array::{GeoArrowArray, GeoArrowArrayAccessor};
use geoarrow_schema::GeoArrowType;
use geozero::{wkb, wkt, ToGeo};
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use rayon::prelude::*;

use crate::graph::{Edge, Fow, Frc, Node, RoadNetwork};
use crate::spatial::{EdgeEnvelope, SpatialIndex};

/// Geometry column format detected from Arrow schema
enum GeometryFormat<'a> {
    /// WKB binary (Binary, LargeBinary, BinaryView)
    Wkb(WkbSource<'a>),
    /// WKT string (String, LargeString, StringView)
    Wkt(WktSource<'a>),
    /// GeoArrow native LineString
    GeoArrow(LineStringArray),
    /// No geometry column present
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

/// Geometry with pre-computed metrics to avoid redundant passes over coordinates
struct GeometryWithMetrics {
    geometry: LineString<f64>,
    length_m: f64,
    bearing_start: f64,
    bearing_end: f64,
}

/// Returns the expected Arrow schema for road network parquet files.
///
/// Required columns:
/// - `stableEdgeId` (UInt64): unique edge identifier
/// - `startVertex` (Int64): start node ID
/// - `endVertex` (Int64): end node ID
/// - `startLat`, `startLon`, `endLat`, `endLon` (Float64): endpoint coordinates
/// - `highway` (Utf8): OSM highway tag
///
/// Optional columns:
/// - `lanes` (Int64): number of lanes, used for Form of Way inference
/// - `geometry`: LineString geometry in one of these formats:
///   - WKB: Binary, LargeBinary, or BinaryView
///   - WKT: String, LargeString, or StringView
///   - GeoArrow native: List<Struct<x,y>> with geoarrow.linestring extension
pub fn road_network_schema() -> Schema {
    Schema::new(vec![
        Field::new("stableEdgeId", DataType::UInt64, false),
        Field::new("startVertex", DataType::Int64, false),
        Field::new("endVertex", DataType::Int64, false),
        Field::new("startLat", DataType::Float64, false),
        Field::new("startLon", DataType::Float64, false),
        Field::new("endLat", DataType::Float64, false),
        Field::new("endLon", DataType::Float64, false),
        Field::new("highway", DataType::Utf8, false),
        Field::new("lanes", DataType::Int64, true),
        Field::new("geometry", DataType::Binary, true),
    ])
}

/// Load a road network from a parquet file with the BigQuery export schema
///
/// Expected columns:
/// - stableEdgeId (STRING): unique edge identifier
/// - startVertex (INTEGER): start node ID
/// - endVertex (INTEGER): end node ID
/// - startLat, startLon, endLat, endLon (FLOAT): endpoint coordinates
/// - highway (STRING): OSM highway tag
/// - lanes (INTEGER): number of lanes (optional, for FOW)
/// - distance (INTEGER): edge length in meters
/// - geometry (GEOGRAPHY/WKB): edge geometry
pub fn load_network_from_parquet(path: &Path) -> Result<(RoadNetwork, SpatialIndex)> {
    let file = File::open(path).context("Failed to open parquet file")?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;

    // Read total row count from parquet metadata for pre-allocation
    let edge_count_hint = builder
        .metadata()
        .file_metadata()
        .num_rows()
        .try_into()
        .unwrap_or(0usize);

    let reader = builder.build()?;

    build_network_from_batches_with_hint(reader.map(|r| r.map_err(|e| anyhow!(e))), edge_count_hint)
}

/// Build a road network from an iterator of Arrow RecordBatches.
///
/// This is the core function that processes Arrow data into a RoadNetwork.
/// It can be called from various sources (Parquet files, PyArrow tables, Polars DataFrames).
///
/// Expected columns in each batch:
/// - `stableEdgeId` (UInt64): unique edge identifier
/// - `startVertex` (Int64): start node ID
/// - `endVertex` (Int64): end node ID
/// - `startLat`, `startLon`, `endLat`, `endLon` (Float64): endpoint coordinates
/// - `highway` (Utf8): OSM highway tag
/// - `lanes` (Int64, optional): number of lanes for FOW inference
/// - `geometry` (optional): LineString geometry in WKB, WKT, or GeoArrow native format
pub fn build_network_from_batches<I>(batches: I) -> Result<(RoadNetwork, SpatialIndex)>
where
    I: Iterator<Item = Result<RecordBatch>>,
{
    build_network_from_batches_with_hint(batches, 0)
}

/// Build a road network with pre-allocation hints for better memory efficiency.
///
/// `edge_count_hint` is used to pre-allocate vectors and hash maps.
/// Pass 0 if the count is unknown.
fn build_network_from_batches_with_hint<I>(
    batches: I,
    edge_count_hint: usize,
) -> Result<(RoadNetwork, SpatialIndex)>
where
    I: Iterator<Item = Result<RecordBatch>>,
{
    // Estimate node count as ~edge count (road networks have roughly equal nodes and edges)
    let node_hint = edge_count_hint;

    let mut network = if edge_count_hint > 0 {
        RoadNetwork::new_with_capacity(node_hint, edge_count_hint)
    } else {
        RoadNetwork::new()
    };

    let mut edge_envelopes: Vec<EdgeEnvelope> = Vec::with_capacity(edge_count_hint);

    // Track nodes we've seen to avoid duplicates
    let mut seen_nodes: AHashMap<i64, Point<f64>> = AHashMap::with_capacity(node_hint);

    for batch_result in batches {
        let batch = batch_result?;
        process_batch(&batch, &mut network, &mut edge_envelopes, &mut seen_nodes)?;
    }

    // Explicitly drop seen_nodes before R-tree bulk_load to reduce peak memory
    drop(seen_nodes);

    let spatial_index = SpatialIndex::new(edge_envelopes);

    // Drop the ID-to-index lookup maps; they are only needed during construction.
    network.compact();

    Ok((network, spatial_index))
}

/// Represents a fully-processed edge ready to insert into the graph.
/// All expensive computations (geometry parsing, FRC/FOW inference, Edge::new) are done.
struct PendingEdge {
    sv_id: i64,
    ev_id: i64,
    edge: Edge,
}

/// Process a single RecordBatch and add its edges to the network.
fn process_batch(
    batch: &RecordBatch,
    network: &mut RoadNetwork,
    edge_envelopes: &mut Vec<EdgeEnvelope>,
    seen_nodes: &mut AHashMap<i64, Point<f64>>,
) -> Result<()> {
    // Extract columns
    let stable_edge_id = batch
        .column_by_name("stableEdgeId")
        .and_then(|c| c.as_any().downcast_ref::<UInt64Array>());

    let start_vertex = batch
        .column_by_name("startVertex")
        .and_then(|c| c.as_any().downcast_ref::<Int64Array>());

    let end_vertex = batch
        .column_by_name("endVertex")
        .and_then(|c| c.as_any().downcast_ref::<Int64Array>());

    let start_lat = batch
        .column_by_name("startLat")
        .and_then(|c| c.as_any().downcast_ref::<Float64Array>());

    let start_lon = batch
        .column_by_name("startLon")
        .and_then(|c| c.as_any().downcast_ref::<Float64Array>());

    let end_lat = batch
        .column_by_name("endLat")
        .and_then(|c| c.as_any().downcast_ref::<Float64Array>());

    let end_lon = batch
        .column_by_name("endLon")
        .and_then(|c| c.as_any().downcast_ref::<Float64Array>());

    // Highway can be String, LargeString, or StringView (Polars PyCapsule uses view types)
    let highway_col = batch.column_by_name("highway");
    let highway_string = highway_col.and_then(|c| c.as_any().downcast_ref::<StringArray>());
    let highway_large = highway_col.and_then(|c| c.as_any().downcast_ref::<LargeStringArray>());
    let highway_view = highway_col.and_then(|c| c.as_any().downcast_ref::<StringViewArray>());

    let lanes = batch
        .column_by_name("lanes")
        .and_then(|c| c.as_any().downcast_ref::<Int64Array>());

    // Detect geometry format: WKB (binary), WKT (string), or GeoArrow native
    let geometry_format = detect_geometry_format(batch);

    // All required columns must be present (highway can be String or LargeString)
    let (stable_edge_id, start_vertex, end_vertex, start_lat, start_lon, end_lat, end_lon) = match (
        stable_edge_id,
        start_vertex,
        end_vertex,
        start_lat,
        start_lon,
        end_lat,
        end_lon,
    ) {
        (Some(eid), Some(sv), Some(ev), Some(slat), Some(slon), Some(elat), Some(elon)) => {
            (eid, sv, ev, slat, slon, elat, elon)
        }
        _ => return Ok(()), // Skip batch if missing required columns
    };

    // Highway must be present (String, LargeString, or StringView)
    if highway_string.is_none() && highway_large.is_none() && highway_view.is_none() {
        return Ok(());
    }

    // Pre-scan batch to collect unique node IDs and coordinates (deduplication)
    // This reduces HashMap operations from 2*num_edges to ~unique_nodes
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

    // Add new nodes to network (only those not seen before)
    for (node_id, coord) in batch_nodes {
        if let std::collections::hash_map::Entry::Vacant(e) = seen_nodes.entry(node_id) {
            e.insert(coord);
            network.add_node(Node { id: node_id, coord });
        }
    }

    // PARALLEL PHASE: Process all rows in parallel (geometry parsing, FRC/FOW, metrics, Edge construction)
    let pending_edges: Vec<Option<PendingEdge>> = (0..batch.num_rows())
        .into_par_iter()
        .map(|row| {
            // Skip nulls
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

            // Parse highway tag for FRC/FOW (handle String, LargeString, and StringView)
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

            // Parse geometry based on detected format
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

            // Compute metrics during geometry processing (fused pass)
            let geom_with_metrics = linestring_with_metrics(geom);

            // Create edge with pre-computed metrics (no redundant computation)
            let edge = Edge::from_precomputed(
                edge_id,
                geom_with_metrics.geometry,
                frc,
                fow,
                geom_with_metrics.length_m,
                geom_with_metrics.bearing_start,
                geom_with_metrics.bearing_end,
            );

            Some(PendingEdge { sv_id, ev_id, edge })
        })
        .collect();

    // SEQUENTIAL PHASE: Insert edges into graph and build envelopes
    for pending in pending_edges.into_iter().flatten() {
        // Add edge to graph
        let edge_idx = network.add_edge(pending.sv_id, pending.ev_id, pending.edge);

        // Add to spatial index (borrow geometry from graph)
        let edge_geom = &network.edge(edge_idx).unwrap().geometry;
        edge_envelopes.push(EdgeEnvelope::new(edge_idx, edge_geom));
    }

    Ok(())
}

/// Detect the geometry format from the batch schema and data.
fn detect_geometry_format<'a>(batch: &'a RecordBatch) -> GeometryFormat<'a> {
    let schema = batch.schema();
    let Some((idx, field)) = schema.column_with_name("geometry") else {
        return GeometryFormat::None;
    };
    let col = batch.column(idx);

    // Check for GeoArrow extension type metadata
    if let Some(ext_name) = field.metadata().get("ARROW:extension:name") {
        if ext_name.starts_with("geoarrow.linestring") {
            // Try to parse as GeoArrow LineString
            if let Some(arr) = try_parse_geoarrow_linestring(col.clone(), field) {
                return GeometryFormat::GeoArrow(arr);
            }
        }
    }

    // Check for binary types (WKB)
    if let Some(arr) = col.as_any().downcast_ref::<BinaryArray>() {
        return GeometryFormat::Wkb(WkbSource::Binary(arr));
    }
    if let Some(arr) = col.as_any().downcast_ref::<LargeBinaryArray>() {
        return GeometryFormat::Wkb(WkbSource::LargeBinary(arr));
    }
    if let Some(arr) = col.as_any().downcast_ref::<BinaryViewArray>() {
        return GeometryFormat::Wkb(WkbSource::BinaryView(arr));
    }

    // Check for string types (WKT)
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

/// Try to parse an Arrow array as a GeoArrow LineString array.
fn try_parse_geoarrow_linestring(col: ArrayRef, field: &Field) -> Option<LineStringArray> {
    // Parse the GeoArrow type from field metadata
    let geo_type = GeoArrowType::try_from(field).ok()?;
    let line_type = match geo_type {
        GeoArrowType::LineString(t) => t,
        _ => return None,
    };

    // Try to convert the array
    LineStringArray::try_from((col.as_ref(), line_type)).ok()
}

/// Parse WKB bytes into a LineString
fn parse_wkb_linestring(wkb_bytes: &[u8]) -> Option<LineString<f64>> {
    let geom = wkb::Wkb(wkb_bytes.to_vec()).to_geo().ok()?;
    extract_linestring(geom)
}

/// Parse WKT string into a LineString
fn parse_wkt_linestring(wkt_str: &str) -> Option<LineString<f64>> {
    let geom = wkt::Wkt(wkt_str).to_geo().ok()?;
    extract_linestring(geom)
}

/// Parse a GeoArrow LineString at a specific row index
fn parse_geoarrow_linestring(arr: &LineStringArray, row: usize) -> Option<LineString<f64>> {
    use geo_traits::{CoordTrait, LineStringTrait};

    let ls = arr.value(row).ok()?;
    let coords: Vec<_> = ls.coords().map(|c| (c.x(), c.y())).collect();
    Some(LineString::from(coords))
}

/// Extract a LineString from a geo::Geometry
fn extract_linestring(geom: geo::Geometry<f64>) -> Option<LineString<f64>> {
    match geom {
        geo::Geometry::LineString(ls) => Some(ls),
        geo::Geometry::MultiLineString(mls) => mls.0.into_iter().next(),
        _ => None,
    }
}

/// Compute edge metrics (length, bearings) from a LineString.
/// This is optimized to walk coordinates once using Haversine approximation.
fn compute_metrics_from_linestring(geom: &LineString<f64>) -> (f64, f64, f64) {
    let coords: Vec<_> = geom.coords().collect();
    if coords.len() < 2 {
        return (0.0, 0.0, 0.0);
    }

    // Compute geodesic length using Haversine formula
    let mut total_length = 0.0;
    for i in 0..coords.len() - 1 {
        let p1 = Point::new(coords[i].x, coords[i].y);
        let p2 = Point::new(coords[i + 1].x, coords[i + 1].y);
        total_length += haversine_distance(p1, p2);
    }

    // Start bearing: from first point toward second
    let start_bearing =
        Point::new(coords[0].x, coords[0].y).geodesic_bearing(Point::new(coords[1].x, coords[1].y));

    // End bearing: from second-to-last toward last
    let n = coords.len();
    let end_bearing = Point::new(coords[n - 2].x, coords[n - 2].y)
        .geodesic_bearing(Point::new(coords[n - 1].x, coords[n - 1].y));

    // Normalize to 0-360
    let normalize = |b: f64| ((b % 360.0) + 360.0) % 360.0;

    (
        total_length,
        normalize(start_bearing),
        normalize(end_bearing),
    )
}

/// Fast Haversine distance approximation in meters.
/// More efficient than full geodesic calculation for our use case.
#[inline]
fn haversine_distance(p1: Point<f64>, p2: Point<f64>) -> f64 {
    const EARTH_RADIUS_M: f64 = 6371000.0; // Earth's radius in meters

    let lat1 = p1.y().to_radians();
    let lat2 = p2.y().to_radians();
    let delta_lat = (p2.y() - p1.y()).to_radians();
    let delta_lon = (p2.x() - p1.x()).to_radians();

    let a =
        (delta_lat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (delta_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    EARTH_RADIUS_M * c
}

/// Wrap a LineString with pre-computed metrics.
fn linestring_with_metrics(geom: LineString<f64>) -> GeometryWithMetrics {
    let (length_m, bearing_start, bearing_end) = compute_metrics_from_linestring(&geom);
    GeometryWithMetrics {
        geometry: geom,
        length_m,
        bearing_start,
        bearing_end,
    }
}

/// Load OpenLR test codes from a parquet file
///
/// Expected columns:
/// - openlr (STRING): base64-encoded OpenLR string
/// - id (STRING, optional): identifier for the code
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
        assert_eq!(Frc::from_osm_highway("residential"), Frc::Frc4);
    }

    #[test]
    fn test_parse_wkb_linestring() {
        // WKB for LINESTRING(0 0, 1 1, 2 2) - little endian
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
        // Invalid WKB bytes
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
        // Point geometry should return None (we expect LineString)
        let result = parse_wkt_linestring("POINT(0 0)");
        assert!(result.is_none());
    }

    #[test]
    fn test_parse_geoarrow_linestring() {
        use arrow::array::StructArray;
        use arrow::datatypes::{DataType, Field, Fields};
        use geoarrow_schema::{Dimension, LineStringType, Metadata};

        // Build a GeoArrow LineString array manually
        // GeoArrow LineString is List<Struct<x: f64, y: f64>>
        let x_field = Field::new("x", DataType::Float64, false);
        let y_field = Field::new("y", DataType::Float64, false);
        let coord_fields = Fields::from(vec![x_field.clone(), y_field.clone()]);

        // Create coordinate arrays for one linestring with 3 points: (0,0), (1,1), (2,2)
        let x_values: Vec<f64> = vec![0.0, 1.0, 2.0];
        let y_values: Vec<f64> = vec![0.0, 1.0, 2.0];

        let x_array =
            Arc::new(arrow::array::Float64Array::from(x_values)) as arrow::array::ArrayRef;
        let y_array =
            Arc::new(arrow::array::Float64Array::from(y_values)) as arrow::array::ArrayRef;

        let coord_struct = StructArray::new(coord_fields, vec![x_array, y_array], None);

        // Create list array with one linestring containing all 3 coords
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

        // Convert to GeoArrow LineStringArray
        let line_type = LineStringType::new(Dimension::XY, Arc::new(Metadata::default()))
            .with_coord_type(geoarrow_schema::CoordType::Separated);
        let geoarrow_arr = LineStringArray::try_from((&list_array, line_type)).unwrap();

        // Test parsing
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

        // Create a batch with a binary geometry column
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

        // Create a batch with a string geometry column
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

        // Create a batch without a geometry column
        let data = vec![Some(1i64), Some(2i64)];
        let array = Int64Array::from(data);

        let schema = Schema::new(vec![Field::new("other_column", DataType::Int64, true)]);

        let batch = RecordBatch::try_new(Arc::new(schema), vec![Arc::new(array)]).unwrap();

        let format = detect_geometry_format(&batch);
        assert!(matches!(format, GeometryFormat::None));
    }
}
