//! Load road network from parquet files matching the BigQuery export schema

use std::collections::HashMap;
use std::fs::File;
use std::path::Path;

use anyhow::{Context, Result};
use arrow::array::{Array, BinaryArray, Float64Array, Int64Array, StringArray, UInt64Array};
use geo::{LineString, Point};
use geozero::wkb;
use geozero::ToGeo;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

use crate::graph::{Edge, Fow, Frc, Node, RoadNetwork};
use crate::spatial::{EdgeEnvelope, SpatialIndex};

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
    let reader = builder.build()?;

    let mut network = RoadNetwork::new();
    let mut edge_envelopes: Vec<EdgeEnvelope> = Vec::new();

    // Track nodes we've seen to avoid duplicates
    let mut seen_nodes: HashMap<i64, Point<f64>> = HashMap::new();

    for batch_result in reader {
        let batch = batch_result?;

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

        let highway = batch
            .column_by_name("highway")
            .and_then(|c| c.as_any().downcast_ref::<StringArray>());

        let lanes = batch
            .column_by_name("lanes")
            .and_then(|c| c.as_any().downcast_ref::<Int64Array>());

        let geometry = batch
            .column_by_name("geometry")
            .and_then(|c| c.as_any().downcast_ref::<BinaryArray>());

        // All required columns must be present
        let (
            stable_edge_id,
            start_vertex,
            end_vertex,
            start_lat,
            start_lon,
            end_lat,
            end_lon,
            highway,
        ) = match (
            stable_edge_id,
            start_vertex,
            end_vertex,
            start_lat,
            start_lon,
            end_lat,
            end_lon,
            highway,
        ) {
            (
                Some(eid),
                Some(sv),
                Some(ev),
                Some(slat),
                Some(slon),
                Some(elat),
                Some(elon),
                Some(hw),
            ) => (eid, sv, ev, slat, slon, elat, elon, hw),
            _ => continue, // Skip batch if missing required columns
        };

        for row in 0..batch.num_rows() {
            // Skip nulls
            if stable_edge_id.is_null(row) || start_vertex.is_null(row) || end_vertex.is_null(row) {
                continue;
            }

            let edge_id = stable_edge_id.value(row);
            let sv_id = start_vertex.value(row);
            let ev_id = end_vertex.value(row);
            let sv_lat = start_lat.value(row);
            let sv_lon = start_lon.value(row);
            let ev_lat = end_lat.value(row);
            let ev_lon = end_lon.value(row);

            // Create nodes if not seen
            if !seen_nodes.contains_key(&sv_id) {
                let coord = Point::new(sv_lon, sv_lat);
                seen_nodes.insert(sv_id, coord);
                network.add_node(Node { id: sv_id, coord });
            }
            if !seen_nodes.contains_key(&ev_id) {
                let coord = Point::new(ev_lon, ev_lat);
                seen_nodes.insert(ev_id, coord);
                network.add_node(Node { id: ev_id, coord });
            }

            // Parse highway tag for FRC/FOW
            let hw_tag = if highway.is_null(row) {
                ""
            } else {
                highway.value(row)
            };
            let frc = Frc::from_osm_highway(hw_tag);

            let lane_count = lanes
                .map(|l| {
                    if l.is_null(row) {
                        None
                    } else {
                        Some(l.value(row) as u8)
                    }
                })
                .flatten();
            let fow = Fow::from_osm_tags(hw_tag, None, None, lane_count);

            // Parse geometry
            let geom: LineString<f64> = if let Some(geom_col) = geometry {
                if !geom_col.is_null(row) {
                    let wkb_bytes = geom_col.value(row);
                    parse_wkb_linestring(wkb_bytes).unwrap_or_else(|| {
                        // Fallback to simple start->end line
                        LineString::from(vec![(sv_lon, sv_lat), (ev_lon, ev_lat)])
                    })
                } else {
                    LineString::from(vec![(sv_lon, sv_lat), (ev_lon, ev_lat)])
                }
            } else {
                LineString::from(vec![(sv_lon, sv_lat), (ev_lon, ev_lat)])
            };

            // Create edge
            let edge = Edge::new(edge_id, geom.clone(), frc, fow);
            let edge_idx = network.add_edge(sv_id, ev_id, edge);

            // Add to spatial index
            edge_envelopes.push(EdgeEnvelope::new(edge_idx, geom));
        }
    }

    let spatial_index = SpatialIndex::new(edge_envelopes);

    Ok((network, spatial_index))
}

/// Parse WKB bytes into a LineString
fn parse_wkb_linestring(wkb_bytes: &[u8]) -> Option<LineString<f64>> {
    // Use geozero to parse WKB
    let geom = wkb::Wkb(wkb_bytes.to_vec()).to_geo().ok()?;

    match geom {
        geo::Geometry::LineString(ls) => Some(ls),
        geo::Geometry::MultiLineString(mls) => {
            // Take the first linestring if it's a multi
            mls.0.into_iter().next()
        }
        _ => None,
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

    #[test]
    fn test_frc_highway_mapping() {
        assert_eq!(Frc::from_osm_highway("motorway"), Frc::Frc0);
        assert_eq!(Frc::from_osm_highway("residential"), Frc::Frc5);
    }
}
