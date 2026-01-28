//! Test utilities for building road networks programmatically.
//!
//! This module provides a builder pattern for constructing test road networks
//! without needing parquet files. It's designed to match the patterns used in
//! the Java OpenLR reference implementation's test suite.
//!
//! # Example
//!
//! ```rust
//! use openlr_decoder::test_utils::TestNetworkBuilder;
//! use openlr_decoder::{Frc, Fow};
//!
//! let (network, spatial) = TestNetworkBuilder::new()
//!     .add_node(1, 52.622875, 13.49214)
//!     .add_node(2, 52.614812, 13.546033)
//!     .add_edge(1, 1, 2, 3695.0, Frc::Frc0, Fow::Motorway)
//!     .build();
//!
//! assert_eq!(network.node_count(), 2);
//! assert_eq!(network.edge_count(), 1);
//! ```

use crate::graph::{Edge, Fow, Frc, Node, RoadNetwork};
use crate::spatial::{EdgeEnvelope, SpatialIndex};
use geo::{LineString, Point};
use std::collections::HashMap;

/// A builder for constructing test road networks programmatically.
///
/// This builder allows you to add nodes and edges with explicit IDs,
/// and automatically generates the necessary geometry and spatial index.
pub struct TestNetworkBuilder {
    nodes: HashMap<i64, (f64, f64)>, // id -> (lat, lon)
    edges: Vec<PendingEdge>,
}

struct PendingEdge {
    id: u64,
    start_node: i64,
    end_node: i64,
    length_m: Option<f64>, // If None, calculated from geometry
    frc: Frc,
    fow: Fow,
}

impl TestNetworkBuilder {
    /// Create a new empty test network builder.
    pub fn new() -> Self {
        TestNetworkBuilder {
            nodes: HashMap::new(),
            edges: Vec::new(),
        }
    }

    /// Add a node to the network.
    ///
    /// # Arguments
    /// * `id` - Unique node identifier
    /// * `lat` - Latitude in degrees
    /// * `lon` - Longitude in degrees
    ///
    /// # Panics
    /// Panics if a node with the same ID already exists.
    pub fn add_node(mut self, id: i64, lat: f64, lon: f64) -> Self {
        if self.nodes.contains_key(&id) {
            panic!("Node with id {} already exists", id);
        }
        self.nodes.insert(id, (lat, lon));
        self
    }

    /// Add an edge to the network with an explicit length.
    ///
    /// The geometry will be a straight LineString from the start node to the end node.
    /// The bearing is calculated automatically from the geometry.
    ///
    /// # Arguments
    /// * `id` - Unique edge identifier
    /// * `start_node` - ID of the start node (must exist)
    /// * `end_node` - ID of the end node (must exist)
    /// * `length_m` - Length in meters (used for path finding, not derived from geometry)
    /// * `frc` - Functional Road Class
    /// * `fow` - Form of Way
    pub fn add_edge(
        mut self,
        id: u64,
        start_node: i64,
        end_node: i64,
        length_m: f64,
        frc: Frc,
        fow: Fow,
    ) -> Self {
        self.edges.push(PendingEdge {
            id,
            start_node,
            end_node,
            length_m: Some(length_m),
            frc,
            fow,
        });
        self
    }

    /// Add an edge to the network with length calculated from geometry.
    ///
    /// The geometry will be a straight LineString from the start node to the end node.
    /// The length is calculated using geodesic distance.
    ///
    /// # Arguments
    /// * `id` - Unique edge identifier
    /// * `start_node` - ID of the start node (must exist)
    /// * `end_node` - ID of the end node (must exist)
    /// * `frc` - Functional Road Class
    /// * `fow` - Form of Way
    pub fn add_edge_auto_length(
        mut self,
        id: u64,
        start_node: i64,
        end_node: i64,
        frc: Frc,
        fow: Fow,
    ) -> Self {
        self.edges.push(PendingEdge {
            id,
            start_node,
            end_node,
            length_m: None, // Will be calculated from geometry
            frc,
            fow,
        });
        self
    }

    /// Build the road network and spatial index.
    ///
    /// # Panics
    /// Panics if any edge references a non-existent node.
    ///
    /// # Returns
    /// A tuple of (RoadNetwork, SpatialIndex) ready for use with the decoder.
    pub fn build(self) -> (RoadNetwork, SpatialIndex) {
        let mut network = RoadNetwork::new();

        // Add all nodes first
        for (&id, &(lat, lon)) in &self.nodes {
            let coord = Point::new(lon, lat); // geo uses (x, y) = (lon, lat)
            network.add_node(Node { id, coord });
        }

        // Collect edge envelopes for spatial index
        let mut envelopes = Vec::new();

        // Add all edges
        for pending in self.edges {
            let start_coord = self.nodes.get(&pending.start_node).unwrap_or_else(|| {
                panic!(
                    "Start node {} does not exist for edge {}",
                    pending.start_node, pending.id
                )
            });
            let end_coord = self.nodes.get(&pending.end_node).unwrap_or_else(|| {
                panic!(
                    "End node {} does not exist for edge {}",
                    pending.end_node, pending.id
                )
            });

            // Create LineString geometry (lon, lat order for geo)
            let geometry = LineString::from(vec![
                (start_coord.1, start_coord.0), // (lon, lat)
                (end_coord.1, end_coord.0),
            ]);

            // Create edge - Edge::new will compute bearings (move geometry, no clone)
            let mut edge = Edge::new(pending.id, geometry, pending.frc, pending.fow);

            // Override length if explicitly provided
            if let Some(length) = pending.length_m {
                edge.length_m = length;
            }

            let edge_idx = network.add_edge(pending.start_node, pending.end_node, edge);

            // Create envelope for spatial index (borrow geometry from graph)
            let edge_geom = &network.edge(edge_idx).unwrap().geometry;
            envelopes.push(EdgeEnvelope::new(edge_idx, edge_geom));
        }

        let spatial = SpatialIndex::new(envelopes);

        (network, spatial)
    }
}

impl Default for TestNetworkBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_network_building() {
        let (network, spatial) = TestNetworkBuilder::new()
            .add_node(1, 52.622875, 13.49214)
            .add_node(2, 52.614812, 13.546033)
            .add_edge(1, 1, 2, 3695.0, Frc::Frc0, Fow::Motorway)
            .build();

        assert_eq!(network.node_count(), 2);
        assert_eq!(network.edge_count(), 1);
        assert_eq!(spatial.len(), 1);
    }

    #[test]
    fn test_edge_properties() {
        let (network, _spatial) = TestNetworkBuilder::new()
            .add_node(1, 52.0, 13.0)
            .add_node(2, 52.0, 14.0) // Same lat, 1 degree east
            .add_edge(100, 1, 2, 5000.0, Frc::Frc2, Fow::SingleCarriageway)
            .build();

        let edge_idx = network
            .edge_id_to_index
            .as_ref()
            .unwrap()
            .get(&100)
            .unwrap();
        let edge = network.edge(*edge_idx).unwrap();

        assert_eq!(edge.id, 100);
        assert_eq!(edge.frc, Frc::Frc2);
        assert_eq!(edge.fow, Fow::SingleCarriageway);
        assert_eq!(edge.length_m, 5000.0); // Explicit length preserved

        // Bearing should be roughly east (~90 degrees)
        assert!(edge.bearing_start > 80.0 && edge.bearing_start < 100.0);
    }

    #[test]
    fn test_auto_length_calculation() {
        let (network, _spatial) = TestNetworkBuilder::new()
            .add_node(1, 52.0, 13.0)
            .add_node(2, 52.0, 14.0)
            .add_edge_auto_length(100, 1, 2, Frc::Frc2, Fow::SingleCarriageway)
            .build();

        let edge_idx = network
            .edge_id_to_index
            .as_ref()
            .unwrap()
            .get(&100)
            .unwrap();
        let edge = network.edge(*edge_idx).unwrap();

        // 1 degree of longitude at 52° lat is roughly 68km
        assert!(edge.length_m > 60000.0 && edge.length_m < 75000.0);
    }

    #[test]
    fn test_spatial_index_works() {
        let (_, spatial) = TestNetworkBuilder::new()
            .add_node(1, 52.0, 13.0)
            .add_node(2, 52.0, 13.001) // Very close
            .add_edge(1, 1, 2, 100.0, Frc::Frc4, Fow::SingleCarriageway)
            .build();

        // Search near the edge
        let center = Point::new(13.0005, 52.0); // (lon, lat)
        let nearby = spatial.find_nearby(center, 500.0);

        assert_eq!(nearby.len(), 1);
    }

    #[test]
    fn test_multi_edge_network() {
        // Build a simple triangle network
        let (network, spatial) = TestNetworkBuilder::new()
            .add_node(1, 52.0, 13.0)
            .add_node(2, 52.01, 13.0)
            .add_node(3, 52.005, 13.01)
            .add_edge(1, 1, 2, 1000.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(2, 2, 3, 1000.0, Frc::Frc3, Fow::SingleCarriageway)
            .add_edge(3, 3, 1, 1000.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();

        assert_eq!(network.node_count(), 3);
        assert_eq!(network.edge_count(), 3);
        assert_eq!(spatial.len(), 3);
    }

    #[test]
    #[should_panic(expected = "Node with id 1 already exists")]
    fn test_duplicate_node_panics() {
        TestNetworkBuilder::new()
            .add_node(1, 52.0, 13.0)
            .add_node(1, 52.1, 13.1); // Duplicate!
    }

    #[test]
    #[should_panic(expected = "Start node 99 does not exist")]
    fn test_missing_node_panics() {
        TestNetworkBuilder::new()
            .add_node(1, 52.0, 13.0)
            .add_edge(1, 99, 1, 1000.0, Frc::Frc3, Fow::SingleCarriageway)
            .build();
    }
}
