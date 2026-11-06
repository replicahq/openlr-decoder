//! Builds small road networks in code, without parquet files.
//!
//! # Example
//!
//! ```rust
//! use road_network::test_utils::TestNetworkBuilder;
//! use road_network::{Frc, Fow};
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

use crate::graph::{Edge, RoadNetwork, RoadNetworkBuilder};
use crate::road_class::{Fow, Frc};
use crate::spatial::{EdgeEnvelope, SpatialIndex};
use geo::{LineString, Point};
use std::collections::HashMap;

/// Network builder with caller-chosen node and edge ids. Edges get a straight
/// geometry between their nodes unless added with [`Self::add_edge_via`].
pub struct TestNetworkBuilder {
    nodes: HashMap<i64, (f64, f64)>, // id -> (lat, lon)
    edges: Vec<PendingEdge>,
}

struct PendingEdge {
    id: u64,
    start_node: i64,
    end_node: i64,
    via: Vec<(f64, f64)>,  // (lat, lon) shape points between the end nodes
    length_m: Option<f64>, // None: geodesic length of the geometry
    frc: Frc,
    fow: Fow,
    is_access_road: bool,
}

impl TestNetworkBuilder {
    pub fn new() -> Self {
        TestNetworkBuilder {
            nodes: HashMap::new(),
            edges: Vec::new(),
        }
    }

    /// Takes (lat, lon), unlike geo's (x, y) order. Panics on a duplicate id.
    pub fn add_node(mut self, id: i64, lat: f64, lon: f64) -> Self {
        if self.nodes.contains_key(&id) {
            panic!("Node with id {} already exists", id);
        }
        self.nodes.insert(id, (lat, lon));
        self
    }

    /// `length_m` overrides the geometry length, so path lengths can be set
    /// independently of node placement. Bearings still come from the geometry.
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
            via: Vec::new(),
            length_m: Some(length_m),
            frc,
            fow,
            is_access_road: false,
        });
        self
    }

    /// Like [`Self::add_edge`], with `via` as (lat, lon) shape points between
    /// the end nodes.
    #[allow(clippy::too_many_arguments)]
    pub fn add_edge_via(
        mut self,
        id: u64,
        start_node: i64,
        end_node: i64,
        via: &[(f64, f64)],
        length_m: f64,
        frc: Frc,
        fow: Fow,
    ) -> Self {
        self.edges.push(PendingEdge {
            id,
            start_node,
            end_node,
            via: via.to_vec(),
            length_m: Some(length_m),
            frc,
            fow,
            is_access_road: false,
        });
        self
    }

    /// Like [`Self::add_edge`], but sets `Edge::is_access_road`.
    pub fn add_access_road(
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
            via: Vec::new(),
            length_m: Some(length_m),
            frc,
            fow,
            is_access_road: true,
        });
        self
    }

    /// Like [`Self::add_edge`], with the geodesic length of the straight geometry.
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
            via: Vec::new(),
            length_m: None,
            frc,
            fow,
            is_access_road: false,
        });
        self
    }

    /// Panics if any edge references a node that was never added.
    pub fn build(self) -> (RoadNetwork, SpatialIndex) {
        let mut network = RoadNetworkBuilder::new();

        for (&id, &(lat, lon)) in &self.nodes {
            network.get_or_add_node(id, Point::new(lon, lat));
        }

        let mut envelopes = Vec::new();

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

            let geometry: LineString<f64> = std::iter::once(*start_coord)
                .chain(pending.via.iter().copied())
                .chain(std::iter::once(*end_coord))
                .map(|(lat, lon)| (lon, lat))
                .collect();

            let mut edge = Edge::new(pending.id, geometry, pending.frc, pending.fow);
            edge.is_access_road = pending.is_access_road;

            if let Some(length) = pending.length_m {
                edge.length_m = length;
            }

            let from = network.node_index(pending.start_node).unwrap();
            let to = network.node_index(pending.end_node).unwrap();
            let edge_idx = network.add_edge(from, to, edge);

            let edge_geom = &network.edge(edge_idx).unwrap().geometry;
            envelopes.push(EdgeEnvelope::new(edge_idx, edge_geom));
        }

        let spatial = SpatialIndex::new(envelopes);

        (network.finish(), spatial)
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

        let edge_idx = network.find_edge_by_id(100).unwrap();
        let edge = network.edge(edge_idx).unwrap();

        assert_eq!(edge.id, 100);
        assert_eq!(edge.frc, Frc::Frc2);
        assert_eq!(edge.fow, Fow::SingleCarriageway);
        assert_eq!(edge.length_m, 5000.0);
    }

    #[test]
    fn test_auto_length_calculation() {
        let (network, _spatial) = TestNetworkBuilder::new()
            .add_node(1, 52.0, 13.0)
            .add_node(2, 52.0, 14.0)
            .add_edge_auto_length(100, 1, 2, Frc::Frc2, Fow::SingleCarriageway)
            .build();

        let edge_idx = network.find_edge_by_id(100).unwrap();
        let edge = network.edge(edge_idx).unwrap();

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

        let center = Point::new(13.0005, 52.0); // (lon, lat)
        let nearby = spatial.find_nearby(center, 500.0);

        assert_eq!(nearby.len(), 1);
    }

    #[test]
    fn test_multi_edge_network() {
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
            .add_node(1, 52.1, 13.1);
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
