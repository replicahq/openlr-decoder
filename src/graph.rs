use geo::{GeodesicBearing, GeodesicLength, LineString, Point};
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Functional Road Class (OpenLR spec)
/// FRC0 = Main road (motorway)
/// FRC7 = Other (lowest importance)
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum Frc {
    Frc0 = 0, // Main road (motorway, freeway)
    Frc1 = 1, // First class road (major routes)
    Frc2 = 2, // Second class road (regional routes)
    Frc3 = 3, // Third class road (local connecting roads)
    Frc4 = 4, // Fourth class road (local roads of high importance)
    Frc5 = 5, // Fifth class road (local roads)
    Frc6 = 6, // Sixth class road (local roads of low importance)
    Frc7 = 7, // Other (parking, service roads, etc.)
}

impl Frc {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => Frc::Frc0,
            1 => Frc::Frc1,
            2 => Frc::Frc2,
            3 => Frc::Frc3,
            4 => Frc::Frc4,
            5 => Frc::Frc5,
            6 => Frc::Frc6,
            _ => Frc::Frc7,
        }
    }

    /// Map OSM highway tag to FRC (aligned with HERE map classification)
    /// HERE uses importance-based FRC rather than strict road type hierarchy
    /// HERE classifies slip roads (ramps) separately from main roads
    pub fn from_osm_highway(highway: &str) -> Self {
        match highway {
            "motorway" => Frc::Frc0,
            "trunk" => Frc::Frc1,
            "primary" => Frc::Frc2,
            "secondary" => Frc::Frc3,
            // HERE classifies slip roads/ramps as FRC 3-4 (connecting roads)
            // regardless of the main road they serve
            "motorway_link" | "trunk_link" | "primary_link" => Frc::Frc3,
            // HERE classifies service roads and unclassified roads as FRC 4
            // (unclassified = minor public roads, typically local connections)
            "tertiary" | "secondary_link" | "tertiary_link" | "service" | "unclassified" => {
                Frc::Frc4
            }
            // Track roads and residential - minor local roads
            "residential" | "track" => Frc::Frc5,
            "living_street" => Frc::Frc6,
            _ => Frc::Frc7,
        }
    }

    /// Check if another FRC is within tolerance (per OpenLR spec: ±1 or exact)
    pub fn is_compatible(&self, other: Frc, tolerance: u8) -> bool {
        let diff = (*self as i8 - other as i8).unsigned_abs();
        diff <= tolerance
    }
}

/// Form of Way (OpenLR spec)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum Fow {
    Undefined = 0,
    Motorway = 1,
    MultipleCarriageway = 2,
    SingleCarriageway = 3,
    Roundabout = 4,
    TrafficSquare = 5,
    SlipRoad = 6,
    Other = 7,
}

impl Fow {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => Fow::Undefined,
            1 => Fow::Motorway,
            2 => Fow::MultipleCarriageway,
            3 => Fow::SingleCarriageway,
            4 => Fow::Roundabout,
            5 => Fow::TrafficSquare,
            6 => Fow::SlipRoad,
            _ => Fow::Other,
        }
    }

    /// Map OSM tags to FOW
    pub fn from_osm_tags(
        highway: &str,
        junction: Option<&str>,
        oneway: Option<&str>,
        lanes: Option<u8>,
    ) -> Self {
        if junction == Some("roundabout") {
            return Fow::Roundabout;
        }

        match highway {
            "motorway" => Fow::Motorway,
            "motorway_link" | "trunk_link" | "primary_link" | "secondary_link"
            | "tertiary_link" => Fow::SlipRoad,
            "trunk" | "primary" | "secondary" => {
                // Multiple carriageway if has many lanes or is oneway with parallel road
                if lanes.unwrap_or(1) >= 4 || oneway == Some("yes") {
                    Fow::MultipleCarriageway
                } else {
                    Fow::SingleCarriageway
                }
            }
            _ => Fow::SingleCarriageway,
        }
    }

    /// Check FOW compatibility (OpenLR allows some flexibility)
    /// For cross-provider decoding, FOW is treated as a hint rather than strict filter
    pub fn is_compatible(&self, other: Fow) -> bool {
        // Undefined matches anything
        if *self == Fow::Undefined || other == Fow::Undefined {
            return true;
        }
        // Exact match
        if *self == other {
            return true;
        }
        // For cross-provider decoding, allow flexibility between similar FOW types:
        // - Single/Multiple/TrafficSquare/Other are common road types
        // - Only Motorway, Roundabout, and SlipRoad should be strictly matched
        let is_generic_road = |f: &Fow| {
            matches!(
                f,
                Fow::SingleCarriageway | Fow::MultipleCarriageway | Fow::TrafficSquare | Fow::Other
            )
        };
        is_generic_road(self) && is_generic_road(&other)
    }
}

/// A node in the road network (intersection or endpoint)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    pub id: i64,
    pub coord: Point<f64>,
}

/// An edge in the road network (road segment)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Edge {
    pub id: u64, // stableEdgeId from the source data
    pub geometry: LineString<f64>,
    pub length_m: f64,
    pub frc: Frc,
    pub fow: Fow,
    pub bearing_start: f64, // Bearing at start of edge (0-360)
    pub bearing_end: f64,   // Bearing at end of edge (0-360)
}

impl Edge {
    /// Create edge from geometry with computed attributes
    pub fn new(id: u64, geometry: LineString<f64>, frc: Frc, fow: Fow) -> Self {
        let length_m = geometry.geodesic_length();
        let (bearing_start, bearing_end) = Self::compute_bearings(&geometry);

        Edge {
            id,
            geometry,
            length_m,
            frc,
            fow,
            bearing_start,
            bearing_end,
        }
    }

    fn compute_bearings(geom: &LineString<f64>) -> (f64, f64) {
        let coords: Vec<_> = geom.coords().collect();
        if coords.len() < 2 {
            return (0.0, 0.0);
        }

        // Start bearing: from first point toward second
        let start_bearing = Point::new(coords[0].x, coords[0].y)
            .geodesic_bearing(Point::new(coords[1].x, coords[1].y));

        // End bearing: from second-to-last toward last
        let n = coords.len();
        let end_bearing = Point::new(coords[n - 2].x, coords[n - 2].y)
            .geodesic_bearing(Point::new(coords[n - 1].x, coords[n - 1].y));

        // Normalize to 0-360
        let normalize = |b: f64| ((b % 360.0) + 360.0) % 360.0;

        (normalize(start_bearing), normalize(end_bearing))
    }
}

/// The road network graph
pub struct RoadNetwork {
    pub graph: DiGraph<Node, Edge>,
    pub node_id_to_index: HashMap<i64, NodeIndex>,
    pub edge_id_to_index: HashMap<u64, EdgeIndex>,
}

impl RoadNetwork {
    pub fn new() -> Self {
        RoadNetwork {
            graph: DiGraph::new(),
            node_id_to_index: HashMap::new(),
            edge_id_to_index: HashMap::new(),
        }
    }

    /// Add a node to the network
    pub fn add_node(&mut self, node: Node) -> NodeIndex {
        let id = node.id;
        let idx = self.graph.add_node(node);
        self.node_id_to_index.insert(id, idx);
        idx
    }

    /// Get or create a node
    pub fn get_or_add_node(&mut self, id: i64, coord: Point<f64>) -> NodeIndex {
        if let Some(&idx) = self.node_id_to_index.get(&id) {
            return idx;
        }
        self.add_node(Node { id, coord })
    }

    /// Add an edge to the network
    pub fn add_edge(&mut self, from_node: i64, to_node: i64, edge: Edge) -> EdgeIndex {
        let from_idx = *self
            .node_id_to_index
            .get(&from_node)
            .expect("from_node must exist");
        let to_idx = *self
            .node_id_to_index
            .get(&to_node)
            .expect("to_node must exist");

        let edge_id = edge.id;
        let idx = self.graph.add_edge(from_idx, to_idx, edge);
        self.edge_id_to_index.insert(edge_id, idx);
        idx
    }

    /// Get edge by index
    pub fn edge(&self, idx: EdgeIndex) -> Option<&Edge> {
        self.graph.edge_weight(idx)
    }

    /// Get node by index
    pub fn node(&self, idx: NodeIndex) -> Option<&Node> {
        self.graph.node_weight(idx)
    }

    /// Get all outgoing edges from a node
    pub fn outgoing_edges(&self, node_idx: NodeIndex) -> impl Iterator<Item = EdgeIndex> + '_ {
        self.graph.edges(node_idx).map(|e| e.id())
    }

    /// Get the target node of an edge
    pub fn edge_target(&self, edge_idx: EdgeIndex) -> Option<NodeIndex> {
        self.graph.edge_endpoints(edge_idx).map(|(_, t)| t)
    }

    /// Get the source node of an edge
    pub fn edge_source(&self, edge_idx: EdgeIndex) -> Option<NodeIndex> {
        self.graph.edge_endpoints(edge_idx).map(|(s, _)| s)
    }

    pub fn node_count(&self) -> usize {
        self.graph.node_count()
    }

    pub fn edge_count(&self) -> usize {
        self.graph.edge_count()
    }
}

impl Default for RoadNetwork {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frc_from_osm() {
        assert_eq!(Frc::from_osm_highway("motorway"), Frc::Frc0);
        assert_eq!(Frc::from_osm_highway("primary"), Frc::Frc2);
        assert_eq!(Frc::from_osm_highway("unclassified"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("residential"), Frc::Frc5);
    }

    #[test]
    fn test_frc_compatibility() {
        assert!(Frc::Frc2.is_compatible(Frc::Frc2, 1));
        assert!(Frc::Frc2.is_compatible(Frc::Frc3, 1));
        assert!(!Frc::Frc2.is_compatible(Frc::Frc5, 1));
    }

    #[test]
    fn test_fow_compatibility() {
        assert!(Fow::SingleCarriageway.is_compatible(Fow::MultipleCarriageway));
        assert!(Fow::Undefined.is_compatible(Fow::Motorway));
        assert!(!Fow::Roundabout.is_compatible(Fow::Motorway));
    }
}
