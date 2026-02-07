use geo::{GeodesicBearing, GeodesicLength, LineString, Point};
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Functional Road Class — OpenLR binary values (0–7) that represent road importance.
///
/// HERE uses a 5-level classification (FC1–FC5, where FC1 is highest importance).
/// OpenLR encodes these as 0-indexed values: Frc0=FC1, Frc1=FC2, ..., Frc4=FC5.
/// Values Frc5–Frc7 exist in the OpenLR spec (3 bits = 0–7) but are unused by HERE.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum Frc {
    Frc0 = 0, // Highest importance: motorways, controlled-access highways (HERE: FC1)
    Frc1 = 1, // Major routes for travel between/through cities (HERE: FC2)
    Frc2 = 2, // High volume roads interconnecting with major routes (HERE: FC3)
    Frc3 = 3, // Moderate speed roads between neighborhoods (HERE: FC4)
    Frc4 = 4, // Lowest importance: local/residential roads (HERE: FC5)
    Frc5 = 5, // Not used by HERE
    Frc6 = 6, // Not used by HERE
    Frc7 = 7, // Not used by HERE
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

    /// Map OSM highway tag to OpenLR FRC value.
    ///
    /// Maps OSM road types to the 0-indexed FRC values used in OpenLR binary format.
    /// These correspond to HERE's FC1–FC5 importance levels (Frc0=FC1 ... Frc4=FC5).
    ///
    /// All navigable OSM roads map into the Frc0–Frc4 range to match HERE-encoded LRPs.
    /// Residential/access roads are also Frc4 (same as tertiary) but are deprioritized
    /// via A* cost penalties rather than FRC classification.
    pub fn from_osm_highway(highway: &str) -> Self {
        match highway {
            // Frc0: Controlled-access highways (highest importance)
            "motorway" => Frc::Frc0,
            // Frc1: Major routes for inter-city travel
            "trunk" => Frc::Frc1,
            // Frc2: High volume roads interconnecting with major routes
            "primary" => Frc::Frc2,
            // Frc3: Moderate speed roads connecting neighborhoods
            "secondary" => Frc::Frc3,
            "motorway_link" | "trunk_link" | "primary_link" => Frc::Frc3,
            // Frc4: Local/residential roads (lowest importance in HERE's scheme)
            "tertiary" | "secondary_link" | "tertiary_link" | "unclassified" | "residential"
            | "living_street" | "service" | "track" => Frc::Frc4,
            // Not in HERE's network — use Frc7 so these edges won't match
            // HERE-encoded LRPs (outside ±2 tolerance from Frc0–Frc4)
            _ => Frc::Frc7,
        }
    }

    /// Returns true for OSM highway tags that represent access/local roads
    /// (residential, living_street, service, track). These are deprioritized
    /// via A* cost penalties rather than FRC classification.
    pub fn is_access_road(highway: &str) -> bool {
        matches!(
            highway,
            "residential" | "living_street" | "service" | "track"
        )
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

    /// Get FOW substitution score for cross-provider decoding.
    /// Returns a score from 0.0 (incompatible) to 1.0 (exact match).
    /// Based on the official TomTom OpenLR reference implementation.
    ///
    /// The matrix encodes how well `other` (from the LRP) can be substituted
    /// by `self` (from the map edge).
    pub fn substitution_score(&self, other: Fow) -> f64 {
        // FOW substitution score matrix from official TomTom OpenLR implementation
        // Row = LRP's FOW (wanted), Column = Edge's FOW (actual/self)
        // Score 1.0 = exact match, 0.0 = incompatible, 0.5-0.75 = partial match
        #[rustfmt::skip]
        const FOW_SCORE_MATRIX: [[f64; 8]; 8] = [
            //       Und   Mwy   Mlt   Sgl   Rnd   Tsq   Slp   Oth
            /* Und */ [0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50],
            /* Mwy */ [0.50, 1.00, 0.75, 0.00, 0.00, 0.00, 0.00, 0.00],
            /* Mlt */ [0.50, 0.75, 1.00, 0.75, 0.50, 0.00, 0.00, 0.00],
            /* Sgl */ [0.50, 0.00, 0.75, 1.00, 0.50, 0.50, 0.00, 0.00],
            /* Rnd */ [0.50, 0.00, 0.50, 0.50, 1.00, 0.50, 0.00, 0.00],
            /* Tsq */ [0.50, 0.00, 0.00, 0.50, 0.50, 1.00, 0.00, 0.00],
            /* Slp */ [0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00],
            /* Oth */ [0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00],
        ];

        let wanted_idx = other as usize;
        let actual_idx = *self as usize;
        FOW_SCORE_MATRIX[wanted_idx][actual_idx]
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
    /// True for residential/living_street/service/track roads.
    /// Used by A* to apply a cost penalty, preferring tertiary over residential.
    pub is_access_road: bool,
}

impl Edge {
    /// Create edge from geometry with pre-computed metrics.
    /// Use this when metrics have already been computed during geometry parsing.
    pub fn from_precomputed(
        id: u64,
        geometry: LineString<f64>,
        frc: Frc,
        fow: Fow,
        length_m: f64,
        bearing_start: f64,
        bearing_end: f64,
        is_access_road: bool,
    ) -> Self {
        Edge {
            id,
            geometry,
            length_m,
            frc,
            fow,
            bearing_start,
            bearing_end,
            is_access_road,
        }
    }

    /// Create edge from geometry with computed attributes.
    /// Use this when parsing geometry separately from metrics computation.
    #[allow(dead_code)]
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
            is_access_road: false,
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
    pub node_id_to_index: Option<HashMap<i64, NodeIndex>>,
    pub edge_id_to_index: Option<HashMap<u64, EdgeIndex>>,
}

impl RoadNetwork {
    pub fn new() -> Self {
        RoadNetwork {
            graph: DiGraph::new(),
            node_id_to_index: Some(HashMap::new()),
            edge_id_to_index: Some(HashMap::new()),
        }
    }

    /// Create a new network with pre-allocated capacity.
    pub fn new_with_capacity(node_hint: usize, edge_hint: usize) -> Self {
        RoadNetwork {
            graph: DiGraph::with_capacity(node_hint, edge_hint),
            node_id_to_index: Some(HashMap::with_capacity(node_hint)),
            edge_id_to_index: Some(HashMap::with_capacity(edge_hint)),
        }
    }

    /// Drop the ID-to-index lookup maps to free memory.
    ///
    /// Call this after loading is complete and the maps are no longer needed.
    /// After compaction, `add_node`, `get_or_add_node`, and `add_edge` will panic.
    pub fn compact(&mut self) {
        self.node_id_to_index = None;
        self.edge_id_to_index = None;
    }

    /// Add a node to the network
    pub fn add_node(&mut self, node: Node) -> NodeIndex {
        let id = node.id;
        let idx = self.graph.add_node(node);
        self.node_id_to_index
            .as_mut()
            .expect("cannot add_node after compact()")
            .insert(id, idx);
        idx
    }

    /// Get or create a node
    pub fn get_or_add_node(&mut self, id: i64, coord: Point<f64>) -> NodeIndex {
        let map = self
            .node_id_to_index
            .as_ref()
            .expect("cannot get_or_add_node after compact()");
        if let Some(&idx) = map.get(&id) {
            return idx;
        }
        self.add_node(Node { id, coord })
    }

    /// Add an edge to the network
    pub fn add_edge(&mut self, from_node: i64, to_node: i64, edge: Edge) -> EdgeIndex {
        let node_map = self
            .node_id_to_index
            .as_ref()
            .expect("cannot add_edge after compact()");
        let from_idx = *node_map.get(&from_node).expect("from_node must exist");
        let to_idx = *node_map.get(&to_node).expect("to_node must exist");

        let edge_id = edge.id;
        let idx = self.graph.add_edge(from_idx, to_idx, edge);
        self.edge_id_to_index
            .as_mut()
            .expect("cannot add_edge after compact()")
            .insert(edge_id, idx);
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
        assert_eq!(Frc::from_osm_highway("trunk"), Frc::Frc1);
        assert_eq!(Frc::from_osm_highway("primary"), Frc::Frc2);
        assert_eq!(Frc::from_osm_highway("secondary"), Frc::Frc3);
        assert_eq!(Frc::from_osm_highway("motorway_link"), Frc::Frc3);
        // FRC4: Local/residential roads
        assert_eq!(Frc::from_osm_highway("tertiary"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("secondary_link"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("tertiary_link"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("unclassified"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("residential"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("living_street"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("service"), Frc::Frc4);
        assert_eq!(Frc::from_osm_highway("track"), Frc::Frc4);
        // FRC7: Not navigable
        assert_eq!(Frc::from_osm_highway("cycleway"), Frc::Frc7);
    }

    #[test]
    fn test_frc_compatibility() {
        assert!(Frc::Frc2.is_compatible(Frc::Frc2, 1));
        assert!(Frc::Frc2.is_compatible(Frc::Frc3, 1));
        assert!(!Frc::Frc2.is_compatible(Frc::Frc5, 1));
    }

    #[test]
    fn test_fow_substitution_score() {
        // Exact match should be 1.0
        assert_eq!(Fow::Motorway.substitution_score(Fow::Motorway), 1.0);
        assert_eq!(
            Fow::SingleCarriageway.substitution_score(Fow::SingleCarriageway),
            1.0
        );

        // Undefined should match anything at 0.5
        assert_eq!(Fow::Undefined.substitution_score(Fow::Motorway), 0.5);
        assert_eq!(Fow::Motorway.substitution_score(Fow::Undefined), 0.5);

        // Motorway and MultipleCarriageway should be compatible (0.75)
        // This is important for cross-provider decoding where freeways may
        // be encoded as MultipleCarriageway by some providers
        assert_eq!(
            Fow::Motorway.substitution_score(Fow::MultipleCarriageway),
            0.75
        );
        assert_eq!(
            Fow::MultipleCarriageway.substitution_score(Fow::Motorway),
            0.75
        );

        // Incompatible types should score 0.0
        assert_eq!(Fow::Roundabout.substitution_score(Fow::Motorway), 0.0);
        assert_eq!(
            Fow::SlipRoad.substitution_score(Fow::SingleCarriageway),
            0.0
        );
    }
}
