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

    /// Check whether a node is a *valid* node per OpenLR whitepaper Rule 4 (§6).
    ///
    /// A node is **invalid** when a route search can step over it without making
    /// a choice: it connects exactly two distinct neighbours, with at most one
    /// incoming and one outgoing edge per neighbour and equal in/out degree
    /// (i.e. 1-in/1-out, or 2-in/2-out on a bidirectional through-road).
    /// Everything else — junctions, dead ends, one-way merges/splits — is valid.
    ///
    /// Encoders place LRPs on valid nodes, so a decoded path terminating on an
    /// invalid node has most likely stopped short of the intended endpoint.
    pub fn is_valid_node(&self, node_idx: NodeIndex) -> bool {
        use petgraph::Direction;

        let mut in_deg = 0usize;
        let mut out_deg = 0usize;
        // (neighbour, incoming count, outgoing count) — at most 2 tracked before bailing
        let mut neighbours: Vec<(NodeIndex, usize, usize)> = Vec::with_capacity(3);

        let mut record = |other: NodeIndex, incoming: bool| -> bool {
            if other == node_idx {
                return false; // self-loop: not a through-node
            }
            match neighbours.iter_mut().find(|(n, _, _)| *n == other) {
                Some(entry) => {
                    if incoming {
                        entry.1 += 1;
                    } else {
                        entry.2 += 1;
                    }
                }
                None => {
                    if neighbours.len() >= 2 {
                        return false; // 3+ distinct neighbours: junction
                    }
                    neighbours.push(if incoming {
                        (other, 1, 0)
                    } else {
                        (other, 0, 1)
                    });
                }
            }
            true
        };

        for e in self.graph.edges_directed(node_idx, Direction::Incoming) {
            in_deg += 1;
            if !record(e.source(), true) {
                return true;
            }
        }
        for e in self.graph.edges_directed(node_idx, Direction::Outgoing) {
            out_deg += 1;
            if !record(e.target(), false) {
                return true;
            }
        }

        if neighbours.len() != 2 || in_deg != out_deg {
            return true;
        }
        // Each neighbour may contribute at most one edge in each direction;
        // parallel edges to the same neighbour imply a real choice.
        if neighbours.iter().any(|(_, i, o)| *i > 1 || *o > 1) {
            return true;
        }
        false
    }

    /// If `node_idx` is an invalid node (see [`is_valid_node`](Self::is_valid_node)),
    /// return the single outgoing edge that continues travel after arriving via
    /// `incoming` — i.e. the only exit that is not a U-turn. Returns `None` at a
    /// valid node, or if `incoming` does not actually end at `node_idx`.
    pub fn forced_continuation(
        &self,
        node_idx: NodeIndex,
        incoming: EdgeIndex,
    ) -> Option<EdgeIndex> {
        let (from, to) = self.graph.edge_endpoints(incoming)?;
        if to != node_idx || self.is_valid_node(node_idx) {
            return None;
        }
        self.graph
            .edges(node_idx)
            .find(|e| e.target() != from)
            .map(|e| e.id())
    }

    /// Mirror of [`forced_continuation`](Self::forced_continuation) for walking
    /// backwards: if `node_idx` is invalid, return the single incoming edge a
    /// traveller must have arrived on before leaving via `outgoing`.
    pub fn forced_predecessor(
        &self,
        node_idx: NodeIndex,
        outgoing: EdgeIndex,
    ) -> Option<EdgeIndex> {
        use petgraph::Direction;
        let (from, to) = self.graph.edge_endpoints(outgoing)?;
        if from != node_idx || self.is_valid_node(node_idx) {
            return None;
        }
        self.graph
            .edges_directed(node_idx, Direction::Incoming)
            .find(|e| e.source() != to)
            .map(|e| e.id())
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

    fn ls(a: (f64, f64), b: (f64, f64)) -> LineString<f64> {
        LineString::from(vec![a, b])
    }

    /// Build a small network: A→X→B one-way through-road, plus a junction J
    /// with three neighbours, and a dead end D hanging off B.
    fn topo_network() -> (RoadNetwork, HashMap<&'static str, NodeIndex>) {
        let mut net = RoadNetwork::new();
        let pts: [(&str, i64, (f64, f64)); 6] = [
            ("A", 1, (0.0, 0.0)),
            ("X", 2, (0.001, 0.0)),
            ("B", 3, (0.002, 0.0)),
            ("J", 4, (0.003, 0.0)),
            ("C", 5, (0.003, 0.001)),
            ("D", 6, (0.002, -0.001)),
        ];
        let mut idx = HashMap::new();
        for (name, id, (x, y)) in pts {
            idx.insert(name, net.get_or_add_node(id, Point::new(x, y)));
        }
        let p = |n: &str| pts.iter().find(|q| q.0 == n).unwrap().2;
        let mut eid = 0;
        let mut add = |net: &mut RoadNetwork, a: &str, b: &str| {
            eid += 1;
            let (ia, ib) = (
                pts.iter().find(|q| q.0 == a).unwrap().1,
                pts.iter().find(|q| q.0 == b).unwrap().1,
            );
            net.add_edge(
                ia,
                ib,
                Edge::new(eid, ls(p(a), p(b)), Frc::Frc4, Fow::SingleCarriageway),
            )
        };
        add(&mut net, "A", "X"); // one-way A→X
        add(&mut net, "X", "B"); // one-way X→B
        add(&mut net, "B", "J");
        add(&mut net, "J", "B");
        add(&mut net, "J", "C");
        add(&mut net, "C", "J");
        add(&mut net, "B", "D"); // dead-end spur, two-way
        add(&mut net, "D", "B");
        (net, idx)
    }

    #[test]
    fn test_is_valid_node_rule4() {
        let (net, n) = topo_network();
        // X: 1-in/1-out through-node → invalid
        assert!(!net.is_valid_node(n["X"]));
        // A: source only (0-in/1-out) → valid
        assert!(net.is_valid_node(n["A"]));
        // B: neighbours X, J, D → junction → valid
        assert!(net.is_valid_node(n["B"]));
        // J: neighbours B, C, bidirectional, 2-in/2-out → invalid (through-node)
        assert!(!net.is_valid_node(n["J"]));
        // C, D: dead ends (single neighbour) → valid
        assert!(net.is_valid_node(n["C"]));
        assert!(net.is_valid_node(n["D"]));
    }

    #[test]
    fn test_forced_continuation_and_predecessor() {
        let (net, n) = topo_network();
        let edge_between = |a: NodeIndex, b: NodeIndex| net.graph.find_edge(a, b).unwrap();

        let ax = edge_between(n["A"], n["X"]);
        let xb = edge_between(n["X"], n["B"]);
        let bj = edge_between(n["B"], n["J"]);
        let jc = edge_between(n["J"], n["C"]);

        // Arriving at X via A→X must continue on X→B
        assert_eq!(net.forced_continuation(n["X"], ax), Some(xb));
        // Arriving at J via B→J must continue to C (not U-turn back to B)
        assert_eq!(net.forced_continuation(n["J"], bj), Some(jc));
        // B is a valid node: no forced continuation
        assert_eq!(net.forced_continuation(n["B"], xb), None);
        // Mismatched incoming edge
        assert_eq!(net.forced_continuation(n["X"], xb), None);

        // Backwards: leaving X via X→B means we arrived via A→X
        assert_eq!(net.forced_predecessor(n["X"], xb), Some(ax));
        // Leaving J via J→C means we arrived via B→J
        assert_eq!(net.forced_predecessor(n["J"], jc), Some(bj));
        assert_eq!(net.forced_predecessor(n["B"], bj), None);
    }
}
