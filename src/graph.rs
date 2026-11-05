//! Directed road graph. Nodes are junctions and shape breaks with an OSM node
//! id; edges are road segments with geometry, length and road class.
//!
//! [`Frc`] and [`Fow`] are OpenLR's road-class attributes, inferred from OSM
//! tags.

use geo::{Haversine, Length, LineString, Point};
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Functional Road Class, 0 (most important) to 7. HERE's FC1–FC5 encode as
/// Frc0–Frc4; HERE never uses Frc5–Frc7.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum Frc {
    Frc0 = 0, // motorways
    Frc1 = 1, // major routes between/through cities
    Frc2 = 2, // high-volume roads feeding major routes
    Frc3 = 3, // roads between neighborhoods
    Frc4 = 4, // local/residential
    Frc5 = 5,
    Frc6 = 6,
    Frc7 = 7,
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

    /// Map an OSM `highway` tag onto HERE's Frc0–Frc4 range.
    pub fn from_osm_highway(highway: &str) -> Self {
        match highway {
            "motorway" => Frc::Frc0,
            "trunk" => Frc::Frc1,
            "primary" => Frc::Frc2,
            "secondary" => Frc::Frc3,
            "motorway_link" | "trunk_link" | "primary_link" => Frc::Frc3,
            "tertiary" | "secondary_link" | "tertiary_link" | "unclassified" | "residential"
            | "living_street" | "service" | "track" => Frc::Frc4,
            // Not drivable in HERE's network. Frc7 is far enough from Frc0–Frc4
            // that the default ±2 tolerance never matches it.
            _ => Frc::Frc7,
        }
    }

    /// Residential, living_street, service and track. These share Frc4 with
    /// tertiary roads, so A* tells them apart with a cost penalty instead.
    pub fn is_access_road(highway: &str) -> bool {
        matches!(
            highway,
            "residential" | "living_street" | "service" | "track"
        )
    }

    /// Whether the two classes are at most `tolerance` levels apart.
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

    /// Infer FOW from OSM tags. The loader has no `junction` or `oneway`
    /// columns and passes `None` for both.
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
                // OSM maps dual carriageways as two one-way ways.
                if lanes.unwrap_or(1) >= 4 || oneway == Some("yes") {
                    Fow::MultipleCarriageway
                } else {
                    Fow::SingleCarriageway
                }
            }
            _ => Fow::SingleCarriageway,
        }
    }

    /// How well an edge with this FOW stands in for an LRP with FOW `other`:
    /// 1.0 for an exact match, 0.0 for incompatible. The matrix comes from
    /// TomTom's reference implementation.
    pub fn substitution_score(&self, other: Fow) -> f64 {
        // Rows: LRP FOW. Columns: edge FOW.
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    pub id: i64, // OSM node id
    pub coord: Point<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Edge {
    pub id: u64, // stableEdgeId
    pub geometry: LineString<f64>,
    pub length_m: f64,
    pub frc: Frc,
    pub fow: Fow,
    /// See [`Frc::is_access_road`].
    pub is_access_road: bool,
}

impl Edge {
    /// An edge with its length computed from `geometry`, as the loader does,
    /// and `is_access_road` false.
    pub fn new(id: u64, geometry: LineString<f64>, frc: Frc, fow: Fow) -> Self {
        let length_m = Haversine.length(&geometry);

        Edge {
            id,
            geometry,
            length_m,
            frc,
            fow,
            is_access_road: false,
        }
    }
}

/// Directed road graph. The id → index maps are only needed while loading;
/// [`compact`](Self::compact) drops them.
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

    pub fn new_with_capacity(node_hint: usize, edge_hint: usize) -> Self {
        RoadNetwork {
            graph: DiGraph::with_capacity(node_hint, edge_hint),
            node_id_to_index: Some(HashMap::with_capacity(node_hint)),
            edge_id_to_index: Some(HashMap::with_capacity(edge_hint)),
        }
    }

    /// Drop the id → index maps to free memory once loading is done. After
    /// this, `add_node`, `get_or_add_node` and `add_edge` panic.
    pub fn compact(&mut self) {
        self.node_id_to_index = None;
        self.edge_id_to_index = None;
    }

    pub fn add_node(&mut self, node: Node) -> NodeIndex {
        let id = node.id;
        let idx = self.graph.add_node(node);
        self.node_id_to_index
            .as_mut()
            .expect("cannot add_node after compact()")
            .insert(id, idx);
        idx
    }

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

    /// Panics if either endpoint node hasn't been added yet.
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

    pub fn edge(&self, idx: EdgeIndex) -> Option<&Edge> {
        self.graph.edge_weight(idx)
    }

    pub fn node(&self, idx: NodeIndex) -> Option<&Node> {
        self.graph.node_weight(idx)
    }

    pub fn outgoing_edges(&self, node_idx: NodeIndex) -> impl Iterator<Item = EdgeIndex> + '_ {
        self.graph.edges(node_idx).map(|e| e.id())
    }

    pub fn edge_target(&self, edge_idx: EdgeIndex) -> Option<NodeIndex> {
        self.graph.edge_endpoints(edge_idx).map(|(_, t)| t)
    }

    pub fn edge_source(&self, edge_idx: EdgeIndex) -> Option<NodeIndex> {
        self.graph.edge_endpoints(edge_idx).map(|(s, _)| s)
    }

    /// Whether a node is valid under OpenLR Rule 4 (whitepaper §6).
    ///
    /// A node is invalid when a route search passes through it without a
    /// choice: exactly two neighbours, at most one edge each way per neighbour,
    /// and equal in- and out-degree. That covers 1-in/1-out nodes and the
    /// middle of a two-way road. Junctions, dead ends and one-way merges or
    /// splits are all valid.
    ///
    /// Encoders put LRPs on valid nodes, so a decoded path that ends on an
    /// invalid one has probably stopped short.
    pub fn is_valid_node(&self, node_idx: NodeIndex) -> bool {
        use petgraph::Direction;

        // (neighbour, edges in from it, edges out to it)
        let mut neighbours: Vec<(NodeIndex, usize, usize)> = Vec::with_capacity(2);
        let incoming = self
            .graph
            .edges_directed(node_idx, Direction::Incoming)
            .map(|e| (e.source(), true));
        let outgoing = self.graph.edges(node_idx).map(|e| (e.target(), false));

        for (other, is_incoming) in incoming.chain(outgoing) {
            if other == node_idx {
                return true; // a self-loop is a choice
            }
            let entry = match neighbours.iter().position(|n| n.0 == other) {
                Some(i) => &mut neighbours[i],
                None if neighbours.len() == 2 => return true, // a third neighbour
                None => {
                    neighbours.push((other, 0, 0));
                    neighbours.last_mut().unwrap()
                }
            };
            if is_incoming {
                entry.1 += 1;
            } else {
                entry.2 += 1;
            }
        }

        let ins: usize = neighbours.iter().map(|n| n.1).sum();
        let outs: usize = neighbours.iter().map(|n| n.2).sum();
        let pass_through = neighbours.len() == 2
            && ins == outs
            && neighbours.iter().all(|&(_, i, o)| i <= 1 && o <= 1);
        !pass_through
    }

    /// At an invalid node, the one exit that isn't a U-turn after arriving via
    /// `incoming`. `None` if the node is valid or `incoming` doesn't end there.
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

    /// [`forced_continuation`](Self::forced_continuation) in reverse: the edge
    /// a traveller must have arrived on before leaving via `outgoing`.
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
        assert_eq!(Fow::Motorway.substitution_score(Fow::Motorway), 1.0);
        assert_eq!(
            Fow::SingleCarriageway.substitution_score(Fow::SingleCarriageway),
            1.0
        );

        // Undefined should match anything at 0.5
        assert_eq!(Fow::Undefined.substitution_score(Fow::Motorway), 0.5);
        assert_eq!(Fow::Motorway.substitution_score(Fow::Undefined), 0.5);

        // Some providers encode freeways as MultipleCarriageway.
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

    /// A→X→B one-way, then two-way B–J, J–C and B–D. X and J are pass-through
    /// nodes; B is a junction; A, C and D are ends.
    fn topo_network() -> (RoadNetwork, HashMap<&'static str, NodeIndex>) {
        let nodes = [
            ("A", (0.0, 0.0)),
            ("X", (0.001, 0.0)),
            ("B", (0.002, 0.0)),
            ("J", (0.003, 0.0)),
            ("C", (0.003, 0.001)),
            ("D", (0.002, -0.001)),
        ];
        let edges = [
            ("A", "X"),
            ("X", "B"),
            ("B", "J"),
            ("J", "B"),
            ("J", "C"),
            ("C", "J"),
            ("B", "D"),
            ("D", "B"),
        ];

        let mut net = RoadNetwork::new();
        let mut idx = HashMap::new();
        let osm_id = |name| nodes.iter().position(|n| n.0 == name).unwrap() as i64;
        let coord = |name| nodes[osm_id(name) as usize].1;
        for (name, (x, y)) in nodes {
            idx.insert(name, net.get_or_add_node(osm_id(name), Point::new(x, y)));
        }
        for (id, (a, b)) in edges.into_iter().enumerate() {
            let geom = LineString::from(vec![coord(a), coord(b)]);
            let edge = Edge::new(id as u64, geom, Frc::Frc4, Fow::SingleCarriageway);
            net.add_edge(osm_id(a), osm_id(b), edge);
        }
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
