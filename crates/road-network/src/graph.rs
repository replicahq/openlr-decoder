//! Directed road graph. Nodes are junctions and shape breaks with an OSM node
//! id; edges are road segments with geometry, length and road class
//! ([`Frc`], [`Fow`]).

use geo::{Haversine, Length, LineString, Point};
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// Re-exported so `graph::Frc` and `graph::Fow` keep working.
pub use crate::road_class::{Fow, Frc};

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

/// Directed road graph, built with [`RoadNetworkBuilder`].
pub struct RoadNetwork {
    pub graph: DiGraph<Node, Edge>,
}

impl RoadNetwork {
    /// The edge whose [`Edge::id`] is `id`. A linear scan, for tests and
    /// one-off lookups.
    pub fn find_edge_by_id(&self, id: u64) -> Option<EdgeIndex> {
        self.graph.edge_indices().find(|&e| self.graph[e].id == id)
    }

    /// The node whose [`Node::id`] is `id`. A linear scan, for tests and
    /// one-off lookups.
    pub fn find_node_by_id(&self, id: i64) -> Option<NodeIndex> {
        self.graph.node_indices().find(|&n| self.graph[n].id == id)
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

    /// Whether `b` is the same road as `a` driven the other way: it joins the
    /// same two nodes in the opposite direction, has about the same length,
    /// and each edge's middle shape point lies within 5 m of the other. The
    /// shape check tells a two-way road's twin edges apart from two different
    /// roads between one pair of nodes, such as the halves of a loop.
    pub fn is_reverse_of(&self, a: EdgeIndex, b: EdgeIndex) -> bool {
        const MAX_OFFSET_M: f64 = 5.0;
        if a == b
            || self.edge_source(a) != self.edge_target(b)
            || self.edge_target(a) != self.edge_source(b)
        {
            return false;
        }
        let (Some(ea), Some(eb)) = (self.edge(a), self.edge(b)) else {
            return false;
        };
        if (ea.length_m - eb.length_m).abs() > MAX_OFFSET_M.max(0.05 * ea.length_m) {
            return false;
        }
        let middle_on = |from: &Edge, onto: &Edge| {
            let coords = &from.geometry.0;
            coords.get(coords.len() / 2).is_some_and(|&c| {
                crate::spatial::project_onto_line(Point::from(c), &onto.geometry)
                    .is_some_and(|p| p.distance_m <= MAX_OFFSET_M)
            })
        };
        middle_on(ea, eb) && middle_on(eb, ea)
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

/// Builds a [`RoadNetwork`], deduplicating nodes by OSM id. The id map is
/// dropped by [`finish`](Self::finish).
#[derive(Default)]
pub struct RoadNetworkBuilder {
    graph: DiGraph<Node, Edge>,
    node_index: HashMap<i64, NodeIndex>,
}

impl RoadNetworkBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_capacity(nodes: usize, edges: usize) -> Self {
        RoadNetworkBuilder {
            graph: DiGraph::with_capacity(nodes, edges),
            node_index: HashMap::with_capacity(nodes),
        }
    }

    /// The node with OSM id `id`, added at `coord` if it is new. An existing
    /// node keeps its first coordinate.
    pub fn get_or_add_node(&mut self, id: i64, coord: Point<f64>) -> NodeIndex {
        *self
            .node_index
            .entry(id)
            .or_insert_with(|| self.graph.add_node(Node { id, coord }))
    }

    pub fn node_index(&self, id: i64) -> Option<NodeIndex> {
        self.node_index.get(&id).copied()
    }

    pub fn add_edge(&mut self, from: NodeIndex, to: NodeIndex, edge: Edge) -> EdgeIndex {
        self.graph.add_edge(from, to, edge)
    }

    pub fn edge(&self, idx: EdgeIndex) -> Option<&Edge> {
        self.graph.edge_weight(idx)
    }

    pub fn finish(self) -> RoadNetwork {
        RoadNetwork { graph: self.graph }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

        let mut net = RoadNetworkBuilder::new();
        let mut idx = HashMap::new();
        let osm_id = |name| nodes.iter().position(|n| n.0 == name).unwrap() as i64;
        let coord = |name| nodes[osm_id(name) as usize].1;
        for (name, (x, y)) in nodes {
            idx.insert(name, net.get_or_add_node(osm_id(name), Point::new(x, y)));
        }
        for (id, (a, b)) in edges.into_iter().enumerate() {
            let geom = LineString::from(vec![coord(a), coord(b)]);
            let edge = Edge::new(id as u64, geom, Frc::Frc4, Fow::SingleCarriageway);
            net.add_edge(idx[a], idx[b], edge);
        }
        (net.finish(), idx)
    }

    #[test]
    fn test_builder_dedupes_nodes_and_finds_ids() {
        let mut b = RoadNetworkBuilder::new();
        let n1 = b.get_or_add_node(7, Point::new(1.0, 2.0));
        assert_eq!(b.get_or_add_node(7, Point::new(9.0, 9.0)), n1);
        let n2 = b.get_or_add_node(8, Point::new(1.0, 2.1));
        let geom = LineString::from(vec![(1.0, 2.0), (1.0, 2.1)]);
        let e = b.add_edge(
            n1,
            n2,
            Edge::new(42, geom, Frc::Frc4, Fow::SingleCarriageway),
        );
        let net = b.finish();
        assert_eq!(net.node_count(), 2);
        assert_eq!(net.node(n1).unwrap().coord, Point::new(1.0, 2.0));
        assert_eq!(net.find_node_by_id(8), Some(n2));
        assert_eq!(net.find_edge_by_id(42), Some(e));
        assert_eq!(net.find_edge_by_id(43), None);
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

    #[test]
    fn test_is_reverse_of() {
        // Nodes P and Q 100 m apart east-west, joined by a two-way straight
        // road (edges 0, 1) and by a one-way arc bulging 30 m north (edge 2).
        let mut b = RoadNetworkBuilder::new();
        let deg = 1.0 / 86_500.0;
        let p = b.get_or_add_node(1, Point::new(0.0, 39.0));
        let q = b.get_or_add_node(2, Point::new(100.0 * deg, 39.0));
        let straight =
            |from: f64, to: f64| LineString::from(vec![(from * deg, 39.0), (to * deg, 39.0)]);
        let arc = LineString::from(vec![
            (100.0 * deg, 39.0),
            (50.0 * deg, 39.0 + 30.0 / 111_000.0),
            (0.0, 39.0),
        ]);
        let edge = |id, g| Edge::new(id, g, Frc::Frc4, Fow::SingleCarriageway);
        let pq = b.add_edge(p, q, edge(0, straight(0.0, 100.0)));
        let qp = b.add_edge(q, p, edge(1, straight(100.0, 0.0)));
        let qp_arc = b.add_edge(q, p, edge(2, arc));
        let net = b.finish();

        assert!(net.is_reverse_of(pq, qp));
        assert!(net.is_reverse_of(qp, pq));
        assert!(
            !net.is_reverse_of(pq, qp_arc),
            "a loop half is a different road"
        );
        assert!(!net.is_reverse_of(pq, pq));
        assert!(!net.is_reverse_of(qp, qp_arc), "same direction");
    }
}
