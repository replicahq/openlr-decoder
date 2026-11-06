//! Bounded one-to-many shortest paths over a [`RoadNetwork`].
//!
//! [`ShortestPaths`] keeps its per-node arrays between searches and resets only
//! the nodes a search touched, so many short searches on a large network cost
//! no allocation. Keep one per thread.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;

use crate::graph::{Edge, RoadNetwork};

/// Reusable Dijkstra state. Run a search with [`run`](Self::run), then read
/// results with [`cost`](Self::cost) and [`path_to`](Self::path_to) until the
/// next run.
#[derive(Default)]
pub struct ShortestPaths {
    dist: Vec<f64>,
    prev: Vec<EdgeIndex>,
    settled: Vec<bool>,
    is_target: Vec<bool>,
    touched: Vec<u32>,
    heap: BinaryHeap<HeapItem>,
    start: Option<NodeIndex>,
    settled_count: usize,
}

impl ShortestPaths {
    pub fn new() -> Self {
        Self::default()
    }

    /// Dijkstra from `start`, which is reached at `start_cost`. `edge_cost`
    /// gives the cost of traversing an edge, or `None` to skip it; costs must
    /// not be negative. Nodes costing more than `max_cost` are never reached.
    ///
    /// The search stops once every node in `targets` is settled. With no
    /// targets it settles everything within `max_cost`.
    pub fn run<F>(
        &mut self,
        network: &RoadNetwork,
        start: NodeIndex,
        start_cost: f64,
        max_cost: f64,
        targets: &[NodeIndex],
        mut edge_cost: F,
    ) where
        F: FnMut(&Edge) -> Option<f64>,
    {
        self.reset(network.node_count());
        self.start = Some(start);
        if start_cost > max_cost {
            return;
        }

        let mut remaining = 0;
        for t in targets {
            if !std::mem::replace(&mut self.is_target[t.index()], true) {
                remaining += 1;
            }
        }

        self.visit(start.index(), start_cost, EdgeIndex::end());
        while let Some(HeapItem { cost, node }) = self.heap.pop() {
            let n = node.index();
            if self.settled[n] || cost > self.dist[n] {
                continue;
            }
            self.settled[n] = true;
            self.settled_count += 1;
            if self.is_target[n] {
                remaining -= 1;
                if remaining == 0 {
                    break;
                }
            }
            for er in network.graph.edges(node) {
                let Some(c) = edge_cost(er.weight()) else {
                    continue;
                };
                let next = cost + c;
                let t = er.target().index();
                if next <= max_cost && next < self.dist[t] {
                    self.visit(t, next, er.id());
                }
            }
        }

        for t in targets {
            self.is_target[t.index()] = false;
        }
    }

    /// Nodes the last run settled, a measure of how much work it did.
    pub fn settled_count(&self) -> usize {
        self.settled_count
    }

    /// Cost to reach `node` in the last run, or `None` if the search didn't
    /// settle it.
    pub fn cost(&self, node: NodeIndex) -> Option<f64> {
        let n = node.index();
        (n < self.settled.len() && self.settled[n]).then(|| self.dist[n])
    }

    /// Edges from the last run's start to `node`, in travel order. Empty for
    /// the start itself; `None` if the search didn't settle `node`.
    pub fn path_to(&self, network: &RoadNetwork, node: NodeIndex) -> Option<Vec<EdgeIndex>> {
        self.cost(node)?;
        let start = self.start?;
        let mut edges = Vec::new();
        let mut n = node;
        while n != start {
            let e = self.prev[n.index()];
            edges.push(e);
            n = network.edge_source(e)?;
        }
        edges.reverse();
        Some(edges)
    }

    fn visit(&mut self, n: usize, cost: f64, via: EdgeIndex) {
        if self.dist[n] == f64::INFINITY {
            self.touched.push(n as u32);
        }
        self.dist[n] = cost;
        self.prev[n] = via;
        self.heap.push(HeapItem {
            cost,
            node: NodeIndex::new(n),
        });
    }

    fn reset(&mut self, node_count: usize) {
        if self.dist.len() != node_count {
            self.dist = vec![f64::INFINITY; node_count];
            self.prev = vec![EdgeIndex::end(); node_count];
            self.settled = vec![false; node_count];
            self.is_target = vec![false; node_count];
            self.touched.clear();
        }
        for &n in &self.touched {
            self.dist[n as usize] = f64::INFINITY;
            self.settled[n as usize] = false;
        }
        self.touched.clear();
        self.heap.clear();
        self.start = None;
        self.settled_count = 0;
    }
}

/// Min-heap entry: `BinaryHeap` is a max-heap, so the order is reversed.
#[derive(Clone, Copy)]
struct HeapItem {
    cost: f64,
    node: NodeIndex,
}

impl PartialEq for HeapItem {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for HeapItem {}

impl PartialOrd for HeapItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapItem {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.total_cmp(&self.cost)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{Fow, Frc};
    use geo::{LineString, Point};

    /// 0 → 1 → 2 → 3 along a line, 0 → 3 directly as a long detour, and a
    /// one-way spur 2 → 4. Lengths in meters are roughly 111 per 0.001°.
    fn line_network() -> (RoadNetwork, Vec<NodeIndex>) {
        let mut net = RoadNetwork::new();
        let nodes: Vec<_> = (0..5)
            .map(|i| net.get_or_add_node(i, Point::new(0.001 * i as f64, 0.0)))
            .collect();
        let mut add = |a: i64, b: i64, frc: Frc, id: u64| {
            let pa = net.node(nodes[a as usize]).unwrap().coord;
            let pb = net.node(nodes[b as usize]).unwrap().coord;
            let geom = LineString::from(vec![pa, pb]);
            net.add_edge(a, b, Edge::new(id, geom, frc, Fow::SingleCarriageway));
        };
        add(0, 1, Frc::Frc4, 1);
        add(1, 2, Frc::Frc4, 2);
        add(2, 3, Frc::Frc4, 3);
        add(0, 3, Frc::Frc0, 4);
        add(2, 4, Frc::Frc4, 5);
        (net, nodes)
    }

    fn length(e: &Edge) -> Option<f64> {
        Some(e.length_m)
    }

    #[test]
    fn test_costs_and_paths() {
        let (net, n) = line_network();
        let mut sp = ShortestPaths::new();
        sp.run(&net, n[0], 5.0, f64::INFINITY, &[n[3], n[4]], length);

        // Equal length both ways (0→3 direct is the same 333 m); either is a
        // valid shortest path, so check the cost and that the path is connected.
        let c3 = sp.cost(n[3]).unwrap();
        assert!((c3 - 5.0 - 333.6).abs() < 1.0, "{c3}");
        let c4 = sp.cost(n[4]).unwrap();
        assert!((c4 - 5.0 - 222.4 - 222.4).abs() < 1.0, "{c4}");
        let p4 = sp.path_to(&net, n[4]).unwrap();
        let ids: Vec<u64> = p4.iter().map(|&e| net.edge(e).unwrap().id).collect();
        assert_eq!(ids, vec![1, 2, 5]);
        assert_eq!(sp.path_to(&net, n[0]), Some(vec![]));
    }

    #[test]
    fn test_edge_filter_and_bound() {
        let (net, n) = line_network();
        let mut sp = ShortestPaths::new();
        // Skip the Frc0 shortcut and stop at 250 m: node 3 (333 m) is out of reach.
        let no_frc0 = |e: &Edge| (e.frc != Frc::Frc0).then_some(e.length_m);
        sp.run(&net, n[0], 0.0, 250.0, &[n[3]], no_frc0);
        assert!(sp.cost(n[2]).is_some());
        assert_eq!(sp.cost(n[3]), None);
        assert_eq!(sp.path_to(&net, n[3]), None);
    }

    #[test]
    fn test_reuse_resets_state() {
        let (net, n) = line_network();
        let mut sp = ShortestPaths::new();
        sp.run(&net, n[0], 0.0, f64::INFINITY, &[], length);
        assert!(sp.cost(n[4]).is_some());
        // From 3 nothing is reachable; results from the first run must be gone.
        sp.run(&net, n[3], 0.0, f64::INFINITY, &[n[0]], length);
        assert_eq!(sp.cost(n[3]), Some(0.0));
        assert_eq!(sp.cost(n[4]), None);
        assert_eq!(sp.cost(n[0]), None);
    }

    #[test]
    fn test_stops_once_targets_settled() {
        let (net, n) = line_network();
        let mut sp = ShortestPaths::new();
        // Duplicate targets count once.
        sp.run(&net, n[0], 0.0, f64::INFINITY, &[n[1], n[1]], length);
        assert!(sp.cost(n[1]).is_some());
        assert_eq!(sp.cost(n[4]), None, "searched past the only target");
    }
}
