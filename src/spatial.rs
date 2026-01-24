use geo::{Closest, HaversineClosestPoint, HaversineDistance};
use geo::{Coord, LineString, Point};
use petgraph::graph::EdgeIndex;
use rstar::{RTree, RTreeObject, AABB};

/// Index mode selection for spatial queries
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SpatialIndexMode {
    /// Use R-tree for fast queries (slower startup, faster queries)
    /// This is the default mode for production use
    #[default]
    RTree,
    /// Use linear scan for fast startup (faster startup, slower queries)
    /// This is useful for debugging when only decoding a few locations
    LinearScan,
}

/// Envelope wrapper for an edge's geometry in the R-tree
#[derive(Debug, Clone)]
pub struct EdgeEnvelope {
    pub edge_idx: EdgeIndex,
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
    /// First point of geometry (for quick access)
    pub start_point: Point<f64>,
    /// Full geometry reference for precise distance calculations
    pub geometry: LineString<f64>,
}

impl EdgeEnvelope {
    pub fn new(edge_idx: EdgeIndex, geometry: LineString<f64>) -> Self {
        let coords: Vec<_> = geometry.coords().collect();

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

        for c in &coords {
            min_x = min_x.min(c.x);
            min_y = min_y.min(c.y);
            max_x = max_x.max(c.x);
            max_y = max_y.max(c.y);
        }

        let start_point = if coords.is_empty() {
            Point::new(0.0, 0.0)
        } else {
            Point::new(coords[0].x, coords[0].y)
        };

        EdgeEnvelope {
            edge_idx,
            min_x,
            min_y,
            max_x,
            max_y,
            start_point,
            geometry,
        }
    }

    /// Get the closest point on this edge to a query point
    pub fn closest_point(&self, query: Point<f64>) -> Point<f64> {
        match self.geometry.haversine_closest_point(&query) {
            Closest::SinglePoint(p) | Closest::Intersection(p) => p,
            Closest::Indeterminate => self.start_point,
        }
    }

    /// Distance in meters from query point to closest point on edge
    pub fn distance_to(&self, query: Point<f64>) -> f64 {
        let closest = self.closest_point(query);
        query.haversine_distance(&closest)
    }
}

impl RTreeObject for EdgeEnvelope {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners([self.min_x, self.min_y], [self.max_x, self.max_y])
    }
}

/// Internal storage for spatial index
enum SpatialIndexStorage {
    /// R-tree index for O(log N) queries
    RTree(RTree<EdgeEnvelope>),
    /// Linear scan storage for O(N) queries but instant startup
    LinearScan(Vec<EdgeEnvelope>),
}

/// Spatial index for fast edge lookup
pub struct SpatialIndex {
    storage: SpatialIndexStorage,
}

impl SpatialIndex {
    /// Build a new spatial index from edge envelopes using R-tree (default)
    pub fn new(edges: Vec<EdgeEnvelope>) -> Self {
        Self::with_mode(edges, SpatialIndexMode::RTree)
    }

    /// Build a new spatial index with the specified mode
    ///
    /// - `RTree`: Builds an R-tree index for O(log N) queries. Slower startup but fast queries.
    /// - `LinearScan`: Skips R-tree construction for instant startup. O(N) queries.
    pub fn with_mode(edges: Vec<EdgeEnvelope>, mode: SpatialIndexMode) -> Self {
        let storage = match mode {
            SpatialIndexMode::RTree => SpatialIndexStorage::RTree(RTree::bulk_load(edges)),
            SpatialIndexMode::LinearScan => SpatialIndexStorage::LinearScan(edges),
        };
        SpatialIndex { storage }
    }

    /// Create a spatial index with linear scan mode (fast startup, slow queries)
    ///
    /// This is useful for debugging scenarios where you only need to decode
    /// a small number of locations and want to minimize startup time.
    pub fn new_linear_scan(edges: Vec<EdgeEnvelope>) -> Self {
        Self::with_mode(edges, SpatialIndexMode::LinearScan)
    }

    /// Find all edges within a bounding box (degrees)
    /// The bbox is expanded from a center point by radius_m meters
    pub fn find_nearby(&self, center: Point<f64>, radius_m: f64) -> Vec<&EdgeEnvelope> {
        // Approximate degrees from meters (rough, ~111km per degree at equator)
        // Adjust for latitude
        let lat_rad = center.y().to_radians();
        let meters_per_deg_lat = 111_132.0;
        let meters_per_deg_lon = 111_132.0 * lat_rad.cos();

        let delta_lat = radius_m / meters_per_deg_lat;
        let delta_lon = radius_m / meters_per_deg_lon;

        let min_corner = [center.x() - delta_lon, center.y() - delta_lat];
        let max_corner = [center.x() + delta_lon, center.y() + delta_lat];
        let search_box = AABB::from_corners(min_corner, max_corner);

        match &self.storage {
            SpatialIndexStorage::RTree(rtree) => {
                rtree.locate_in_envelope_intersecting(&search_box).collect()
            }
            SpatialIndexStorage::LinearScan(edges) => {
                // Linear scan: check each edge's bounding box
                edges
                    .iter()
                    .filter(|e| {
                        e.min_x <= max_corner[0]
                            && e.max_x >= min_corner[0]
                            && e.min_y <= max_corner[1]
                            && e.max_y >= min_corner[1]
                    })
                    .collect()
            }
        }
    }

    /// Find edges within radius, sorted by distance to query point
    pub fn find_within_radius(
        &self,
        center: Point<f64>,
        radius_m: f64,
    ) -> Vec<(&EdgeEnvelope, f64)> {
        let candidates = self.find_nearby(center, radius_m * 1.5); // Slight buffer for bbox approximation

        let mut results: Vec<_> = candidates
            .into_iter()
            .map(|e| {
                let dist = e.distance_to(center);
                (e, dist)
            })
            .filter(|(_, dist)| *dist <= radius_m)
            .collect();

        results.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        results
    }

    pub fn len(&self) -> usize {
        match &self.storage {
            SpatialIndexStorage::RTree(rtree) => rtree.size(),
            SpatialIndexStorage::LinearScan(edges) => edges.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the index mode being used
    pub fn mode(&self) -> SpatialIndexMode {
        match &self.storage {
            SpatialIndexStorage::RTree(_) => SpatialIndexMode::RTree,
            SpatialIndexStorage::LinearScan(_) => SpatialIndexMode::LinearScan,
        }
    }
}

/// Calculate bearing difference, accounting for wraparound
/// Returns value in 0-180 range
pub fn bearing_difference(b1: f64, b2: f64) -> f64 {
    let diff = (b1 - b2).abs();
    if diff > 180.0 {
        360.0 - diff
    } else {
        diff
    }
}

/// Project a point onto a line and return the fraction along the line (0.0 to 1.0)
pub fn project_point_to_line_fraction(point: Point<f64>, line: &LineString<f64>) -> f64 {
    let coords: Vec<Coord<f64>> = line.coords().cloned().collect();
    if coords.len() < 2 {
        return 0.0;
    }

    let mut total_length = 0.0;
    let mut segments: Vec<(f64, f64)> = Vec::new(); // (segment_start_dist, segment_length)

    for i in 0..coords.len() - 1 {
        let p1 = Point::new(coords[i].x, coords[i].y);
        let p2 = Point::new(coords[i + 1].x, coords[i + 1].y);
        let seg_len = p1.haversine_distance(&p2);
        segments.push((total_length, seg_len));
        total_length += seg_len;
    }

    if total_length == 0.0 {
        return 0.0;
    }

    // Find closest segment and position
    let mut best_dist = f64::MAX;
    let mut best_fraction = 0.0;

    for (i, (seg_start, seg_len)) in segments.iter().enumerate() {
        let p1 = Point::new(coords[i].x, coords[i].y);
        let p2 = Point::new(coords[i + 1].x, coords[i + 1].y);

        // Project point onto segment
        let (closest, frac) = project_point_to_segment(point, p1, p2);
        let dist = point.haversine_distance(&closest);

        if dist < best_dist {
            best_dist = dist;
            // Calculate overall fraction
            let dist_along = seg_start + frac * seg_len;
            best_fraction = dist_along / total_length;
        }
    }

    best_fraction.clamp(0.0, 1.0)
}

/// Project a point onto a line segment, returns (closest_point, fraction_along_segment)
fn project_point_to_segment(
    point: Point<f64>,
    p1: Point<f64>,
    p2: Point<f64>,
) -> (Point<f64>, f64) {
    let dx = p2.x() - p1.x();
    let dy = p2.y() - p1.y();

    if dx == 0.0 && dy == 0.0 {
        return (p1, 0.0);
    }

    let t = ((point.x() - p1.x()) * dx + (point.y() - p1.y()) * dy) / (dx * dx + dy * dy);
    let t_clamped = t.clamp(0.0, 1.0);

    let closest = Point::new(p1.x() + t_clamped * dx, p1.y() + t_clamped * dy);
    (closest, t_clamped)
}

/// Get the bearing at the closest point on a line to a query point
/// Returns the bearing of the segment containing the closest point (0-360 degrees)
pub fn bearing_at_projection(point: Point<f64>, line: &LineString<f64>) -> f64 {
    use geo::GeodesicBearing;

    let coords: Vec<Coord<f64>> = line.coords().cloned().collect();
    if coords.len() < 2 {
        return 0.0;
    }

    // Find closest segment
    let mut best_dist = f64::MAX;
    let mut best_segment_idx = 0;

    for i in 0..coords.len() - 1 {
        let p1 = Point::new(coords[i].x, coords[i].y);
        let p2 = Point::new(coords[i + 1].x, coords[i + 1].y);
        let (closest, _) = project_point_to_segment(point, p1, p2);
        let dist = point.haversine_distance(&closest);

        if dist < best_dist {
            best_dist = dist;
            best_segment_idx = i;
        }
    }

    // Calculate bearing of the best segment
    let p1 = Point::new(coords[best_segment_idx].x, coords[best_segment_idx].y);
    let p2 = Point::new(
        coords[best_segment_idx + 1].x,
        coords[best_segment_idx + 1].y,
    );

    let bearing = p1.geodesic_bearing(p2);
    // Normalize to 0-360
    ((bearing % 360.0) + 360.0) % 360.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bearing_difference() {
        assert!((bearing_difference(10.0, 20.0) - 10.0).abs() < 0.01);
        assert!((bearing_difference(350.0, 10.0) - 20.0).abs() < 0.01);
        assert!((bearing_difference(180.0, 0.0) - 180.0).abs() < 0.01);
    }

    #[test]
    fn test_edge_envelope() {
        let line = LineString::from(vec![(0.0, 0.0), (1.0, 1.0)]);
        let env = EdgeEnvelope::new(EdgeIndex::new(0), line);

        assert_eq!(env.min_x, 0.0);
        assert_eq!(env.max_x, 1.0);
    }

    fn create_test_edges() -> Vec<EdgeEnvelope> {
        vec![
            EdgeEnvelope::new(
                EdgeIndex::new(0),
                LineString::from(vec![(0.0, 0.0), (0.001, 0.0)]),
            ),
            EdgeEnvelope::new(
                EdgeIndex::new(1),
                LineString::from(vec![(0.001, 0.0), (0.002, 0.0)]),
            ),
            EdgeEnvelope::new(
                EdgeIndex::new(2),
                LineString::from(vec![(1.0, 1.0), (1.001, 1.0)]),
            ),
        ]
    }

    #[test]
    fn test_spatial_index_mode_rtree() {
        let edges = create_test_edges();
        let index = SpatialIndex::with_mode(edges, SpatialIndexMode::RTree);

        assert_eq!(index.mode(), SpatialIndexMode::RTree);
        assert_eq!(index.len(), 3);
        assert!(!index.is_empty());
    }

    #[test]
    fn test_spatial_index_mode_linear_scan() {
        let edges = create_test_edges();
        let index = SpatialIndex::with_mode(edges, SpatialIndexMode::LinearScan);

        assert_eq!(index.mode(), SpatialIndexMode::LinearScan);
        assert_eq!(index.len(), 3);
        assert!(!index.is_empty());
    }

    #[test]
    fn test_spatial_index_modes_find_same_results() {
        let edges = create_test_edges();
        let rtree_index = SpatialIndex::with_mode(edges.clone(), SpatialIndexMode::RTree);
        let linear_index = SpatialIndex::with_mode(edges, SpatialIndexMode::LinearScan);

        // Query near the first two edges
        let center = Point::new(0.001, 0.0);
        let radius_m = 500.0; // ~500m should capture nearby edges

        let rtree_results = rtree_index.find_within_radius(center, radius_m);
        let linear_results = linear_index.find_within_radius(center, radius_m);

        // Both should find the same edges
        assert_eq!(
            rtree_results.len(),
            linear_results.len(),
            "R-tree found {} edges, linear scan found {}",
            rtree_results.len(),
            linear_results.len()
        );

        // Extract edge indices and sort for comparison
        let mut rtree_edges: Vec<_> = rtree_results.iter().map(|(e, _)| e.edge_idx).collect();
        let mut linear_edges: Vec<_> = linear_results.iter().map(|(e, _)| e.edge_idx).collect();
        rtree_edges.sort();
        linear_edges.sort();

        assert_eq!(
            rtree_edges, linear_edges,
            "Both modes should find the same edges"
        );
    }

    #[test]
    fn test_new_linear_scan_convenience() {
        let edges = create_test_edges();
        let index = SpatialIndex::new_linear_scan(edges);

        assert_eq!(index.mode(), SpatialIndexMode::LinearScan);
    }

    #[test]
    fn test_default_mode_is_rtree() {
        let edges = create_test_edges();
        let index = SpatialIndex::new(edges);

        assert_eq!(index.mode(), SpatialIndexMode::RTree);
    }
}
