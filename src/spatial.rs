use geo::{Coord, HaversineDistance, LineString, Point};
use petgraph::graph::EdgeIndex;
use rstar::{RTree, RTreeObject, AABB};

/// Envelope wrapper for an edge's bounding box in the R-tree.
///
/// This struct intentionally does NOT store the edge geometry to avoid
/// duplicating it (the canonical copy lives in `Edge` in the graph).
/// Precise distance calculations look up geometry via `network.edge()`.
#[derive(Debug, Clone)]
pub struct EdgeEnvelope {
    pub edge_idx: EdgeIndex,
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

impl EdgeEnvelope {
    pub fn new(edge_idx: EdgeIndex, geometry: &LineString<f64>) -> Self {
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

        for c in geometry.coords() {
            min_x = min_x.min(c.x);
            min_y = min_y.min(c.y);
            max_x = max_x.max(c.x);
            max_y = max_y.max(c.y);
        }

        EdgeEnvelope {
            edge_idx,
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }
}

impl RTreeObject for EdgeEnvelope {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners([self.min_x, self.min_y], [self.max_x, self.max_y])
    }
}

/// Spatial index for fast edge lookup
pub struct SpatialIndex {
    rtree: RTree<EdgeEnvelope>,
}

impl SpatialIndex {
    /// Build a new spatial index from edge envelopes
    pub fn new(edges: Vec<EdgeEnvelope>) -> Self {
        SpatialIndex {
            rtree: RTree::bulk_load(edges),
        }
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

        self.rtree
            .locate_in_envelope_intersecting(&search_box)
            .collect()
    }

    pub fn len(&self) -> usize {
        self.rtree.size()
    }

    pub fn is_empty(&self) -> bool {
        self.rtree.size() == 0
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
        let env = EdgeEnvelope::new(EdgeIndex::new(0), &line);

        assert_eq!(env.min_x, 0.0);
        assert_eq!(env.max_x, 1.0);
    }
}
