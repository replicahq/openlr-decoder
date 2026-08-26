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

        let (min_lon, max_lon) = (center.x() - delta_lon, center.x() + delta_lon);
        let (min_lat, max_lat) = (center.y() - delta_lat, center.y() + delta_lat);

        // A box that pokes past ±180 must also search the far side of the
        // antimeridian, where those longitudes are actually stored.
        let mut boxes = vec![AABB::from_corners(
            [min_lon.max(-180.0), min_lat],
            [max_lon.min(180.0), max_lat],
        )];
        if min_lon < -180.0 {
            boxes.push(AABB::from_corners(
                [min_lon + 360.0, min_lat],
                [180.0, max_lat],
            ));
        }
        if max_lon > 180.0 {
            boxes.push(AABB::from_corners(
                [-180.0, min_lat],
                [max_lon - 360.0, max_lat],
            ));
        }

        let mut results: Vec<&EdgeEnvelope> = boxes
            .iter()
            .flat_map(|b| self.rtree.locate_in_envelope_intersecting(b))
            .collect();
        if boxes.len() > 1 {
            results.sort_by_key(|e| e.edge_idx);
            results.dedup_by_key(|e| e.edge_idx);
        }
        results
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

/// Wrap a longitude difference into [-180, 180] so arithmetic near the
/// antimeridian follows the short path across ±180 instead of through 0.
fn wrap_lon_delta(dx: f64) -> f64 {
    if dx > 180.0 {
        dx - 360.0
    } else if dx < -180.0 {
        dx + 360.0
    } else {
        dx
    }
}

/// Normalize a longitude into [-180, 180].
fn wrap_lon(x: f64) -> f64 {
    if x > 180.0 {
        x - 360.0
    } else if x < -180.0 {
        x + 360.0
    } else {
        x
    }
}

/// Project a point onto a line segment, returns (closest_point, fraction_along_segment).
/// Longitude deltas are wrapped so segments and query points near the antimeridian
/// project correctly.
fn project_point_to_segment(
    point: Point<f64>,
    p1: Point<f64>,
    p2: Point<f64>,
) -> (Point<f64>, f64) {
    let dx = wrap_lon_delta(p2.x() - p1.x());
    let dy = p2.y() - p1.y();

    if dx == 0.0 && dy == 0.0 {
        return (p1, 0.0);
    }

    let px = wrap_lon_delta(point.x() - p1.x());
    let t = (px * dx + (point.y() - p1.y()) * dy) / (dx * dx + dy * dy);
    let t_clamped = t.clamp(0.0, 1.0);

    let closest = Point::new(wrap_lon(p1.x() + t_clamped * dx), p1.y() + t_clamped * dy);
    (closest, t_clamped)
}

/// BEARDIST from the OpenLR spec (§5.2.4): bearing is measured over a 20 m chord.
pub const BEARDIST_M: f64 = 20.0;

/// Which side of the projection point the bearing chord is taken from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BearingDirection {
    /// Chord from the projection point to BEARDIST further along the line
    /// (first / intermediate LRPs, whose attributes describe the outgoing line).
    Forward,
    /// Chord from BEARDIST before the projection point up to it
    /// (last LRP, whose attributes describe the incoming line).
    Backward,
}

/// Bearing (0-360, in travel direction) of the line at the point closest to `point`,
/// measured over a BEARDIST chord per OpenLR §5.2.4 rather than a single segment.
/// If the line ends within BEARDIST in the requested direction the chord is clamped to
/// the line end; if that leaves no usable chord the other direction is tried, and a
/// degenerate line falls back to 0.0.
pub fn bearing_at_projection(
    point: Point<f64>,
    line: &LineString<f64>,
    direction: BearingDirection,
) -> f64 {
    use geo::GeodesicBearing;

    let coords: Vec<Coord<f64>> = line.coords().cloned().collect();
    if coords.len() < 2 {
        return 0.0;
    }

    // Cumulative distance at each vertex, and the projection's distance along the line.
    let mut cum = Vec::with_capacity(coords.len());
    cum.push(0.0);
    for i in 0..coords.len() - 1 {
        let d = Point::from(coords[i]).haversine_distance(&Point::from(coords[i + 1]));
        cum.push(cum[i] + d);
    }
    let total = *cum.last().unwrap();

    let mut best_dist = f64::MAX;
    let mut along = 0.0;
    for i in 0..coords.len() - 1 {
        let (closest, frac) =
            project_point_to_segment(point, Point::from(coords[i]), Point::from(coords[i + 1]));
        let dist = point.haversine_distance(&closest);
        if dist < best_dist {
            best_dist = dist;
            along = cum[i] + frac * (cum[i + 1] - cum[i]);
        }
    }

    let proj = point_at_distance(&coords, &cum, along);
    let chord = |dir: BearingDirection| -> Option<(Point<f64>, Point<f64>)> {
        let (a, b) = match dir {
            BearingDirection::Forward => (
                proj,
                point_at_distance(&coords, &cum, (along + BEARDIST_M).min(total)),
            ),
            BearingDirection::Backward => (
                point_at_distance(&coords, &cum, (along - BEARDIST_M).max(0.0)),
                proj,
            ),
        };
        (a.haversine_distance(&b) > 0.5).then_some((a, b))
    };
    let other = match direction {
        BearingDirection::Forward => BearingDirection::Backward,
        BearingDirection::Backward => BearingDirection::Forward,
    };
    let Some((a, b)) = chord(direction).or_else(|| chord(other)) else {
        return 0.0;
    };

    let bearing = a.geodesic_bearing(b);
    ((bearing % 360.0) + 360.0) % 360.0
}

/// Interpolate the point `dist` meters along a polyline with cumulative lengths `cum`.
fn point_at_distance(coords: &[Coord<f64>], cum: &[f64], dist: f64) -> Point<f64> {
    let dist = dist.clamp(0.0, *cum.last().unwrap());
    for i in 0..coords.len() - 1 {
        if dist <= cum[i + 1] {
            let seg = cum[i + 1] - cum[i];
            let t = if seg > 0.0 {
                (dist - cum[i]) / seg
            } else {
                0.0
            };
            // Wrap the longitude delta so segments crossing the antimeridian
            // (e.g. 179.999 -> -179.999) interpolate across ±180 rather than
            // the ~360° numeric path through longitude 0.
            let dx = wrap_lon_delta(coords[i + 1].x - coords[i].x);
            return Point::new(
                wrap_lon(coords[i].x + t * dx),
                coords[i].y + t * (coords[i + 1].y - coords[i].y),
            );
        }
    }
    Point::from(*coords.last().unwrap())
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

    // ~1e-4 deg lat ≈ 11 m; ~1e-4 deg lon ≈ 8.7 m at 39°N.
    // Line: 30 m due north, then 30 m due east.
    fn kinked_line() -> LineString<f64> {
        LineString::from(vec![
            (-94.5, 39.0),
            (-94.5, 39.00027),
            (-94.49965, 39.00027),
        ])
    }

    #[test]
    fn test_projection_across_antimeridian() {
        // Segment crossing 180°; query point on the west side of the line.
        let (closest, frac) = project_point_to_segment(
            Point::new(-179.99990, 0.0001),
            Point::new(179.99985, 0.0),
            Point::new(-179.99985, 0.0),
        );
        // 0.00025° of the 0.0003° span → t ≈ 0.833, at longitude ≈ -179.9999 (not near 0°).
        assert!((frac - 0.8333).abs() < 0.01, "frac {frac}");
        assert!(closest.x() < -179.999, "closest lon {}", closest.x());
    }

    #[test]
    fn test_find_nearby_across_antimeridian() {
        use crate::graph::{Edge, Fow, Frc, RoadNetwork};
        let mut net = RoadNetwork::new();
        net.get_or_add_node(1, Point::new(-179.9999, 0.0));
        net.get_or_add_node(2, Point::new(-179.999, 0.0));
        let geom = LineString::from(vec![(-179.9999, 0.0), (-179.999, 0.0)]);
        let edge_idx = net.add_edge(
            1,
            2,
            Edge::new(1, geom.clone(), Frc::Frc4, Fow::SingleCarriageway),
        );
        let idx = SpatialIndex::new(vec![EdgeEnvelope::new(edge_idx, &geom)]);
        // Query from just east of the line (179.9999°): the 100m box crosses ±180
        // and must still find the edge stored at -179.9999°.
        let hits = idx.find_nearby(Point::new(179.9999, 0.0), 100.0);
        assert_eq!(hits.len(), 1);
    }

    #[test]
    fn test_bearing_chord_across_antimeridian() {
        // ~35 m due-east edge crossing the antimeridian at the equator.
        let line = LineString::from(vec![(179.99985, 0.0), (-179.99985, 0.0)]);
        let p = Point::new(179.99995, 0.0); // ~11 m along, mid-edge
        let b = bearing_at_projection(p, &line, BearingDirection::Forward);
        assert!(
            (b - 90.0).abs() < 1.0,
            "expected ~90° (due east) across the antimeridian, got {b}"
        );
        let b = bearing_at_projection(p, &line, BearingDirection::Backward);
        assert!((b - 90.0).abs() < 1.0, "got {b}");
    }

    #[test]
    fn test_bearing_forward_chord_spans_kink() {
        // Project 15 m up the north leg: the next 20 m covers 15 m north + 5 m east,
        // so the chord bearing is NNE, not the segment's due-north.
        let p = Point::new(-94.5, 39.000135);
        let b = bearing_at_projection(p, &kinked_line(), BearingDirection::Forward);
        assert!(b > 10.0 && b < 25.0, "got {b}");
    }

    #[test]
    fn test_bearing_backward_chord_spans_kink() {
        // Project 15 m along the east leg: the previous 20 m covers 5 m north + 15 m east.
        let p = Point::new(-94.499826, 39.00027);
        let b = bearing_at_projection(p, &kinked_line(), BearingDirection::Backward);
        assert!(b > 65.0 && b < 80.0, "got {b}");
    }

    #[test]
    fn test_bearing_clamps_at_line_end() {
        // Near the end of the line, Forward has <20 m left; chord clamps to the end (east).
        let p = Point::new(-94.49970, 39.00027);
        let b = bearing_at_projection(p, &kinked_line(), BearingDirection::Forward);
        assert!((b - 90.0).abs() < 2.0, "got {b}");
        // At the very end, Forward has no chord and falls back to Backward.
        let p = Point::new(-94.49965, 39.00027);
        let b = bearing_at_projection(p, &kinked_line(), BearingDirection::Forward);
        assert!(b > 60.0 && b < 90.0, "got {b}");
    }

    #[test]
    fn test_bearing_straight_line_both_directions_agree() {
        let line = LineString::from(vec![(-94.5, 39.0), (-94.5, 39.001)]);
        let p = Point::new(-94.5001, 39.0005);
        let f = bearing_at_projection(p, &line, BearingDirection::Forward);
        let b = bearing_at_projection(p, &line, BearingDirection::Backward);
        assert!(f.abs() < 0.5 && b.abs() < 0.5, "got {f} {b}");
    }

    #[test]
    fn test_edge_envelope() {
        let line = LineString::from(vec![(0.0, 0.0), (1.0, 1.0)]);
        let env = EdgeEnvelope::new(EdgeIndex::new(0), &line);

        assert_eq!(env.min_x, 0.0);
        assert_eq!(env.max_x, 1.0);
    }
}
