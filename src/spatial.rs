//! Edge lookup by location, and point-on-line geometry: projection, bearings
//! and positions along a polyline. Coordinates are WGS84 degrees (x = lon,
//! y = lat) and distances are meters.

use geo::{BoundingRect, Coord, Distance, Haversine, LineString, Point};
use petgraph::graph::EdgeIndex;
use rstar::{Envelope, RTree, RTreeObject, AABB};

/// An edge's bounding box, in degrees. The geometry stays in the graph's
/// [`Edge`](crate::graph::Edge) so it isn't stored twice.
#[derive(Debug, Clone)]
pub struct EdgeEnvelope {
    pub edge_idx: EdgeIndex,
    envelope: AABB<[f64; 2]>,
}

impl EdgeEnvelope {
    /// An empty `geometry` gets an empty box, which no query intersects.
    pub fn new(edge_idx: EdgeIndex, geometry: &LineString<f64>) -> Self {
        let envelope = match geometry.bounding_rect() {
            Some(r) => AABB::from_corners(r.min().into(), r.max().into()),
            None => AABB::new_empty(),
        };
        EdgeEnvelope { edge_idx, envelope }
    }
}

impl RTreeObject for EdgeEnvelope {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

pub struct SpatialIndex {
    rtree: RTree<EdgeEnvelope>,
}

impl SpatialIndex {
    pub fn new(edges: Vec<EdgeEnvelope>) -> Self {
        SpatialIndex {
            rtree: RTree::bulk_load(edges),
        }
    }

    /// Edges whose bounding box intersects a box extending `radius_m` from
    /// `center` in each direction. The meters-to-degrees conversion is approximate,
    /// so callers needing an exact radius should pad it and filter by true distance.
    pub fn find_nearby(&self, center: Point<f64>, radius_m: f64) -> Vec<&EdgeEnvelope> {
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
            .flat_map(|b| self.rtree.locate_in_envelope_intersecting(*b))
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

/// Smallest angle between two bearings, in [0, 180]. Inputs must be in [0, 360).
pub fn bearing_difference(b1: f64, b2: f64) -> f64 {
    let diff = (b1 - b2).abs();
    if diff > 180.0 {
        360.0 - diff
    } else {
        diff
    }
}

/// Where a point projects onto a line.
#[derive(Debug, Clone, Copy)]
pub struct LineProjection {
    /// The closest point on the line.
    pub point: Point<f64>,
    /// Meters from the projected point to [`point`](Self::point).
    pub distance_m: f64,
    /// Meters along the line from its start to the closest point.
    pub along_m: f64,
    pub length_m: f64,
}

impl LineProjection {
    /// [`along_m`](Self::along_m) as a fraction of the length, in [0, 1]; 0.0 for
    /// a zero-length line.
    pub fn fraction(&self) -> f64 {
        if self.length_m > 0.0 {
            (self.along_m / self.length_m).clamp(0.0, 1.0)
        } else {
            0.0
        }
    }
}

/// Project `point` onto the nearest segment of `line`. `None` if the line has
/// fewer than two coordinates.
pub fn project_onto_line(point: Point<f64>, line: &LineString<f64>) -> Option<LineProjection> {
    let coords = &line.0;
    if coords.len() < 2 {
        return None;
    }
    let cum = cumulative_lengths(coords);

    let mut best = LineProjection {
        point: Point::from(coords[0]),
        distance_m: f64::MAX,
        along_m: 0.0,
        length_m: *cum.last().unwrap(),
    };
    for i in 0..coords.len() - 1 {
        let (closest, frac) =
            project_point_to_segment(point, Point::from(coords[i]), Point::from(coords[i + 1]));
        let dist = Haversine.distance(point, closest);
        if dist < best.distance_m {
            best.point = closest;
            best.distance_m = dist;
            best.along_m = cum[i] + frac * (cum[i + 1] - cum[i]);
        }
    }
    Some(best)
}

/// Meters along the line at each vertex.
fn cumulative_lengths(coords: &[Coord<f64>]) -> Vec<f64> {
    let mut cum = Vec::with_capacity(coords.len());
    cum.push(0.0);
    for w in coords.windows(2) {
        let d = Haversine.distance(Point::from(w[0]), Point::from(w[1]));
        cum.push(cum.last().unwrap() + d);
    }
    cum
}

/// Wrap a longitude, or a longitude difference, into [-180, 180]. Applied to
/// differences it makes arithmetic near the antimeridian take the short way
/// across ±180 instead of through 0.
fn wrap_lon(x: f64) -> f64 {
    if x > 180.0 {
        x - 360.0
    } else if x < -180.0 {
        x + 360.0
    } else {
        x
    }
}

/// Closest point on segment `p1`–`p2` and its fraction along the segment.
/// Planar, with longitude scaled by cos(latitude) so a degree counts the same
/// in both axes, and wrapped via [`wrap_lon`]. Over a road segment the error
/// against a great-circle projection is far below coordinate precision.
fn project_point_to_segment(
    point: Point<f64>,
    p1: Point<f64>,
    p2: Point<f64>,
) -> (Point<f64>, f64) {
    let k = point.y().to_radians().cos();
    let dx = wrap_lon(p2.x() - p1.x());
    let dy = p2.y() - p1.y();

    if dx == 0.0 && dy == 0.0 {
        return (p1, 0.0);
    }

    let px = wrap_lon(point.x() - p1.x());
    let py = point.y() - p1.y();
    let t = (px * dx * k * k + py * dy) / (dx * dx * k * k + dy * dy);
    let t = t.clamp(0.0, 1.0);

    let closest = Point::new(wrap_lon(p1.x() + t * dx), p1.y() + t * dy);
    (closest, t)
}

/// Which side of a point on a line [`bearing_along`] takes its chord from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BearingDirection {
    /// The stretch after the point.
    Forward,
    /// The stretch leading up to the point.
    Backward,
}

/// Bearing in degrees [0, 360), in the line's direction, of the `chord_m`
/// chord on the `direction` side of the point `along_m` meters along `line`
/// (see [`LineProjection`]). The chord is cut short at the line's end; if that
/// leaves under 0.5 m, the other side is used, and if both are that short the
/// result is 0.0. A chord smooths over vertex noise that a single segment's
/// bearing would pick up.
pub fn bearing_along(
    line: &LineString<f64>,
    along_m: f64,
    chord_m: f64,
    direction: BearingDirection,
) -> f64 {
    use geo::{Bearing, Geodesic};

    let coords = &line.0;
    if coords.len() < 2 {
        return 0.0;
    }
    let cum = cumulative_lengths(coords);
    let total = *cum.last().unwrap();

    let proj = point_at_distance(coords, &cum, along_m);
    let chord = |dir: BearingDirection| -> Option<(Point<f64>, Point<f64>)> {
        let (a, b) = match dir {
            BearingDirection::Forward => (
                proj,
                point_at_distance(coords, &cum, (along_m + chord_m).min(total)),
            ),
            BearingDirection::Backward => (
                point_at_distance(coords, &cum, (along_m - chord_m).max(0.0)),
                proj,
            ),
        };
        (Haversine.distance(a, b) > 0.5).then_some((a, b))
    };
    let other = match direction {
        BearingDirection::Forward => BearingDirection::Backward,
        BearingDirection::Backward => BearingDirection::Forward,
    };
    let Some((a, b)) = chord(direction).or_else(|| chord(other)) else {
        return 0.0;
    };

    Geodesic.bearing(a, b)
}

/// The point `dist` meters along a polyline whose vertices are `cum` meters along.
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
            let dx = wrap_lon(coords[i + 1].x - coords[i].x);
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

    fn bearing_at_projection(p: Point<f64>, line: &LineString<f64>, dir: BearingDirection) -> f64 {
        bearing_along(line, project_onto_line(p, line).unwrap().along_m, 20.0, dir)
    }

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
        // Segment crossing 180°; query point just north of it, east of 180°.
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
    fn test_projection_scales_longitude_by_latitude() {
        // At 60°N a degree of longitude is half a degree of latitude, so this
        // segment runs at 45° NE. A point 55.6 m due north of its start projects
        // 55.6·cos45° = 39.3 m along, a quarter of its 157 m. Unscaled degrees
        // would put it at 0.1.
        let line = LineString::from(vec![(0.0, 60.0), (0.002, 60.001)]);
        let proj = project_onto_line(Point::new(0.0, 60.0005), &line).unwrap();
        assert!((proj.fraction() - 0.25).abs() < 0.005, "{proj:?}");
        assert!((proj.distance_m - 39.3).abs() < 0.5, "{proj:?}");
        // A quarter of the way from (0, 60) to (0.002, 60.001).
        assert!((proj.point.x() - 0.0005).abs() < 1e-5, "{proj:?}");
        assert!((proj.point.y() - 60.00025).abs() < 1e-5, "{proj:?}");
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
        // Query from 179.9999°, ~11 m across 180° from the edge stored at
        // -179.9999°. The 100 m box crosses ±180 and must still find it.
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
        let env = EdgeEnvelope::new(EdgeIndex::new(0), &line).envelope();
        assert_eq!(env.lower(), [0.0, 0.0]);
        assert_eq!(env.upper(), [1.0, 1.0]);

        let empty = EdgeEnvelope::new(EdgeIndex::new(0), &LineString::new(vec![]));
        let idx = SpatialIndex::new(vec![empty]);
        assert!(idx.find_nearby(Point::new(0.0, 0.0), 1e6).is_empty());
    }
}
