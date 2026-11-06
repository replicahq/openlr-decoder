//! Edge lookup by location, and point-on-line geometry: projection, bearings
//! and positions along a polyline. Coordinates are WGS84 degrees (x = lon,
//! y = lat) and distances are meters.

use geo::{BoundingRect, Coord, Distance, Haversine, LineString, Point};
use petgraph::graph::EdgeIndex;
use rstar::{Envelope, RTree, RTreeObject, AABB};

/// Earth radius used by geo's [`Haversine`], in meters.
const EARTH_RADIUS_M: f64 = 6_371_008.8;

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

    /// Edges whose bounding box intersects the bounding box of the circle of
    /// `radius_m` around `center`. That includes every edge with a point within
    /// `radius_m` by [`Haversine`] distance, plus some further away, so callers
    /// filter by true distance.
    pub fn find_nearby(&self, center: Point<f64>, radius_m: f64) -> Vec<&EdgeEnvelope> {
        // Exact on the sphere Haversine uses: a circle of angular radius r around
        // latitude φ spans ±r in latitude and ±asin(sin r / cos φ) in longitude.
        let r = (radius_m / EARTH_RADIUS_M) * (1.0 + 1e-9);
        let delta_lat = r.to_degrees();
        let sin_ratio = r.sin() / center.y().to_radians().cos();
        let delta_lon = if sin_ratio < 1.0 {
            sin_ratio.asin().to_degrees()
        } else {
            180.0 // the circle reaches a pole
        };

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
    /// Meters from the point that was projected to [`point`](Self::point).
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
    // Segments are compared by the same scaled planar distance the projection
    // minimises; only the winner's distance is converted to meters.
    let k = point.y().to_radians().cos();
    let mut best = (Point::from(coords[0]), f64::MAX, 0.0);
    let mut length_m = 0.0;
    for w in coords.windows(2) {
        let (a, b) = (Point::from(w[0]), Point::from(w[1]));
        let (closest, t) = project_point_to_segment(point, a, b);
        let dx = wrap_lon(closest.x() - point.x()) * k;
        let dy = closest.y() - point.y();
        let seg_m = Haversine.distance(a, b);
        if dx * dx + dy * dy < best.1 {
            best = (closest, dx * dx + dy * dy, length_m + t * seg_m);
        }
        length_m += seg_m;
    }
    Some(LineProjection {
        point: best.0,
        distance_m: Haversine.distance(point, best.0),
        along_m: best.2,
        length_m,
    })
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
    let coords = &line.0;
    if coords.len() < 2 {
        return 0.0;
    }

    let [back, at, ahead] =
        points_at_distances(coords, [along_m - chord_m, along_m, along_m + chord_m]);
    let chord = |dir: BearingDirection| {
        let (a, b) = match dir {
            BearingDirection::Forward => (at, ahead),
            BearingDirection::Backward => (back, at),
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

    local_bearing(a, b)
}

/// Bearing in degrees [0, 360) from `a` to `b` on the WGS84 ellipsoid, treating
/// the short span between them as flat. Over 50 m it is within 0.002° of geo's
/// `Geodesic` up to 80° latitude, at a fraction of the cost.
fn local_bearing(a: Point<f64>, b: Point<f64>) -> f64 {
    // East and north meters per degree are N·cos(φ) and M, the prime vertical
    // and meridian radii of curvature. Only their ratio matters here:
    // N·cos(φ) / M = cos(φ)·(1 − e²·sin²φ) / (1 − e²).
    const E2: f64 = 6.694_379_990_141_316e-3; // WGS84 first eccentricity squared
    let phi = ((a.y() + b.y()) / 2.0).to_radians();
    let (sin_phi, cos_phi) = phi.sin_cos();
    let east_per_north = cos_phi * (1.0 - E2 * sin_phi * sin_phi) / (1.0 - E2);
    let east = wrap_lon(b.x() - a.x()) * east_per_north;
    let north = b.y() - a.y();
    east.atan2(north).to_degrees().rem_euclid(360.0)
}

/// The points at each of `dists` meters along a polyline, in one pass.
/// `dists` must be ascending; each is clamped to the line.
fn points_at_distances<const N: usize>(coords: &[Coord<f64>], dists: [f64; N]) -> [Point<f64>; N] {
    let mut out = [Point::from(*coords.last().unwrap()); N];
    let mut next = 0;
    let mut start_m = 0.0;
    for w in coords.windows(2) {
        let seg_m = Haversine.distance(Point::from(w[0]), Point::from(w[1]));
        while next < N && dists[next] <= start_m + seg_m {
            let t = if seg_m > 0.0 {
                ((dists[next] - start_m) / seg_m).max(0.0)
            } else {
                0.0
            };
            let dx = wrap_lon(w[1].x - w[0].x);
            out[next] = Point::new(wrap_lon(w[0].x + t * dx), w[0].y + t * (w[1].y - w[0].y));
            next += 1;
        }
        if next == N {
            break;
        }
        start_m += seg_m;
    }
    out
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
        // At the very end, Forward has no chord and falls back to Backward,
        // the last 20 m of the 30 m east leg.
        let p = Point::new(-94.49965, 39.00027);
        let b = bearing_at_projection(p, &kinked_line(), BearingDirection::Forward);
        assert!((b - 90.0).abs() < 0.01, "got {b}");
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
    fn test_local_bearing_matches_geodesic() {
        use geo::{Bearing, Destination, Geodesic};
        for lat in [0.0, 39.0, 60.0, -45.0, 80.0] {
            for deg in (0..360).step_by(15) {
                let a = Point::new(-94.5, lat);
                let b = Geodesic.destination(a, deg as f64, 50.0);
                let got = local_bearing(a, b);
                let diff = bearing_difference(got, Geodesic.bearing(a, b));
                assert!(diff < 0.002, "lat {lat} bearing {deg}: {got} off by {diff}");
            }
        }
    }

    #[test]
    fn test_find_nearby_box_holds_the_circle() {
        use geo::Destination;
        // A tiny edge at every point on a circle of the query radius, at several
        // latitudes, must be found. Distances just inside the radius are the
        // ones an under-sized box would miss.
        for lat in [0.0, 39.0, 70.0, -85.0] {
            let center = Point::new(10.0, lat);
            let mut envs = Vec::new();
            for (i, deg) in (0..360).step_by(5).enumerate() {
                let p = Haversine.destination(center, deg as f64, 99.999);
                let line = LineString::from(vec![p, p]);
                envs.push(EdgeEnvelope::new(EdgeIndex::new(i), &line));
            }
            let n = envs.len();
            let idx = SpatialIndex::new(envs);
            assert_eq!(idx.find_nearby(center, 100.0).len(), n, "lat {lat}");
        }
    }

    #[test]
    fn test_points_at_distances_one_pass() {
        // kinked_line: 30 m north, then 30 m east.
        let coords = &kinked_line().0;
        let [before, mid, kink, after] = points_at_distances(coords, [-5.0, 15.0, 30.0, 100.0]);
        assert_eq!(before, Point::from(coords[0]));
        assert!(
            (mid.y() - 39.000135).abs() < 1e-6 && mid.x() == -94.5,
            "{mid:?}"
        );
        assert!((kink.y() - 39.00027).abs() < 1e-6, "{kink:?}");
        assert_eq!(after, Point::from(coords[2]));
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
