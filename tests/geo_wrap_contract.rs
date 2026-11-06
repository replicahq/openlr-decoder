//! Contract tests: the geo-crate operations we rely on must be wrap-safe across
//! the antimeridian (the USA network includes Aleutian islands east of 170°E).
//! Our own longitude arithmetic is covered in src/spatial.rs tests; these guard
//! against a geo upgrade regressing the library half of that assumption.
use geo::{
    Bearing, Closest, Distance, Geodesic, Haversine, HaversineClosestPoint, LineString, Point,
};

#[test]
fn geo_ops_are_wrap_safe_across_antimeridian() {
    let line = LineString::from(vec![(179.9998, 0.0), (-179.9998, 0.0)]); // ~44m across 180
    let p = Point::new(-179.9999, 0.0001); // ~11m from the line's east part
    match line.haversine_closest_point(&p) {
        Closest::SinglePoint(c) | Closest::Intersection(c) => {
            let d = Haversine.distance(p, c);
            println!("closest=({}, {}) dist={:.2}m", c.x(), c.y(), d);
            assert!(
                d < 20.0,
                "HaversineClosestPoint NOT wrap-safe: dist {d:.1}m"
            );
        }
        Closest::Indeterminate => panic!("indeterminate"),
    }
    let b = Geodesic.bearing(Point::new(179.9998, 0.0), Point::new(-179.9998, 0.0));
    println!("geodesic_bearing across 180: {b:.2}");
    assert!(
        (b - 90.0).abs() < 1.0,
        "geodesic_bearing NOT wrap-safe: {b:.1}"
    );
    let d = Haversine.distance(Point::new(179.9998, 0.0), Point::new(-179.9998, 0.0));
    println!("haversine across 180: {d:.1}m");
    assert!(d < 100.0, "haversine_distance NOT wrap-safe: {d:.0}m");
}

/// Paths that existed before the road network moved to its own crate.
#[test]
fn test_old_module_paths_still_resolve() {
    #[allow(unused_imports)]
    use openlr_decoder::graph::{Edge, Fow, Frc, RoadNetwork};
    #[allow(unused_imports)]
    use openlr_decoder::loader::{load_network_from_parquet, load_openlr_codes, NetworkBuilder};
    #[allow(unused_imports)]
    use openlr_decoder::spatial::{project_onto_line, SpatialIndex};
}
