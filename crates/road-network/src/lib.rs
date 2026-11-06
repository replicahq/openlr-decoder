//! A directed road graph loaded from parquet, with an R-tree over edge
//! geometries and bounded shortest-path search. Coordinates are WGS84 degrees
//! and distances are meters.

pub mod graph;
pub mod loader;
pub mod road_class;
pub mod routing;
pub mod spatial;

#[cfg(any(test, feature = "test-utils"))]
pub mod test_utils;

pub use graph::{Edge, Node, RoadNetwork, RoadNetworkBuilder};
pub use loader::{load_network_from_parquet, road_network_schema};
pub use road_class::{Fow, Frc};
pub use routing::ShortestPaths;
pub use spatial::{BearingDirection, SpatialIndex};
