pub mod batch;
pub mod candidates;
pub mod codes;
pub mod decoder;

// The road network lives in the `road-network` crate; these keep the
// `openlr_decoder::graph` etc. paths working.
pub use road_network::{graph, loader, road_class, routing, spatial};

#[cfg(any(test, feature = "test-utils"))]
pub use road_network::test_utils;

#[cfg(feature = "python")]
mod python;

pub use batch::{decode_batch, BatchLayout};
pub use candidates::{find_candidates, Candidate, CandidateConfig};
pub use codes::load_openlr_codes;
pub use decoder::{DecodeError, DecodedPath, Decoder, DecoderConfig};
pub use road_network::{
    load_network_from_parquet, road_network_schema, BearingDirection, Edge, Fow, Frc, Node,
    RoadNetwork, RoadNetworkBuilder, ShortestPaths, SpatialIndex,
};
