//! Python bindings for openlr_decoder using PyO3

use std::path::Path;
use std::sync::Arc;

use arrow::array::{Array, AsArray};
use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::Py;
use pyo3_arrow::input::AnyArray;
use pyo3_arrow::{PyRecordBatch, PyRecordBatchReader, PySchema};

use crate::batch::{self, decode_error_message, BatchLayout};
use crate::candidates::CandidateConfig;
use crate::decoder::{DecodedPath, Decoder, DecoderConfig};
use crate::graph::RoadNetwork;
use crate::loader::{load_network_from_parquet, NetworkBuilder};
use crate::spatial::SpatialIndex;

/// Python-exposed road network loaded from parquet
#[pyclass(name = "RoadNetwork")]
pub struct PyRoadNetwork {
    network: Arc<RoadNetwork>,
    spatial: Arc<SpatialIndex>,
}

#[pymethods]
impl PyRoadNetwork {
    /// Load a road network from a parquet file
    ///
    /// Args:
    ///     path: Path to the parquet file containing the road network
    ///
    /// Returns:
    ///     RoadNetwork: The loaded road network ready for decoding
    #[staticmethod]
    #[pyo3(signature = (path))]
    fn from_parquet(py: Python<'_>, path: &str) -> PyResult<Self> {
        // Parquet I/O and graph construction touch no Python objects, so run
        // them with the GIL released.
        let (network, spatial) = py
            .detach(|| load_network_from_parquet(Path::new(path)))
            .map_err(|e| PyValueError::new_err(format!("Failed to load network: {}", e)))?;

        Ok(PyRoadNetwork {
            network: Arc::new(network),
            spatial: Arc::new(spatial),
        })
    }

    /// Load a road network from Arrow data (zero-copy).
    ///
    /// This method accepts any Arrow-compatible data source via the Arrow PyCapsule
    /// interface, including:
    /// - PyArrow Table or RecordBatch
    /// - PyArrow RecordBatchReader
    /// - Polars DataFrame (zero-copy via Arrow)
    /// - Any object implementing __arrow_c_stream__ or __arrow_c_array__
    ///
    /// Args:
    ///     data: Arrow-compatible data with the road network schema.
    ///           Must have columns: stableEdgeId (uint64), startOsmNode (int64),
    ///           endOsmNode (int64), startLat/startLon/endLat/endLon (float64),
    ///           highway (string). Optional: lanes (int64), geometry (binary/WKB).
    ///
    /// Returns:
    ///     RoadNetwork: The loaded road network ready for decoding
    ///
    /// Example:
    ///     >>> import pyarrow as pa
    ///     >>> table = pa.parquet.read_table("network.parquet")
    ///     >>> network = RoadNetwork.from_arrow(table)
    ///
    ///     >>> import polars as pl
    ///     >>> df = pl.read_parquet("network.parquet")
    ///     >>> network = RoadNetwork.from_arrow(df)  # Zero-copy!
    #[staticmethod]
    #[pyo3(signature = (data))]
    fn from_arrow(py: Python<'_>, data: PyRecordBatchReader) -> PyResult<Self> {
        // Convert to arrow-rs RecordBatchReader via pyo3-arrow (zero-copy)
        let reader = data
            .into_reader()
            .map_err(|e| PyValueError::new_err(format!("Failed to read Arrow data: {}", e)))?;

        // Pull batches with the GIL held — the stream's producer may be implemented
        // in Python — but release it for the graph construction, which is the
        // expensive part and touches no Python objects. Each batch is dropped before
        // the next is fetched, so a streaming reader is still consumed incrementally.
        let mut builder = NetworkBuilder::default();
        for batch in reader {
            let batch = batch
                .map_err(|e| PyValueError::new_err(format!("Failed to read Arrow data: {}", e)))?;
            py.detach(|| builder.push_batch(&batch)).map_err(|e| {
                PyValueError::new_err(format!("Failed to build network from Arrow data: {}", e))
            })?;
        }
        let (network, spatial) = py.detach(|| builder.finish());

        Ok(PyRoadNetwork {
            network: Arc::new(network),
            spatial: Arc::new(spatial),
        })
    }

    /// Get the number of edges in the network
    #[getter]
    fn edge_count(&self) -> usize {
        self.network.graph.edge_count()
    }

    /// Get the number of nodes in the network
    #[getter]
    fn node_count(&self) -> usize {
        self.network.graph.node_count()
    }
}

/// Configuration for the OpenLR decoder
#[pyclass(name = "DecoderConfig")]
#[derive(Clone)]
pub struct PyDecoderConfig {
    /// Search radius in meters for finding candidate edges
    #[pyo3(get, set)]
    pub search_radius_m: f64,
    /// Maximum bearing difference in degrees
    #[pyo3(get, set)]
    pub max_bearing_diff: f64,
    /// FRC tolerance (0-7)
    #[pyo3(get, set)]
    pub frc_tolerance: u8,
    /// Maximum number of candidates per LRP
    #[pyo3(get, set)]
    pub max_candidates: usize,
    /// Maximum distance from LRP to candidate edge in meters
    #[pyo3(get, set)]
    pub max_candidate_distance_m: f64,
    /// Path length tolerance as a fraction (e.g., 0.20 = 20%)
    #[pyo3(get, set)]
    pub length_tolerance: f64,
    /// Absolute path length tolerance in meters
    #[pyo3(get, set)]
    pub absolute_length_tolerance: f64,
    /// Maximum search distance factor
    #[pyo3(get, set)]
    pub max_search_distance_factor: f64,
    /// Scoring weight for distance (higher = distance matters more)
    #[pyo3(get, set)]
    pub distance_weight: f64,
    /// Scoring weight for bearing difference
    #[pyo3(get, set)]
    pub bearing_weight: f64,
    /// Scoring weight for FRC difference
    #[pyo3(get, set)]
    pub frc_weight: f64,
    /// Scoring weight for FOW compatibility
    #[pyo3(get, set)]
    pub fow_weight: f64,
    /// Cost penalty in meters for traversing slip roads (ramps/links) in A* search.
    /// Makes decoder prefer main roads over slip roads. Default 20m.
    #[pyo3(get, set)]
    pub slip_road_cost_penalty: f64,
    /// Cost penalty in meters for traversing access/local roads (residential, living_street,
    /// service, track) in A* search. Makes decoder prefer tertiary over residential. Default 10m.
    #[pyo3(get, set)]
    pub access_road_cost_penalty: f64,
    /// Extend decoded paths so they start/end on valid (junction) nodes per OpenLR
    /// Rule 4, folding the extra length into the offsets. Default True.
    #[pyo3(get, set)]
    pub snap_to_valid_nodes: bool,
    /// Cap in meters on a terminal-node snap at each end. Default 25m.
    #[pyo3(get, set)]
    pub max_snap_extension_m: f64,
}

#[pymethods]
impl PyDecoderConfig {
    /// Create a new decoder configuration
    /// All parameters are optional and default to values optimized for cross-provider decoding
    #[new]
    #[pyo3(signature = (
        search_radius_m = 100.0,
        max_bearing_diff = 90.0,
        frc_tolerance = 2,
        max_candidates = 10,
        max_candidate_distance_m = 35.0,
        length_tolerance = 0.35,
        absolute_length_tolerance = 100.0,
        max_search_distance_factor = 2.0,
        distance_weight = 10.0,
        bearing_weight = 0.2,
        frc_weight = 0.1,
        fow_weight = 0.1,
        slip_road_cost_penalty = 20.0,
        access_road_cost_penalty = 10.0,
        snap_to_valid_nodes = true,
        max_snap_extension_m = 25.0
    ))]
    fn new(
        search_radius_m: f64,
        max_bearing_diff: f64,
        frc_tolerance: u8,
        max_candidates: usize,
        max_candidate_distance_m: f64,
        length_tolerance: f64,
        absolute_length_tolerance: f64,
        max_search_distance_factor: f64,
        distance_weight: f64,
        bearing_weight: f64,
        frc_weight: f64,
        fow_weight: f64,
        slip_road_cost_penalty: f64,
        access_road_cost_penalty: f64,
        snap_to_valid_nodes: bool,
        max_snap_extension_m: f64,
    ) -> Self {
        PyDecoderConfig {
            search_radius_m,
            max_bearing_diff,
            frc_tolerance,
            max_candidates,
            max_candidate_distance_m,
            length_tolerance,
            absolute_length_tolerance,
            max_search_distance_factor,
            distance_weight,
            bearing_weight,
            frc_weight,
            fow_weight,
            slip_road_cost_penalty,
            access_road_cost_penalty,
            snap_to_valid_nodes,
            max_snap_extension_m,
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "DecoderConfig(search_radius_m={}, max_bearing_diff={}, frc_tolerance={}, \
             max_candidates={}, max_candidate_distance_m={}, length_tolerance={}, \
             absolute_length_tolerance={}, max_search_distance_factor={}, \
             distance_weight={}, bearing_weight={}, frc_weight={}, fow_weight={}, \
             slip_road_cost_penalty={}, access_road_cost_penalty={}, \
             snap_to_valid_nodes={}, max_snap_extension_m={})",
            self.search_radius_m,
            self.max_bearing_diff,
            self.frc_tolerance,
            self.max_candidates,
            self.max_candidate_distance_m,
            self.length_tolerance,
            self.absolute_length_tolerance,
            self.max_search_distance_factor,
            self.distance_weight,
            self.bearing_weight,
            self.frc_weight,
            self.fow_weight,
            self.slip_road_cost_penalty,
            self.access_road_cost_penalty,
            self.snap_to_valid_nodes,
            self.max_snap_extension_m
        )
    }
}

impl From<&PyDecoderConfig> for DecoderConfig {
    fn from(config: &PyDecoderConfig) -> Self {
        DecoderConfig {
            candidate_config: CandidateConfig {
                search_radius_m: config.search_radius_m,
                max_bearing_diff: config.max_bearing_diff,
                frc_tolerance: config.frc_tolerance,
                max_candidates: config.max_candidates,
                max_candidate_distance_m: config.max_candidate_distance_m,
                distance_weight: config.distance_weight,
                bearing_weight: config.bearing_weight,
                frc_weight: config.frc_weight,
                fow_weight: config.fow_weight,
            },
            length_tolerance: config.length_tolerance,
            absolute_length_tolerance: config.absolute_length_tolerance,
            max_search_distance_factor: config.max_search_distance_factor,
            slip_road_cost_penalty: config.slip_road_cost_penalty,
            access_road_cost_penalty: config.access_road_cost_penalty,
            snap_to_valid_nodes: config.snap_to_valid_nodes,
            max_snap_extension_m: config.max_snap_extension_m,
        }
    }
}

/// Result of decoding an OpenLR reference
#[pyclass(name = "DecodedPath")]
pub struct PyDecodedPath {
    /// Stable edge IDs forming the path
    #[pyo3(get)]
    pub edge_ids: Vec<u64>,
    /// Total path length in meters
    #[pyo3(get)]
    pub length: f64,
    /// Distance from start of first edge to first LRP projection (trim from start) in meters
    #[pyo3(get)]
    pub positive_offset: f64,
    /// Distance from last LRP projection to end of last edge (trim from end) in meters
    #[pyo3(get)]
    pub negative_offset: f64,
    /// Fraction along first edge where first LRP projects (0.0-1.0)
    #[pyo3(get)]
    pub positive_offset_fraction: f64,
    /// Fraction along last edge where last LRP projects (0.0-1.0, measured from end)
    #[pyo3(get)]
    pub negative_offset_fraction: f64,
    /// The edge ID that covers the most distance in the decoded path
    #[pyo3(get)]
    pub primary_edge_id: u64,
    /// The distance covered by the primary edge in meters
    #[pyo3(get)]
    pub primary_edge_coverage: f64,
}

#[pymethods]
impl PyDecodedPath {
    fn __repr__(&self) -> String {
        format!(
            "DecodedPath(edge_ids=[...{} edges], length={:.1}m, positive_offset={:.1}m, negative_offset={:.1}m, primary_edge_id={}, primary_edge_coverage={:.1}m)",
            self.edge_ids.len(),
            self.length,
            self.positive_offset,
            self.negative_offset,
            self.primary_edge_id,
            self.primary_edge_coverage
        )
    }
}

impl From<DecodedPath> for PyDecodedPath {
    fn from(path: DecodedPath) -> Self {
        PyDecodedPath {
            edge_ids: path.edge_ids,
            length: path.length_m,
            positive_offset: path.positive_offset_m,
            negative_offset: path.negative_offset_m,
            positive_offset_fraction: path.positive_offset_fraction,
            negative_offset_fraction: path.negative_offset_fraction,
            primary_edge_id: path.primary_edge_id,
            primary_edge_coverage: path.primary_edge_coverage_m,
        }
    }
}

/// OpenLR decoder for resolving location references to map paths
#[pyclass(name = "Decoder")]
pub struct PyDecoder {
    network: Arc<RoadNetwork>,
    spatial: Arc<SpatialIndex>,
    config: DecoderConfig,
}

#[pymethods]
impl PyDecoder {
    /// Create a new decoder for the given road network
    ///
    /// Args:
    ///     network: The road network to decode against
    ///     config: Optional decoder configuration
    #[new]
    #[pyo3(signature = (network, config = None))]
    fn new(network: &PyRoadNetwork, config: Option<PyDecoderConfig>) -> Self {
        let decoder_config = config.as_ref().map(DecoderConfig::from).unwrap_or_default();

        PyDecoder {
            network: Arc::clone(&network.network),
            spatial: Arc::clone(&network.spatial),
            config: decoder_config,
        }
    }

    /// Decode an OpenLR base64 string to a path
    ///
    /// Args:
    ///     openlr_base64: Base64-encoded OpenLR location reference
    ///
    /// Returns:
    ///     DecodedPath: The decoded path with edge IDs and length
    ///
    /// Raises:
    ///     ValueError: If decoding fails
    fn decode(&self, py: Python<'_>, openlr_base64: &str) -> PyResult<PyDecodedPath> {
        // Decoding is pure Rust; release the GIL so other threads can run.
        py.detach(|| {
            let decoder =
                Decoder::new(&self.network, &self.spatial).with_config(self.config.clone());
            decoder.decode(openlr_base64)
        })
        .map(PyDecodedPath::from)
        .map_err(|e| PyValueError::new_err(decode_error_message(e)))
    }

    /// Decode multiple OpenLR strings in parallel
    ///
    /// Args:
    ///     openlr_codes: base64-encoded OpenLR location references, either as a
    ///         list of str or as any Arrow string array (``string``,
    ///         ``large_string`` or ``string_view``) exposed through the Arrow
    ///         PyCapsule interface — a PyArrow ``Array``/``ChunkedArray`` or a
    ///         Polars ``Series``. Arrow input is read zero-copy; null entries
    ///         are reported as errors.
    ///     explode: if True, return one row per (code, edge) instead of one row
    ///         per code (see below).
    ///
    /// Returns:
    ///     PyArrow RecordBatch. With ``explode=False`` (default), one row per code:
    ///     - edge_ids: list<uint64> - edge IDs for each decoded path
    ///     - length: float64 - path length in meters
    ///     - positive_offset: float64 - distance from first edge start to first LRP projection (meters)
    ///     - negative_offset: float64 - distance from last LRP projection to last edge end (meters)
    ///     - positive_offset_fraction: float64 - fraction along first edge where first LRP projects (0-1)
    ///     - negative_offset_fraction: float64 - fraction along last edge from last LRP to edge end (0-1)
    ///     - primary_edge_id: uint64 - edge ID covering the most distance
    ///     - primary_edge_coverage: float64 - distance covered by primary edge in meters
    ///     - error: string (nullable) - error message if decode failed, null if succeeded
    ///
    ///     With ``explode=True``, one row per edge of each decoded path, and a
    ///     single row with null edge columns for each failed code:
    ///     - code_index: uint64 - position of the code in ``openlr_codes``
    ///     - edge_id: uint64 - edge ID
    ///     - edge_position: uint32 - 0-based position of the edge in the path
    ///     - is_primary: bool - whether this is the path's primary edge
    ///     - offset_fraction: float64 - fraction (0-1) of this edge outside the
    ///       location; the meter offsets are spread from each end of the path,
    ///       so edges added by a terminal-node snap are 1.0
    ///     - length: float64 - path length in meters
    ///     - error: string (nullable) - error message if decode failed
    #[pyo3(signature = (openlr_codes, *, explode = false))]
    fn decode_batch(
        &self,
        py: Python<'_>,
        openlr_codes: CodesInput,
        explode: bool,
    ) -> PyResult<Py<PyAny>> {
        let layout = if explode {
            BatchLayout::Exploded
        } else {
            BatchLayout::Nested
        };
        let decoder = Decoder::new(&self.network, &self.spatial).with_config(self.config.clone());

        // Decoding and Arrow building touch no Python objects, so release the GIL.
        let batch = match openlr_codes {
            CodesInput::List(codes) => {
                let n = codes.len();
                py.detach(|| {
                    batch::decode_batch(&decoder, codes.iter().map(|c| Some(c.as_str())), layout, n)
                })
            }
            CodesInput::Arrow(input) => {
                // Pull the chunks with the GIL held (the stream may be produced in
                // Python); importing them is zero-copy.
                let chunks = input
                    .into_chunked_array()
                    .map_err(|e| {
                        PyValueError::new_err(format!("Failed to read Arrow data: {}", e))
                    })?
                    .into_inner()
                    .0;
                let n = chunks.iter().map(|c| c.len()).sum();
                let codes = chunks
                    .iter()
                    .map(|c| string_values(c.as_ref()))
                    .collect::<PyResult<Vec<_>>>()?;
                py.detach(|| batch::decode_batch(&decoder, codes.into_iter().flatten(), layout, n))
            }
        }
        .map_err(|e| PyValueError::new_err(format!("Failed to create RecordBatch: {}", e)))?;

        // Convert to PyArrow via pyo3-arrow
        PyRecordBatch::new(batch)
            .into_pyarrow(py)
            .map(|b| b.unbind())
            .map_err(|e| PyValueError::new_err(format!("Failed to convert to PyArrow: {}", e)))
    }
}

/// OpenLR codes accepted by `Decoder.decode_batch`.
#[derive(FromPyObject)]
enum CodesInput {
    /// Anything exposing `__arrow_c_array__` or `__arrow_c_stream__`.
    Arrow(AnyArray),
    /// A Python sequence of str (copied).
    List(Vec<String>),
}

type StrIter<'a> = Box<dyn Iterator<Item = Option<&'a str>> + Send + 'a>;

/// Iterate the values of an Arrow string array of any offset width.
fn string_values(array: &dyn Array) -> PyResult<StrIter<'_>> {
    if let Some(a) = array.as_string_opt::<i32>() {
        Ok(Box::new(a.iter()))
    } else if let Some(a) = array.as_string_opt::<i64>() {
        Ok(Box::new(a.iter()))
    } else if let Some(a) = array.as_string_view_opt() {
        Ok(Box::new(a.iter()))
    } else {
        Err(PyTypeError::new_err(format!(
            "openlr_codes must be a string array, got {}",
            array.data_type()
        )))
    }
}

/// Get the expected PyArrow schema for road network parquet files
#[pyfunction]
fn road_network_schema(py: Python<'_>) -> PyResult<Py<PyAny>> {
    let schema = Arc::new(crate::loader::road_network_schema());
    PySchema::new(schema)
        .into_pyarrow(py)
        .map(|b| b.unbind())
        .map_err(|e| PyValueError::new_err(format!("Failed to convert schema: {}", e)))
}

/// Python module for openlr_decoder
#[pymodule]
fn _openlr_decoder(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRoadNetwork>()?;
    m.add_class::<PyDecoder>()?;
    m.add_class::<PyDecoderConfig>()?;
    m.add_class::<PyDecodedPath>()?;
    m.add_function(wrap_pyfunction!(road_network_schema, m)?)?;
    Ok(())
}
