//! PyO3 bindings, built as the `openlr_decoder._openlr_decoder` extension module.

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

/// A road network graph and its spatial index. Build one with `from_parquet`
/// or `from_arrow`.
#[pyclass(name = "RoadNetwork")]
pub struct PyRoadNetwork {
    network: Arc<RoadNetwork>,
    spatial: Arc<SpatialIndex>,
}

#[pymethods]
impl PyRoadNetwork {
    /// Load a road network from a parquet file.
    ///
    /// Args:
    ///     path: Path to a parquet file with the columns described in
    ///         `road_network_schema()`.
    ///
    /// Returns:
    ///     RoadNetwork
    ///
    /// Raises:
    ///     OSError: If the file cannot be opened, e.g. FileNotFoundError.
    ///     ValueError: If the file is not valid parquet or does not match the schema.
    #[staticmethod]
    #[pyo3(signature = (path))]
    fn from_parquet(py: Python<'_>, path: &str) -> PyResult<Self> {
        // Nothing here touches Python objects, so release the GIL.
        let (network, spatial) = py
            .detach(|| load_network_from_parquet(Path::new(path)))
            .map_err(|e| {
                let message = format!("Failed to load network: {e:#}");
                // PyErr::from(io::Error) picks the OSError subclass for the kind.
                match e.chain().find_map(|c| c.downcast_ref::<std::io::Error>()) {
                    Some(io) => std::io::Error::new(io.kind(), message).into(),
                    None => PyValueError::new_err(message),
                }
            })?;

        Ok(PyRoadNetwork {
            network: Arc::new(network),
            spatial: Arc::new(spatial),
        })
    }

    /// Load a road network from Arrow data.
    ///
    /// Accepts any object exposing `__arrow_c_stream__` or `__arrow_c_array__`,
    /// such as a PyArrow Table, RecordBatch or RecordBatchReader, or a Polars
    /// DataFrame. The Arrow import is zero-copy, and a stream is consumed one
    /// batch at a time.
    ///
    /// Args:
    ///     data: Arrow data with columns stableEdgeId (uint64), startOsmNode and
    ///         endOsmNode (int64), startLat/startLon/endLat/endLon (float64) and
    ///         highway (string). Optional: lanes (int64) and geometry (WKB, WKT
    ///         or GeoArrow linestring). See `road_network_schema()`.
    ///
    /// Returns:
    ///     RoadNetwork
    ///
    /// Raises:
    ///     ValueError: If the data cannot be read or does not match the schema.
    ///
    /// Example:
    ///     >>> import pyarrow.parquet as pq
    ///     >>> network = RoadNetwork.from_arrow(pq.read_table("network.parquet"))
    ///
    ///     >>> import polars as pl
    ///     >>> network = RoadNetwork.from_arrow(pl.read_parquet("network.parquet"))
    #[staticmethod]
    #[pyo3(signature = (data))]
    fn from_arrow(py: Python<'_>, data: PyRecordBatchReader) -> PyResult<Self> {
        let reader = data
            .into_reader()
            .map_err(|e| PyValueError::new_err(format!("Failed to read Arrow data: {}", e)))?;

        // Pull each batch with the GIL held, since the stream's producer may be
        // Python code, and release it for graph construction, the expensive part.
        // Each batch is dropped before the next is fetched.
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

    /// Number of directed edges.
    #[getter]
    fn edge_count(&self) -> usize {
        self.network.graph.edge_count()
    }

    /// Number of nodes (distinct OSM node ids).
    #[getter]
    fn node_count(&self) -> usize {
        self.network.graph.node_count()
    }
}

/// Decoder settings. Every argument is optional; the defaults are tuned for
/// HERE-encoded codes on an OSM network.
#[pyclass(name = "DecoderConfig")]
#[derive(Clone)]
pub struct PyDecoderConfig {
    /// Radius in meters searched around each LRP for candidate edges.
    #[pyo3(get, set)]
    pub search_radius_m: f64,
    /// Largest bearing difference in degrees between an LRP and a candidate edge.
    #[pyo3(get, set)]
    pub max_bearing_diff: f64,
    /// Largest FRC difference (0-7) between an LRP and a candidate edge. The
    /// path between two LRPs may also exceed their LFRCNP by this much.
    #[pyo3(get, set)]
    pub frc_tolerance: u8,
    /// Candidates kept per LRP, best-scored first.
    #[pyo3(get, set)]
    pub max_candidates: usize,
    /// Decoding fails unless at least one LRP has a candidate within this many
    /// meters. The other LRPs may match up to `search_radius_m` away.
    #[pyo3(get, set)]
    pub max_candidate_distance_m: f64,
    /// Allowed difference between a segment's path length and its DNP, as a
    /// fraction of the DNP (0.35 = 35%). A path passes if it is within either
    /// this or `absolute_length_tolerance`.
    #[pyo3(get, set)]
    pub length_tolerance: f64,
    /// Allowed difference in meters between a segment's path length and its DNP.
    #[pyo3(get, set)]
    pub absolute_length_tolerance: f64,
    /// A* gives up beyond this multiple of the DNP, or 500 m if that is larger.
    #[pyo3(get, set)]
    pub max_search_distance_factor: f64,
    /// Candidate score weight for LRP-to-edge distance.
    #[pyo3(get, set)]
    pub distance_weight: f64,
    /// Candidate score weight for bearing difference.
    #[pyo3(get, set)]
    pub bearing_weight: f64,
    /// Candidate score weight for FRC difference.
    #[pyo3(get, set)]
    pub frc_weight: f64,
    /// Candidate score weight for FOW mismatch.
    #[pyo3(get, set)]
    pub fow_weight: f64,
    /// Extra A* cost in meters for each slip road (ramp or `*_link`) traversed,
    /// so the path prefers main roads when lengths are similar.
    #[pyo3(get, set)]
    pub slip_road_cost_penalty: f64,
    /// Extra A* cost in meters for each residential, living_street, service or
    /// track edge traversed, so the path prefers tertiary and above.
    #[pyo3(get, set)]
    pub access_road_cost_penalty: f64,
    /// Extend the path at each end to the nearest junction (OpenLR Rule 4),
    /// folding the added length into the offsets.
    #[pyo3(get, set)]
    pub snap_to_valid_nodes: bool,
    /// Longest extension in meters that `snap_to_valid_nodes` may add at each end.
    #[pyo3(get, set)]
    pub max_snap_extension_m: f64,
}

#[pymethods]
impl PyDecoderConfig {
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

/// A decoded line location. All distances are in meters.
#[pyclass(name = "DecodedPath")]
pub struct PyDecodedPath {
    /// Stable edge ids along the path, in travel order.
    #[pyo3(get)]
    pub edge_ids: Vec<u64>,
    /// Full length of each edge in `edge_ids`.
    #[pyo3(get)]
    pub edge_lengths: Vec<f64>,
    /// Length of the location along the path, from the first LRP to the last.
    /// This excludes the offsets, and includes any start or end edge under 3%
    /// of the path that was left out of `edge_ids`.
    #[pyo3(get)]
    pub length: f64,
    /// Length to trim from the start of the path to where the location begins.
    #[pyo3(get)]
    pub positive_offset: f64,
    /// Length to trim from the end of the path to where the location ends.
    #[pyo3(get)]
    pub negative_offset: f64,
    /// `positive_offset` as a fraction of the first edge. Exceeds 1.0 when a
    /// terminal-node snap prepended edges; the meter offset is authoritative.
    #[pyo3(get)]
    pub positive_offset_fraction: f64,
    /// `negative_offset` as a fraction of the last edge. Exceeds 1.0 when a
    /// terminal-node snap appended edges; the meter offset is authoritative.
    #[pyo3(get)]
    pub negative_offset_fraction: f64,
    /// The edge covering the most of the location. Ties go to the earliest edge.
    #[pyo3(get)]
    pub primary_edge_id: u64,
    /// Length of the location that lies on `primary_edge_id`.
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
            edge_lengths: path.edge_lengths_m,
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

/// Decodes base64 OpenLR line locations onto a `RoadNetwork`.
///
/// Args:
///     network: The road network to decode against. It is shared, not copied.
///     config: Decoder settings. Defaults to `DecoderConfig()`.
#[pyclass(name = "Decoder")]
pub struct PyDecoder {
    network: Arc<RoadNetwork>,
    spatial: Arc<SpatialIndex>,
    config: DecoderConfig,
}

#[pymethods]
impl PyDecoder {
    #[new]
    #[pyo3(signature = (network, config = None))]
    fn new(network: &PyRoadNetwork, config: Option<PyRef<'_, PyDecoderConfig>>) -> Self {
        let decoder_config = config
            .as_deref()
            .map(DecoderConfig::from)
            .unwrap_or_default();

        PyDecoder {
            network: Arc::clone(&network.network),
            spatial: Arc::clone(&network.spatial),
            config: decoder_config,
        }
    }

    /// Decode one OpenLR line location. Releases the GIL while decoding.
    ///
    /// Args:
    ///     openlr_base64: Base64-encoded OpenLR location reference.
    ///
    /// Returns:
    ///     DecodedPath
    ///
    /// Raises:
    ///     ValueError: If the code is invalid or no matching path is found. The
    ///         message says which.
    fn decode(&self, py: Python<'_>, openlr_base64: &str) -> PyResult<PyDecodedPath> {
        py.detach(|| {
            let decoder =
                Decoder::new(&self.network, &self.spatial).with_config(self.config.clone());
            decoder.decode(openlr_base64)
        })
        .map(PyDecodedPath::from)
        .map_err(|e| PyValueError::new_err(decode_error_message(e)))
    }

    /// Decode many OpenLR codes in parallel into one Arrow RecordBatch.
    /// Releases the GIL while decoding.
    ///
    /// Args:
    ///     openlr_codes: Base64-encoded OpenLR location references, as a list of
    ///         str or as an Arrow ``string``, ``large_string`` or ``string_view``
    ///         array exposed through the Arrow PyCapsule interface, such as a
    ///         PyArrow ``Array``/``ChunkedArray`` or a Polars ``Series``. Arrow
    ///         input is read zero-copy. Null entries are reported as errors.
    ///     explode: If True, return one row per (code, edge) instead of one row
    ///         per code.
    ///
    /// Returns:
    ///     pyarrow.RecordBatch. Columns named as on `DecodedPath` mean the same
    ///     thing. With ``explode=False``, one row per code:
    ///     - edge_ids: list<uint64>, empty on failure
    ///     - length, positive_offset, negative_offset, positive_offset_fraction,
    ///       negative_offset_fraction, primary_edge_coverage: float64
    ///     - primary_edge_id: uint64
    ///     - error: string, null on success. The other columns are null on failure.
    ///
    ///     With ``explode=True``, one row per edge of each path, and a single row
    ///     with null edge columns for a code that has no edges (such as a failed one):
    ///     - code_index: uint64, position of the code in ``openlr_codes``
    ///     - edge_id: uint64
    ///     - edge_position: uint32, 0-based position of the edge in the path
    ///     - is_primary: bool, whether this is the path's ``primary_edge_id``
    ///     - offset_fraction: float64, fraction (0-1) of this edge outside the
    ///       location. The meter offsets are consumed edge by edge from each end
    ///       of the path, so edges added by a terminal-node snap are 1.0.
    ///     - length: float64
    ///     - error: string, null on success
    ///
    /// Raises:
    ///     TypeError: If ``openlr_codes`` is an Arrow array of a non-string type.
    ///     ValueError: If the Arrow data cannot be read.
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
                // Import the chunks (zero-copy) with the GIL held, since a stream
                // may be produced by Python code.
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

/// The PyArrow schema `RoadNetwork.from_parquet` and `RoadNetwork.from_arrow`
/// expect. `lanes` and `geometry` are optional, and `geometry` may also be WKT
/// or GeoArrow.
#[pyfunction]
fn road_network_schema(py: Python<'_>) -> PyResult<Py<PyAny>> {
    let schema = Arc::new(crate::loader::road_network_schema());
    PySchema::new(schema)
        .into_pyarrow(py)
        .map(|b| b.unbind())
        .map_err(|e| PyValueError::new_err(format!("Failed to convert schema: {}", e)))
}

#[pymodule]
fn _openlr_decoder(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRoadNetwork>()?;
    m.add_class::<PyDecoder>()?;
    m.add_class::<PyDecoderConfig>()?;
    m.add_class::<PyDecodedPath>()?;
    m.add_function(wrap_pyfunction!(road_network_schema, m)?)?;
    Ok(())
}
