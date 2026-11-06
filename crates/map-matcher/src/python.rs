//! PyO3 bindings, built as the `map_matcher._map_matcher` extension module.

use std::path::Path;
use std::sync::Arc;

use arrow::array::{
    Array, ArrayRef, AsArray, Float64Array, ListBuilder, RecordBatch, UInt32Array, UInt64Array,
    UInt64Builder,
};
use arrow::compute::{cast, concat_batches, partition, take};
use arrow::datatypes::{DataType, Field, Float64Type, Int64Type, Schema, TimeUnit};
use geo::Point;
use pyo3::exceptions::{PyOSError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3_arrow::{PyRecordBatch, PyRecordBatchReader};
use rayon::prelude::*;
use road_network::loader::{load_network_from_parquet, NetworkBuilder};
use road_network::{Frc, RoadNetwork, SpatialIndex};

use crate::{MatchConfig, MatchError, MatchResult, Matcher, Ping};

/// A road network graph and its spatial index. Build one with `from_parquet`
/// or `from_arrow`.
#[pyclass(name = "RoadNetwork", frozen)]
pub struct PyRoadNetwork {
    network: Arc<RoadNetwork>,
    spatial: Arc<SpatialIndex>,
}

#[pymethods]
impl PyRoadNetwork {
    /// Load a road network from a parquet file with the columns of
    /// `openlr_decoder.road_network_schema()`.
    ///
    /// Raises:
    ///     OSError: If the file cannot be opened.
    ///     ValueError: If the file is not valid parquet or does not match the schema.
    #[staticmethod]
    fn from_parquet(py: Python<'_>, path: &str) -> PyResult<Self> {
        let (network, spatial) = py
            .detach(|| load_network_from_parquet(Path::new(path)))
            .map_err(|e| {
                let message = format!("Failed to load network: {e:#}");
                match e.chain().find_map(|c| c.downcast_ref::<std::io::Error>()) {
                    Some(io) => match io.raw_os_error() {
                        Some(errno) => PyOSError::new_err((errno, message, path.to_string())),
                        None => std::io::Error::new(io.kind(), message).into(),
                    },
                    None => PyValueError::new_err(message),
                }
            })?;
        Ok(Self::new(network, spatial))
    }

    /// Load a road network from Arrow data: anything exposing
    /// `__arrow_c_stream__` or `__arrow_c_array__`, such as a PyArrow Table or
    /// a Polars DataFrame, with the same columns as `from_parquet`.
    ///
    /// Raises:
    ///     ValueError: If the data cannot be read or does not match the schema.
    #[staticmethod]
    fn from_arrow(py: Python<'_>, data: PyRecordBatchReader) -> PyResult<Self> {
        let reader = data.into_reader().map_err(arrow_read_error)?;
        let mut builder = NetworkBuilder::default();
        for batch in reader {
            let batch = batch.map_err(arrow_read_error)?;
            py.detach(|| builder.push_batch(&batch)).map_err(|e| {
                PyValueError::new_err(format!("Failed to build network from Arrow data: {e}"))
            })?;
        }
        let (network, spatial) = py.detach(|| builder.finish());
        Ok(Self::new(network, spatial))
    }

    /// Number of directed edges.
    #[getter]
    fn edge_count(&self) -> usize {
        self.network.edge_count()
    }

    /// Number of nodes.
    #[getter]
    fn node_count(&self) -> usize {
        self.network.node_count()
    }
}

impl PyRoadNetwork {
    fn new(network: RoadNetwork, spatial: SpatialIndex) -> Self {
        PyRoadNetwork {
            network: Arc::new(network),
            spatial: Arc::new(spatial),
        }
    }
}

/// Matcher settings. Distances are meters, speeds m/s, headings degrees.
///
/// Args:
///     sigma_m: Standard deviation of GPS position error.
///     search_radius_m: Edges further from a ping are not candidates.
///         Default 4 × sigma_m, at least 50.
///     max_candidates: Candidates kept per ping.
///     beta_m: Scale of the penalty on route distance minus straight-line
///         distance between pings. Larger tolerates more detour.
///     use_heading: Score candidates by heading difference when a ping has a heading.
///     heading_kappa: Strength of the heading term.
///     heading_min_speed_mps: Ignore a ping's heading below this speed.
///     use_speed: Score transitions by how well the route distance matches
///         the pings' average speed × the time between them.
///     speed_sigma_mps: Tolerance of that speed match.
///     max_speed_mps: Routes faster than this between pings are not considered.
///     backward_slack_m: How far a ping may project behind the previous one
///         on the same edge and still count as stopped. Default 3 × sigma_m.
///     max_frc: Least important road class matched, 0 (motorway) to 7.
#[pyclass(name = "MatchConfig", frozen)]
pub struct PyMatchConfig {
    config: MatchConfig,
}

#[pymethods]
impl PyMatchConfig {
    #[new]
    #[pyo3(signature = (
        sigma_m = 10.0,
        *,
        search_radius_m = None,
        max_candidates = 8,
        beta_m = 50.0,
        use_heading = true,
        heading_kappa = 4.0,
        heading_min_speed_mps = 3.0,
        use_speed = true,
        speed_sigma_mps = 4.0,
        max_speed_mps = 40.0,
        backward_slack_m = None,
        max_frc = 4,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        sigma_m: f64,
        search_radius_m: Option<f64>,
        max_candidates: usize,
        beta_m: f64,
        use_heading: bool,
        heading_kappa: f64,
        heading_min_speed_mps: f64,
        use_speed: bool,
        speed_sigma_mps: f64,
        max_speed_mps: f64,
        backward_slack_m: Option<f64>,
        max_frc: u8,
    ) -> PyResult<Self> {
        if sigma_m.is_nan() || sigma_m <= 0.0 {
            return Err(PyValueError::new_err("sigma_m must be positive"));
        }
        if max_frc > 7 {
            return Err(PyValueError::new_err("max_frc must be 0 to 7"));
        }
        let base = MatchConfig::with_sigma(sigma_m);
        Ok(PyMatchConfig {
            config: MatchConfig {
                search_radius_m: search_radius_m.unwrap_or(base.search_radius_m),
                max_candidates,
                beta_m,
                use_heading,
                heading_kappa,
                heading_min_speed_mps,
                use_speed,
                speed_sigma_mps,
                max_speed_mps,
                backward_slack_m: backward_slack_m.unwrap_or(base.backward_slack_m),
                max_frc: Frc::from_u8(max_frc),
                ..base
            },
        })
    }

    fn __repr__(&self) -> String {
        let c = &self.config;
        format!(
            "MatchConfig(sigma_m={}, search_radius_m={}, max_candidates={}, beta_m={}, \
             use_heading={}, heading_kappa={}, heading_min_speed_mps={}, use_speed={}, \
             speed_sigma_mps={}, max_speed_mps={}, backward_slack_m={}, max_frc={})",
            c.sigma_m,
            c.search_radius_m,
            c.max_candidates,
            c.beta_m,
            py_bool(c.use_heading),
            c.heading_kappa,
            c.heading_min_speed_mps,
            py_bool(c.use_speed),
            c.speed_sigma_mps,
            c.max_speed_mps,
            c.backward_slack_m,
            c.max_frc as u8,
        )
    }
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

/// Matches GPS traces to a road network.
///
/// Args:
///     network: The RoadNetwork to match onto.
///     config: Optional MatchConfig; defaults to MatchConfig().
#[pyclass(name = "Matcher", frozen)]
pub struct PyMatcher {
    network: Arc<RoadNetwork>,
    spatial: Arc<SpatialIndex>,
    config: MatchConfig,
}

#[pymethods]
impl PyMatcher {
    #[new]
    #[pyo3(signature = (network, config = None))]
    fn new(network: &PyRoadNetwork, config: Option<PyRef<'_, PyMatchConfig>>) -> Self {
        PyMatcher {
            network: network.network.clone(),
            spatial: network.spatial.clone(),
            config: config.map(|c| c.config.clone()).unwrap_or_default(),
        }
    }

    /// Match GPS traces, in parallel.
    ///
    /// Args:
    ///     pings: Arrow data (PyArrow Table, Polars DataFrame, ...) with columns
    ///         - trace_id: any type. A trace is a run of consecutive rows with
    ///           the same trace_id, so sort or group by it first.
    ///         - time: a timestamp, or seconds as a number. Must not decrease
    ///           within a trace.
    ///         - lat, lon: degrees.
    ///         - speed (optional): m/s, may be null.
    ///         - heading (optional): degrees clockwise from north, may be null.
    ///
    /// Returns:
    ///     pyarrow.RecordBatch with one row per matched segment, a stretch of a
    ///     trace matched as one connected path. A trace splits into several
    ///     segments where no plausible route connects consecutive pings.
    ///     - trace_id: same type as the input
    ///     - segment: uint32, 0-based within the trace
    ///     - first_row, last_row: uint64, the input rows the segment spans
    ///     - edge_ids: list<uint64>, stableEdgeId of each edge in travel order
    ///     - start_offset_m: float64, meters along the first edge where the path starts
    ///     - end_offset_m: float64, meters along the last edge where it ends
    ///     - unmatched_rows: list<uint64>, rows in the span with no road nearby
    ///     Rows outside every segment's span also had no road nearby.
    ///
    /// Raises:
    ///     ValueError: If a column is missing, has nulls where not allowed, or
    ///         time decreases within a trace.
    ///     TypeError: If a column has an unsupported type.
    fn match_traces(&self, py: Python<'_>, pings: PyRecordBatchReader) -> PyResult<Py<PyAny>> {
        let reader = pings.into_reader().map_err(arrow_read_error)?;
        let schema = reader.schema();
        let batches = reader
            .collect::<Result<Vec<_>, _>>()
            .map_err(arrow_read_error)?;
        let batch = concat_batches(&schema, &batches)
            .map_err(|e| PyValueError::new_err(format!("Failed to combine Arrow batches: {e}")))?;

        let out = py.detach(|| self.match_batch(&batch))?;
        PyRecordBatch::new(out)
            .into_pyarrow(py)
            .map(|b| b.unbind())
            .map_err(|e| PyValueError::new_err(format!("Failed to convert to PyArrow: {e}")))
    }

    fn __repr__(&self) -> String {
        format!("Matcher(network with {} edges)", self.network.edge_count())
    }
}

impl PyMatcher {
    fn match_batch(&self, batch: &RecordBatch) -> PyResult<RecordBatch> {
        let column = |name: &str| {
            batch
                .column_by_name(name)
                .ok_or_else(|| PyValueError::new_err(format!("Missing column '{name}'")))
        };
        let trace_id = column("trace_id")?.clone();
        let time = seconds(column("time")?)?;
        let lat = floats(column("lat")?, "lat")?;
        let lon = floats(column("lon")?, "lon")?;
        let speed = batch
            .column_by_name("speed")
            .map(|c| floats(c, "speed"))
            .transpose()?;
        let heading = batch
            .column_by_name("heading")
            .map(|c| floats(c, "heading"))
            .transpose()?;
        for (name, col) in [("time", &time), ("lat", &lat), ("lon", &lon)] {
            if let Some(row) = (0..col.len()).find(|&i| col.is_null(i)) {
                return Err(PyValueError::new_err(format!(
                    "Column '{name}' is null at row {row}"
                )));
            }
        }

        let ranges = partition(std::slice::from_ref(&trace_id))
            .map_err(|e| PyTypeError::new_err(format!("Cannot group by trace_id: {e}")))?
            .ranges();

        let ping = |i: usize| Ping {
            point: Point::new(lon.value(i), lat.value(i)),
            time_s: time.value(i),
            speed_mps: speed
                .as_ref()
                .and_then(|s| s.is_valid(i).then(|| s.value(i))),
            heading_deg: heading
                .as_ref()
                .and_then(|h| h.is_valid(i).then(|| h.value(i))),
        };
        let matcher = Matcher::new(&self.network, &self.spatial, self.config.clone());
        let results: Vec<MatchResult> = ranges
            .par_iter()
            .map(|r| {
                let pings: Vec<Ping> = r.clone().map(ping).collect();
                matcher.match_trace(&pings).map_err(|e| match e {
                    MatchError::TimeDecreases { index } => PyValueError::new_err(format!(
                        "time decreases at row {} within its trace",
                        r.start + index
                    )),
                })
            })
            .collect::<PyResult<_>>()?;

        self.segments_batch(&trace_id, &ranges, &results)
    }

    fn segments_batch(
        &self,
        trace_id: &ArrayRef,
        ranges: &[std::ops::Range<usize>],
        results: &[MatchResult],
    ) -> PyResult<RecordBatch> {
        let mut id_rows = Vec::new();
        let mut segment = Vec::new();
        let mut first_row = Vec::new();
        let mut last_row = Vec::new();
        let mut start_offset = Vec::new();
        let mut end_offset = Vec::new();
        let mut edge_ids = ListBuilder::new(UInt64Builder::new());
        let mut unmatched = ListBuilder::new(UInt64Builder::new());

        for (range, result) in ranges.iter().zip(results) {
            let row = |i: usize| (range.start + i) as u64;
            for (k, seg) in result.segments.iter().enumerate() {
                let first = seg.pings.first().map_or(0, |p| p.index);
                let last = seg.pings.last().map_or(0, |p| p.index);
                id_rows.push(row(0));
                segment.push(k as u32);
                first_row.push(row(first));
                last_row.push(row(last));
                start_offset.push(seg.start_offset_m);
                end_offset.push(seg.end_offset_m);
                for &e in &seg.edges {
                    edge_ids
                        .values()
                        .append_value(self.network.edge(e).map_or(0, |e| e.id));
                }
                edge_ids.append(true);
                for &i in result.unmatched.iter().filter(|&&i| first < i && i < last) {
                    unmatched.values().append_value(row(i));
                }
                unmatched.append(true);
            }
        }

        let trace_ids = take(trace_id.as_ref(), &UInt64Array::from(id_rows), None)
            .map_err(|e| PyValueError::new_err(format!("Failed to build trace_id: {e}")))?;
        let list = |name: &str| {
            Field::new(
                name,
                DataType::List(Arc::new(Field::new("item", DataType::UInt64, true))),
                false,
            )
        };
        let schema = Schema::new(vec![
            Field::new("trace_id", trace_id.data_type().clone(), true),
            Field::new("segment", DataType::UInt32, false),
            Field::new("first_row", DataType::UInt64, false),
            Field::new("last_row", DataType::UInt64, false),
            list("edge_ids"),
            Field::new("start_offset_m", DataType::Float64, false),
            Field::new("end_offset_m", DataType::Float64, false),
            list("unmatched_rows"),
        ]);
        RecordBatch::try_new(
            Arc::new(schema),
            vec![
                trace_ids,
                Arc::new(UInt32Array::from(segment)),
                Arc::new(UInt64Array::from(first_row)),
                Arc::new(UInt64Array::from(last_row)),
                Arc::new(edge_ids.finish()),
                Arc::new(Float64Array::from(start_offset)),
                Arc::new(Float64Array::from(end_offset)),
                Arc::new(unmatched.finish()),
            ],
        )
        .map_err(|e| PyValueError::new_err(format!("Failed to build result: {e}")))
    }
}

/// A numeric column as float64.
fn floats(col: &ArrayRef, name: &str) -> PyResult<Float64Array> {
    if !col.data_type().is_numeric() {
        return Err(PyTypeError::new_err(format!(
            "Column '{name}' must be numeric, got {}",
            col.data_type()
        )));
    }
    let f = cast(col, &DataType::Float64)
        .map_err(|e| PyTypeError::new_err(format!("Column '{name}': {e}")))?;
    Ok(f.as_primitive::<Float64Type>().clone())
}

/// The `time` column in seconds: a timestamp of any unit, or a number.
fn seconds(col: &ArrayRef) -> PyResult<Float64Array> {
    let DataType::Timestamp(unit, _) = col.data_type() else {
        return floats(col, "time");
    };
    let per_second = match unit {
        TimeUnit::Second => 1.0,
        TimeUnit::Millisecond => 1e3,
        TimeUnit::Microsecond => 1e6,
        TimeUnit::Nanosecond => 1e9,
    };
    let ints = cast(col, &DataType::Int64)
        .map_err(|e| PyTypeError::new_err(format!("Column 'time': {e}")))?;
    Ok(ints
        .as_primitive::<Int64Type>()
        .unary::<_, Float64Type>(|t| t as f64 / per_second))
}

fn arrow_read_error(e: impl std::fmt::Display) -> PyErr {
    PyValueError::new_err(format!("Failed to read Arrow data: {e}"))
}

#[pymodule]
fn _map_matcher(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRoadNetwork>()?;
    m.add_class::<PyMatchConfig>()?;
    m.add_class::<PyMatcher>()?;
    Ok(())
}
