//! Batch decoding into Arrow record batches.
//!
//! Codes are decoded in parallel one chunk at a time and each chunk's results are
//! appended to the Arrow builders before the next chunk starts, so the per-code
//! `DecodedPath`s never all exist at once alongside the output arrays.

use std::sync::Arc;

use arrow::array::{
    ArrayRef, BooleanBuilder, Float64Builder, ListBuilder, StringBuilder, UInt32Builder,
    UInt64Builder,
};
use arrow::error::ArrowError;
use arrow::record_batch::RecordBatch;
use rayon::prelude::*;

use crate::decoder::{DecodeError, DecodedPath, Decoder};

/// Number of codes decoded in parallel before their results are appended to the
/// output. Large enough to keep every core busy, small enough that the transient
/// `DecodedPath`s are negligible next to the output arrays.
pub const DECODE_CHUNK_SIZE: usize = 1 << 16;

/// Error message reported for a null input code.
pub const NULL_CODE_ERROR: &str = "Missing OpenLR code (null)";

/// Output layout of [`decode_batch`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatchLayout {
    /// One row per code, edge ids as a `list<uint64>` column.
    Nested,
    /// One row per (code, edge); a failed code gets a single row with null edge
    /// columns and its error message.
    Exploded,
}

type Outcome = Result<DecodedPath, String>;

/// Decode `codes` (null entries become errors) into a single `RecordBatch`.
///
/// `len_hint` pre-sizes the per-code columns; pass the number of codes if known.
pub fn decode_batch<'a>(
    decoder: &Decoder,
    codes: impl IntoIterator<Item = Option<&'a str>>,
    layout: BatchLayout,
    len_hint: usize,
) -> Result<RecordBatch, ArrowError> {
    match layout {
        BatchLayout::Nested => decode_into(decoder, codes, NestedSink::with_capacity(len_hint)),
        BatchLayout::Exploded => decode_into(decoder, codes, ExplodedSink::default()),
    }
}

fn decode_into<'a, S: Sink>(
    decoder: &Decoder,
    codes: impl IntoIterator<Item = Option<&'a str>>,
    mut sink: S,
) -> Result<RecordBatch, ArrowError> {
    let mut chunk: Vec<Option<&str>> = Vec::with_capacity(DECODE_CHUNK_SIZE);
    let flush = |chunk: &mut Vec<Option<&str>>, sink: &mut S| {
        let outcomes: Vec<Outcome> = chunk
            .par_iter()
            .map(|code| match code {
                Some(code) => decoder.decode(code).map_err(decode_error_message),
                None => Err(NULL_CODE_ERROR.to_string()),
            })
            .collect();
        for outcome in outcomes {
            sink.push(outcome);
        }
        chunk.clear();
    };

    for code in codes {
        chunk.push(code);
        if chunk.len() == DECODE_CHUNK_SIZE {
            flush(&mut chunk, &mut sink);
        }
    }
    if !chunk.is_empty() {
        flush(&mut chunk, &mut sink);
    }
    sink.finish()
}

/// Convert a decode error to a user-facing message.
pub fn decode_error_message(err: DecodeError) -> String {
    match err {
        DecodeError::Deserialize(msg) => format!("Invalid OpenLR: {}", msg),
        DecodeError::NoCandidates { index } => {
            format!(
                "No matching roads found for location reference point {}",
                index
            )
        }
        DecodeError::NoPath { from, to } => {
            format!("No valid path found between points {} and {}", from, to)
        }
        DecodeError::LengthMismatch { expected, actual } => {
            format!(
                "Path length mismatch: expected {:.0}m, got {:.0}m",
                expected, actual
            )
        }
        DecodeError::UnsupportedType(typ) => {
            format!("Unsupported location type: {}", typ)
        }
    }
}

trait Sink {
    fn push(&mut self, outcome: Outcome);
    fn finish(self) -> Result<RecordBatch, ArrowError>;
}

/// Builders for [`BatchLayout::Nested`].
struct NestedSink {
    edge_ids: ListBuilder<UInt64Builder>,
    length: Float64Builder,
    positive_offset: Float64Builder,
    negative_offset: Float64Builder,
    positive_offset_fraction: Float64Builder,
    negative_offset_fraction: Float64Builder,
    primary_edge_id: UInt64Builder,
    primary_edge_coverage: Float64Builder,
    error: StringBuilder,
}

impl NestedSink {
    fn with_capacity(n: usize) -> Self {
        NestedSink {
            edge_ids: ListBuilder::with_capacity(UInt64Builder::new(), n),
            length: Float64Builder::with_capacity(n),
            positive_offset: Float64Builder::with_capacity(n),
            negative_offset: Float64Builder::with_capacity(n),
            positive_offset_fraction: Float64Builder::with_capacity(n),
            negative_offset_fraction: Float64Builder::with_capacity(n),
            primary_edge_id: UInt64Builder::with_capacity(n),
            primary_edge_coverage: Float64Builder::with_capacity(n),
            error: StringBuilder::new(),
        }
    }
}

impl Sink for NestedSink {
    fn push(&mut self, outcome: Outcome) {
        match outcome {
            Ok(path) => {
                self.edge_ids.values().append_slice(&path.edge_ids);
                self.edge_ids.append(true);
                self.length.append_value(path.length_m);
                self.positive_offset.append_value(path.positive_offset_m);
                self.negative_offset.append_value(path.negative_offset_m);
                self.positive_offset_fraction
                    .append_value(path.positive_offset_fraction);
                self.negative_offset_fraction
                    .append_value(path.negative_offset_fraction);
                self.primary_edge_id.append_value(path.primary_edge_id);
                self.primary_edge_coverage
                    .append_value(path.primary_edge_coverage_m);
                self.error.append_null();
            }
            Err(message) => {
                // Empty list for failed decodes
                self.edge_ids.append(true);
                self.length.append_null();
                self.positive_offset.append_null();
                self.negative_offset.append_null();
                self.positive_offset_fraction.append_null();
                self.negative_offset_fraction.append_null();
                self.primary_edge_id.append_null();
                self.primary_edge_coverage.append_null();
                self.error.append_value(message);
            }
        }
    }

    fn finish(mut self) -> Result<RecordBatch, ArrowError> {
        RecordBatch::try_from_iter(vec![
            ("edge_ids", Arc::new(self.edge_ids.finish()) as ArrayRef),
            ("length", Arc::new(self.length.finish())),
            ("positive_offset", Arc::new(self.positive_offset.finish())),
            ("negative_offset", Arc::new(self.negative_offset.finish())),
            (
                "positive_offset_fraction",
                Arc::new(self.positive_offset_fraction.finish()),
            ),
            (
                "negative_offset_fraction",
                Arc::new(self.negative_offset_fraction.finish()),
            ),
            ("primary_edge_id", Arc::new(self.primary_edge_id.finish())),
            (
                "primary_edge_coverage",
                Arc::new(self.primary_edge_coverage.finish()),
            ),
            ("error", Arc::new(self.error.finish())),
        ])
    }
}

/// Builders for [`BatchLayout::Exploded`].
#[derive(Default)]
struct ExplodedSink {
    next_code_index: u64,
    code_index: UInt64Builder,
    edge_id: UInt64Builder,
    edge_position: UInt32Builder,
    is_primary: BooleanBuilder,
    offset_fraction: Float64Builder,
    length: Float64Builder,
    error: StringBuilder,
}

impl ExplodedSink {
    /// The row for a code with no edges: null edge columns, `error` as given.
    fn push_edgeless(&mut self, code_index: u64, length: Option<f64>, error: Option<&str>) {
        self.code_index.append_value(code_index);
        self.edge_id.append_null();
        self.edge_position.append_null();
        self.is_primary.append_null();
        self.offset_fraction.append_null();
        self.length.append_option(length);
        self.error.append_option(error);
    }
}

impl Sink for ExplodedSink {
    fn push(&mut self, outcome: Outcome) {
        let code_index = self.next_code_index;
        self.next_code_index += 1;

        let path = match outcome {
            Ok(path) => path,
            Err(message) => return self.push_edgeless(code_index, None, Some(&message)),
        };
        // Every code yields at least one row, so callers can always join back.
        if path.edge_ids.is_empty() {
            return self.push_edgeless(code_index, Some(path.length_m), None);
        }

        let offset_fractions = edge_offset_fractions(&path);
        for (position, (&edge_id, offset_fraction)) in
            path.edge_ids.iter().zip(offset_fractions).enumerate()
        {
            self.code_index.append_value(code_index);
            self.edge_id.append_value(edge_id);
            self.edge_position.append_value(position as u32);
            self.is_primary
                .append_value(edge_id == path.primary_edge_id);
            self.offset_fraction.append_value(offset_fraction);
            self.length.append_value(path.length_m);
            self.error.append_null();
        }
    }

    fn finish(mut self) -> Result<RecordBatch, ArrowError> {
        RecordBatch::try_from_iter(vec![
            ("code_index", Arc::new(self.code_index.finish()) as ArrayRef),
            ("edge_id", Arc::new(self.edge_id.finish())),
            ("edge_position", Arc::new(self.edge_position.finish())),
            ("is_primary", Arc::new(self.is_primary.finish())),
            ("offset_fraction", Arc::new(self.offset_fraction.finish())),
            ("length", Arc::new(self.length.finish())),
            ("error", Arc::new(self.error.finish())),
        ])
    }
}

/// Fraction (0-1) of each edge in `path` that lies outside the location.
///
/// Each meter offset is consumed edge by edge from its end of the path, so an
/// offset that spans several edges (after a terminal-node snap) trims all of them,
/// not just the first/last.
fn edge_offset_fractions(path: &DecodedPath) -> Vec<f64> {
    fn trim<'a>(offset_m: f64, edges: impl Iterator<Item = (&'a mut f64, &'a f64)>) {
        let mut remaining = offset_m;
        for (trimmed, &length) in edges {
            if remaining <= 0.0 {
                break;
            }
            let take = remaining.min(length);
            *trimmed += take;
            remaining -= take;
        }
    }

    let lengths = &path.edge_lengths_m;
    let mut trimmed_m = vec![0.0; lengths.len()];
    trim(
        path.positive_offset_m,
        trimmed_m.iter_mut().zip(lengths.iter()),
    );
    trim(
        path.negative_offset_m,
        trimmed_m.iter_mut().zip(lengths.iter()).rev(),
    );
    trimmed_m
        .iter()
        .zip(lengths)
        .map(|(&t, &length)| {
            if length > 0.0 {
                (t / length).min(1.0)
            } else {
                0.0
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{Array, AsArray};
    use arrow::datatypes::{Float64Type, UInt32Type, UInt64Type};

    fn path(edges: &[(u64, f64)], primary: u64, pos_m: f64, neg_m: f64) -> DecodedPath {
        let (edge_ids, edge_lengths_m): (Vec<u64>, Vec<f64>) = edges.iter().copied().unzip();
        DecodedPath {
            positive_offset_fraction: pos_m / edge_lengths_m[0],
            negative_offset_fraction: neg_m / edge_lengths_m[edge_lengths_m.len() - 1],
            edge_ids,
            edge_lengths_m,
            length_m: 100.0,
            positive_offset_m: pos_m,
            negative_offset_m: neg_m,
            primary_edge_id: primary,
            primary_edge_coverage_m: 50.0,
        }
    }

    fn outcomes() -> Vec<Outcome> {
        vec![
            Ok(path(&[(10, 4.0), (11, 10.0), (12, 4.0)], 11, 1.0, 2.0)),
            Err("boom".to_string()),
            Ok(path(&[(20, 10.0)], 20, 1.0, 2.0)),
        ]
    }

    fn sink_batch<S: Sink>(mut sink: S) -> RecordBatch {
        for outcome in outcomes() {
            sink.push(outcome);
        }
        sink.finish().unwrap()
    }

    #[test]
    fn nested_layout_has_one_row_per_code() {
        let batch = sink_batch(NestedSink::with_capacity(0));
        assert_eq!(batch.num_rows(), 3);

        let edge_ids = batch.column_by_name("edge_ids").unwrap().as_list::<i32>();
        let first = edge_ids.value(0);
        assert_eq!(first.as_primitive::<UInt64Type>().values(), &[10, 11, 12]);
        assert_eq!(edge_ids.value(1).len(), 0);

        let error = batch.column_by_name("error").unwrap().as_string::<i32>();
        assert!(error.is_null(0));
        assert_eq!(error.value(1), "boom");
        assert!(batch.column_by_name("length").unwrap().is_null(1));
    }

    #[test]
    fn exploded_layout_has_one_row_per_edge_or_failed_code() {
        let batch = sink_batch(ExplodedSink::default());
        assert_eq!(batch.num_rows(), 5);

        let col = |name| batch.column_by_name(name).unwrap();
        assert_eq!(
            col("code_index").as_primitive::<UInt64Type>().values(),
            &[0, 0, 0, 1, 2]
        );

        let edge_id = col("edge_id").as_primitive::<UInt64Type>();
        assert_eq!(edge_id.value(0), 10);
        assert_eq!(edge_id.value(2), 12);
        assert!(edge_id.is_null(3));
        assert_eq!(edge_id.value(4), 20);

        let position = col("edge_position").as_primitive::<UInt32Type>();
        assert_eq!(
            (0..3).map(|i| position.value(i)).collect::<Vec<_>>(),
            vec![0, 1, 2]
        );
        assert!(position.is_null(3));
        assert_eq!(position.value(4), 0);

        let is_primary = col("is_primary").as_boolean();
        assert_eq!(
            (0..3).map(|i| is_primary.value(i)).collect::<Vec<_>>(),
            vec![false, true, false]
        );
        assert!(is_primary.is_null(3));
        assert!(is_primary.value(4));

        let frac = col("offset_fraction").as_primitive::<Float64Type>();
        assert_eq!(frac.value(0), 0.25);
        assert_eq!(frac.value(1), 0.0);
        assert_eq!(frac.value(2), 0.5);
        assert!(frac.is_null(3));
        // Single-edge path: both offsets trim the same edge.
        assert!((frac.value(4) - 0.3).abs() < 1e-12);

        let error = col("error").as_string::<i32>();
        assert_eq!(error.null_count(), 4);
        assert_eq!(error.value(3), "boom");
        assert!(col("length").is_null(3));
    }

    #[test]
    fn offsets_spanning_several_edges_trim_each_of_them() {
        // As after a terminal-node snap: the positive offset covers two whole
        // prepended edges plus 2 m of the third, the negative offset one whole
        // appended edge plus 2 m of the one before it.
        let p = path(
            &[(1, 5.0), (2, 5.0), (3, 20.0), (4, 5.0), (5, 5.0)],
            3,
            12.0,
            7.0,
        );
        let fractions = edge_offset_fractions(&p);
        let expected = [1.0, 1.0, 0.1, 0.4, 1.0];
        for (got, want) in fractions.iter().zip(expected) {
            assert!((got - want).abs() < 1e-12, "{fractions:?}");
        }

        // Both offsets on one edge add up, capped at the whole edge.
        let p = path(&[(1, 10.0)], 1, 6.0, 7.0);
        assert_eq!(edge_offset_fractions(&p), vec![1.0]);
    }

    #[test]
    fn decode_batch_reports_null_and_invalid_codes_in_order() {
        let (network, spatial) = crate::test_utils::TestNetworkBuilder::new().build();
        let decoder = Decoder::new(&network, &spatial);
        let n = DECODE_CHUNK_SIZE + 3; // spans a chunk boundary
        let codes = (0..n).map(|i| {
            if i % 2 == 0 {
                None
            } else {
                Some("not base64!")
            }
        });

        let batch = decode_batch(&decoder, codes, BatchLayout::Nested, n).unwrap();
        assert_eq!(batch.num_rows(), n);
        let error = batch.column_by_name("error").unwrap().as_string::<i32>();
        assert_eq!(error.value(0), NULL_CODE_ERROR);
        assert!(error.value(1).starts_with("Invalid OpenLR"));
        assert_eq!(error.value(n - 1), NULL_CODE_ERROR);

        let codes = [Some("not base64!"), None];
        let batch = decode_batch(&decoder, codes, BatchLayout::Exploded, 0).unwrap();
        let code_index = batch.column_by_name("code_index").unwrap();
        assert_eq!(code_index.as_primitive::<UInt64Type>().values(), &[0, 1]);
    }
}
