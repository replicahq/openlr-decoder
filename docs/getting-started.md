# Getting Started

## Installation

```bash
pip install openlr-decoder
```

The package ships pre-built wheels for Linux (x86_64), macOS (x86_64/arm64), and Windows. No Rust toolchain needed.

## Loading a Road Network

The decoder needs an OSM-based road network stored as a directed graph. You can load it from a Parquet file or pass Arrow-compatible data directly.

=== "Parquet file"

    ```python
    from openlr_decoder import RoadNetwork

    network = RoadNetwork.from_parquet("network.parquet")
    print(f"Loaded {network.edge_count} edges, {network.node_count} nodes")
    ```

=== "Polars (zero-copy)"

    ```python
    import polars as pl
    from openlr_decoder import RoadNetwork

    df = pl.read_parquet("network.parquet")
    network = RoadNetwork.from_arrow(df)
    ```

=== "PyArrow"

    ```python
    import pyarrow.parquet as pq
    from openlr_decoder import RoadNetwork

    table = pq.read_table("network.parquet")
    network = RoadNetwork.from_arrow(table)
    ```

See [Preparing Network Data](preparing-network-data.md) for the required parquet schema.

## Decoding a Single OpenLR Code

```python
from openlr_decoder import RoadNetwork, Decoder

network = RoadNetwork.from_parquet("network.parquet")
decoder = Decoder(network)

result = decoder.decode("CwRbWyNG9RpsCQCb/jsboAD/6/+E")
```

The returned [`DecodedPath`](api-reference.md#decodedpath) contains:

| Property | Description |
|----------|-------------|
| `edge_ids` | List of edge IDs forming the decoded path |
| `length` | Total path length in meters |
| `positive_offset` | Distance (m) from start of first edge to the first LRP projection |
| `negative_offset` | Distance (m) from last LRP projection to end of last edge |
| `positive_offset_fraction` | Fraction (0-1) along first edge where the path starts |
| `negative_offset_fraction` | Fraction (0-1) along last edge measured from end |
| `primary_edge_id` | Edge ID covering the most distance in the path |
| `primary_edge_coverage` | Distance (m) covered by the primary edge |

If decoding fails, a `ValueError` is raised with a descriptive message.

## Batch Decoding

For decoding many OpenLR codes, `decode_batch()` is much faster — it processes codes in parallel using all available CPU cores and returns results as an Arrow RecordBatch.

```python
codes = ["CwRbWyNG9RpsCQCb/jsboAD/6/+E", "C7yVyRun2SOPAAAD/94jDw==", ...]
batch = decoder.decode_batch(codes)
```

The returned RecordBatch has the same fields as `DecodedPath` plus an `error` column (null on success, error message on failure).

### Converting Results

=== "Polars"

    ```python
    import polars as pl

    df = pl.from_arrow(batch)
    successful = df.filter(pl.col("error").is_null())
    failed = df.filter(pl.col("error").is_not_null())
    print(f"{len(successful)} decoded, {len(failed)} failed")
    ```

=== "Pandas"

    ```python
    df = batch.to_pandas()
    ```

## Custom Configuration

Pass a [`DecoderConfig`](api-reference.md#decoderconfig) to tune decoder behavior:

```python
from openlr_decoder import Decoder, DecoderConfig

config = DecoderConfig(
    search_radius_m=150.0,      # wider search for rural areas
    max_candidates=15,           # more candidates per LRP
    length_tolerance=0.40,       # 40% path length tolerance
)
decoder = Decoder(network, config)
```

See the [Configuration Guide](configuration.md) for detailed tuning advice.
