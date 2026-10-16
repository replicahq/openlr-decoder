# API Reference

All classes and functions are available from the top-level `openlr_decoder` package:

```python
from openlr_decoder import (
    RoadNetwork,
    Decoder,
    DecoderConfig,
    DecodedPath,
    road_network_schema,
)
```

---

## `RoadNetwork`

A road network loaded into memory as a directed graph with a spatial index.

### `RoadNetwork.from_parquet(path)`

Load a road network from a Parquet file.

| Parameter | Type | Description |
|-----------|------|-------------|
| `path` | `str` | Path to the Parquet file |

**Returns:** `RoadNetwork`

**Raises:** `ValueError` if the file cannot be read or has an invalid schema.

```python
network = RoadNetwork.from_parquet("network.parquet")
```

### `RoadNetwork.from_arrow(data)`

Load a road network from Arrow-compatible data (zero-copy).

| Parameter | Type | Description |
|-----------|------|-------------|
| `data` | Arrow-compatible | Any object implementing `__arrow_c_stream__` or `__arrow_c_array__` |

Accepts PyArrow Tables, RecordBatches, RecordBatchReaders, and Polars DataFrames.

**Returns:** `RoadNetwork`

**Raises:** `ValueError` if the data has an invalid schema.

```python
import polars as pl
df = pl.read_parquet("network.parquet")
network = RoadNetwork.from_arrow(df)
```

### `RoadNetwork.edge_count`

*Property.* Number of edges in the network.

### `RoadNetwork.node_count`

*Property.* Number of nodes in the network.

---

## `DecoderConfig`

Configuration for the decoder. All parameters are optional and default to values tuned for cross-provider HERE-to-OSM decoding.

```python
config = DecoderConfig()  # all defaults
config = DecoderConfig(search_radius_m=150.0, max_candidates=15)
```

### Parameters

#### Search & Candidates

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `search_radius_m` | `float` | `100.0` | Radius in meters for finding candidate edges around each LRP |
| `max_bearing_diff` | `float` | `90.0` | Maximum bearing difference in degrees between LRP and candidate |
| `frc_tolerance` | `int` | `2` | Maximum FRC class difference allowed (0-7) |
| `max_candidates` | `int` | `10` | Maximum candidates to consider per LRP |
| `max_candidate_distance_m` | `float` | `35.0` | Maximum distance in meters from LRP to a candidate edge |

#### Path Search

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `length_tolerance` | `float` | `0.35` | Relative path length tolerance as a fraction (0.35 = 35%) |
| `absolute_length_tolerance` | `float` | `100.0` | Absolute path length tolerance in meters |
| `max_search_distance_factor` | `float` | `2.0` | A* search distance limit as multiple of expected DNP |

#### Scoring Weights

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `distance_weight` | `float` | `10.0` | Weight for distance score (higher = distance matters more) |
| `bearing_weight` | `float` | `0.2` | Weight for bearing difference score |
| `frc_weight` | `float` | `0.1` | Weight for FRC difference score |
| `fow_weight` | `float` | `0.1` | Weight for FOW compatibility score |

#### Cost Penalties

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `slip_road_cost_penalty` | `float` | `20.0` | A* cost penalty in meters for traversing slip roads (ramps/links) |
| `access_road_cost_penalty` | `float` | `10.0` | A* cost penalty in meters for traversing access/local roads |

All parameters are readable and writable after construction:

```python
config = DecoderConfig()
config.search_radius_m = 200.0
print(config.distance_weight)  # 10.0
```

---

## `Decoder`

The main decoder class. Decodes OpenLR base64 strings against a road network.

### `Decoder(network, config=None)`

| Parameter | Type | Description |
|-----------|------|-------------|
| `network` | `RoadNetwork` | The road network to decode against |
| `config` | `DecoderConfig \| None` | Optional configuration (defaults are used if not provided) |

```python
decoder = Decoder(network)
decoder = Decoder(network, DecoderConfig(search_radius_m=150.0))
```

### `Decoder.decode(openlr_base64)`

Decode a single OpenLR location reference.

| Parameter | Type | Description |
|-----------|------|-------------|
| `openlr_base64` | `str` | Base64-encoded OpenLR binary string |

**Returns:** [`DecodedPath`](#decodedpath)

**Raises:** `ValueError` with one of:

| Error message | Meaning |
|--------------|---------|
| `Invalid OpenLR: ...` | The base64 string couldn't be parsed as a valid OpenLR reference |
| `No matching roads found for location reference point N` | No candidate edges within search radius for LRP at index N |
| `No valid path found between points N and M` | A* search couldn't connect candidates for consecutive LRPs |
| `Path length mismatch: expected Xm, got Ym` | Best path length is outside tolerance of expected DNP |
| `Unsupported location type: ...` | Non-line location type (e.g., point, area) |

```python
try:
    result = decoder.decode("CwRbWyNG9RpsCQCb/jsboAD/6/+E")
    print(result.edge_ids, result.length)
except ValueError as e:
    print(f"Decode failed: {e}")
```

### `Decoder.decode_batch(openlr_codes, *, explode=False)`

Decode multiple OpenLR codes in parallel. Releases the Python GIL during computation.

| Parameter | Type | Description |
|-----------|------|-------------|
| `openlr_codes` | `list[str]` or Arrow string array | Base64-encoded OpenLR strings. Any object exposing the Arrow PyCapsule interface (`__arrow_c_array__` / `__arrow_c_stream__`) with a `string`, `large_string` or `string_view` type is read zero-copy — e.g. a PyArrow `Array`/`ChunkedArray` or a Polars `Series`. Null entries are reported as errors. |
| `explode` | `bool` | Return one row per (code, edge) instead of one row per code. Default `False`. |

Codes are decoded in chunks whose results are appended to the output as they finish, so peak memory is roughly the network plus one copy of the output. Prefer passing a Polars `Series` or PyArrow array over `.to_list()`, which materializes a Python `str` per code.

**Returns:** PyArrow `RecordBatch`. With `explode=False`, one row per code:

| Column | Type | Description |
|--------|------|-------------|
| `edge_ids` | `list<uint64>` | Edge IDs forming the decoded path (empty list on failure) |
| `length` | `float64` | Path length in meters (null on failure) |
| `positive_offset` | `float64` | Offset from first edge start to first LRP (null on failure) |
| `negative_offset` | `float64` | Offset from last LRP to last edge end (null on failure) |
| `positive_offset_fraction` | `float64` | Fraction along first edge (0-1) (null on failure) |
| `negative_offset_fraction` | `float64` | Fraction along last edge from end (0-1) (null on failure) |
| `primary_edge_id` | `uint64` | Edge covering most distance (null on failure) |
| `primary_edge_coverage` | `float64` | Distance covered by primary edge (null on failure) |
| `error` | `string` | Error message (null on success) |

With `explode=True`, one row per edge of each decoded path, plus a single row with null edge columns for each failed code — every input code appears at least once:

| Column | Type | Description |
|--------|------|-------------|
| `code_index` | `uint64` | Position of the code in `openlr_codes` |
| `edge_id` | `uint64` | Edge ID (null on failure) |
| `edge_position` | `uint32` | 0-based position of the edge in the path (null on failure) |
| `is_primary` | `bool` | Whether this edge is the path's `primary_edge_id` (null on failure) |
| `offset_fraction` | `float64` | Fraction (0-1) of this edge outside the location, i.e. trimmed off by `positive_offset`/`negative_offset`. Offsets are spread from each end of the path, so after a terminal-node snap the extension edges are `1.0` and the next edge in gets the remainder; a single-edge path gets both offsets; `0.0` for edges fully inside (null on failure) |
| `length` | `float64` | Path length in meters (null on failure) |
| `error` | `string` | Error message (null on success) |

```python
import polars as pl

codes = pl.Series(["CwRbWyNG9RpsCQCb/jsboAD/6/+E", ...])
batch = decoder.decode_batch(codes)
df = pl.from_arrow(batch)

# One row per matched edge, joined back to the input codes
edges = pl.from_arrow(decoder.decode_batch(codes, explode=True))
edges = edges.with_columns(openlr_code=codes.gather(edges["code_index"]))
```

---

## `DecodedPath`

Result of successfully decoding an OpenLR location reference.

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `edge_ids` | `list[int]` | Ordered list of edge IDs forming the path |
| `length` | `float` | Total path length in meters |
| `positive_offset` | `float` | Distance (m) from start of first edge to where the location begins |
| `negative_offset` | `float` | Distance (m) from where the location ends to end of last edge |
| `positive_offset_fraction` | `float` | Fraction (0.0-1.0) along first edge where the location begins |
| `negative_offset_fraction` | `float` | Fraction (0.0-1.0) along last edge, measured from the end |
| `primary_edge_id` | `int` | Edge ID that covers the most distance in the decoded path |
| `primary_edge_coverage` | `float` | Distance (m) covered by the primary edge |

### Understanding Offsets

The decoded path may extend beyond the actual OpenLR location at both ends. Offsets indicate how much to trim:

```
First edge                                          Last edge
|◄──────────────────────────────────────────────────────────►|
|  pos_offset  |◄── actual location ──►|  neg_offset  |
|◄────────────►|                       |◄────────────►|
```

- **positive_offset**: trim this much from the start of `edge_ids[0]`
- **negative_offset**: trim this much from the end of `edge_ids[-1]`
- The fraction variants express the same values as 0-1 fractions of the respective edge

---

## `road_network_schema()`

Returns the expected PyArrow Schema for road network input data.

**Returns:** `pyarrow.Schema`

```python
from openlr_decoder import road_network_schema

schema = road_network_schema()
print(schema)
# stableEdgeId: uint64
# startVertex: int64
# endVertex: int64
# startLat: float64
# startLon: float64
# endLat: float64
# endLon: float64
# highway: string
# lanes: int64
# geometry: binary
```
