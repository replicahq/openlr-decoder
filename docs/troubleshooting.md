# Troubleshooting

## Common Errors

### `No matching roads found for location reference point N`

The decoder couldn't find any candidate edges near LRP index N.

**Possible causes:**

- The road doesn't exist in your OSM network extract
- The LRP coordinate is more than `max_candidate_distance_m` (default 35m) from any edge
- All nearby edges were filtered by bearing (>`max_bearing_diff`) or FRC tolerance

**Fixes:**

- Increase `max_candidate_distance_m` and `search_radius_m`
- Widen `max_bearing_diff` (default is already 90°)
- Increase `frc_tolerance` (default 2)
- Verify the road exists in your network data

### `No valid path found between points N and M`

A* search couldn't find a connected path between candidate edges for consecutive LRPs.

**Possible causes:**

- Missing road segments in the network (gap in connectivity)
- One-way restrictions blocking the path
- Search distance limit too tight

**Fixes:**

- Increase `max_search_distance_factor` (default 2.0)
- Increase `max_candidates` to try more start/end edge combinations
- Check network connectivity between the LRP locations

### `Path length mismatch: expected Xm, got Ym`

A path was found but its length doesn't match the expected distance (DNP) within tolerance.

**Fixes:**

- Increase `length_tolerance` (default 0.35 = 35%)
- Increase `absolute_length_tolerance` (default 100m)
- This often indicates the OSM network routes differently than HERE's

### `Invalid OpenLR: ...`

The base64 string couldn't be parsed as a valid OpenLR binary.

**Fixes:**

- Verify the string is valid base64
- Confirm it's a HERE-encoded OpenLR (not TomTom or other provider)
- Check for truncation or encoding issues

### `Unsupported location type: ...`

The OpenLR reference encodes a non-line location (e.g., point or area).

This decoder only supports **line locations**. Point and area locations are not implemented.

## Debugging Workflow

### 1. Inspect the OpenLR code

Decode the raw LRP attributes:

```bash
uvx --with openlr python -m openlr CwRbWyNG9RpsCQCb/jsboAD/6/+E
```

This shows the coordinates, bearing, FRC, FOW, and DNP for each LRP without needing a road network.

### 2. Check your network coverage

Verify edges exist near the LRP coordinates:

```python
import polars as pl

df = pl.read_parquet("network.parquet")

# Check for edges near a coordinate (lat=40.1, lon=-74.5)
lat, lon = 40.1, -74.5
nearby = df.filter(
    ((pl.col("startLat") - lat).abs() < 0.001) &
    ((pl.col("startLon") - lon).abs() < 0.001)
)
print(nearby.select("stableEdgeId", "highway", "startLat", "startLon"))
```

### 3. Inspect network connectivity

Use NetworkX to verify paths exist between locations:

```python
import networkx as nx
import polars as pl

df = pl.read_parquet("network.parquet")
G = nx.from_pandas_edgelist(
    df.to_pandas(),
    source="startVertex",
    target="endVertex",
    edge_attr=["stableEdgeId", "highway"],
    create_using=nx.DiGraph,
)

# Check if a path exists between two vertices
start_vertex, end_vertex = 12345, 67890
print(nx.has_path(G, start_vertex, end_vertex))
```

### 4. Try relaxed configuration

If the default config fails, try relaxed settings to see if a valid path exists at all:

```python
from openlr_decoder import Decoder, DecoderConfig

config = DecoderConfig(
    search_radius_m=200.0,
    max_candidate_distance_m=75.0,
    max_candidates=20,
    length_tolerance=0.50,
    absolute_length_tolerance=200.0,
)
decoder = Decoder(network, config)
result = decoder.decode(code)
```

If this succeeds, gradually tighten parameters to find the right balance.

## Performance Tips

- **Use `decode_batch()`** for multiple codes — it parallelizes across all CPU cores and avoids Python overhead per decode
- **Use `from_arrow()`** with Polars DataFrames for zero-copy network loading (faster than `from_parquet()` if data is already in memory)
- **Filter your network** to the relevant region before loading — fewer edges means faster spatial queries
- **Batch size**: `decode_batch()` scales well to hundreds of thousands of codes
