# Preparing Network Data

## Parquet Schema

The road network must be provided as a Parquet file (or Arrow-compatible data) with the following columns:

| Column | Type | Required | Description |
|--------|------|----------|-------------|
| `stableEdgeId` | uint64 | Yes | Unique edge identifier |
| `startVertex` | int64 | Yes | Start node ID |
| `endVertex` | int64 | Yes | End node ID |
| `startLat` | float64 | Yes | Start point latitude |
| `startLon` | float64 | Yes | Start point longitude |
| `endLat` | float64 | Yes | End point latitude |
| `endLon` | float64 | Yes | End point longitude |
| `highway` | string | Yes | OSM `highway` tag value |
| `lanes` | int64 | No | Lane count (used for FOW inference) |
| `geometry` | binary (WKB) | No | LineString geometry in WKB format |

You can inspect the expected schema programmatically:

```python
from openlr_decoder import road_network_schema

schema = road_network_schema()
print(schema)
```

## Column Details

### `stableEdgeId`

A unique uint64 identifier for each directed edge. These IDs are returned in decoded paths (`DecodedPath.edge_ids`) and in batch results.

### `startVertex` / `endVertex`

Node IDs defining the directed edge. The decoder builds a graph from these, so edges sharing a vertex will be connected. For bidirectional roads, include two edges (one per direction) with swapped start/end vertices.

### Coordinates

`startLat`, `startLon`, `endLat`, `endLon` define the endpoints. These are used for:

- Building the spatial index (for finding candidate edges near LRP coordinates)
- Computing edge bearings when no geometry column is present
- Computing approximate edge lengths when no geometry is present

### `highway`

The OSM `highway` tag value (e.g., `motorway`, `primary`, `residential`). The decoder maps this to FRC and FOW:

| FRC | OSM highway tags |
|-----|-----------------|
| 0 | `motorway` |
| 1 | `trunk` |
| 2 | `primary` |
| 3 | `secondary`, `motorway_link`, `trunk_link`, `primary_link` |
| 4 | `tertiary`, `secondary_link`, `tertiary_link`, `unclassified`, `residential`, `living_street`, `service`, `track` |

### `lanes`

Optional. When present, used to infer whether a road is a Multiple Carriageway (FOW=2). Roads with 4+ lanes or `oneway=yes` are classified as multiple carriageway; otherwise as single carriageway.

### `geometry`

Optional but recommended. A WKB-encoded LineString for accurate edge length computation and bearing calculation. Without it, the decoder uses straight-line distance between start/end coordinates and computes bearing from those two points.

## Loading Data

### From Parquet

The simplest approach — reads and builds the network in one call:

```python
from openlr_decoder import RoadNetwork

network = RoadNetwork.from_parquet("network.parquet")
```

### From Arrow (Zero-Copy)

For workflows where data is already in memory, `from_arrow()` accepts any object implementing the [Arrow PyCapsule interface](https://arrow.apache.org/docs/format/CDataInterface/PyCapsuleInterface.html):

```python
import polars as pl
from openlr_decoder import RoadNetwork

df = pl.read_parquet("network.parquet")
# Filter or transform as needed
df = df.filter(pl.col("highway") != "service")
network = RoadNetwork.from_arrow(df)  # zero-copy transfer
```

This works with PyArrow Tables, RecordBatches, RecordBatchReaders, and Polars DataFrames.

## Sourcing OSM Data

To build a network Parquet file from OpenStreetMap:

1. **Download an OSM extract** from [Geofabrik](https://download.geofabrik.de/) or [BBBike](https://extract.bbbike.org/) for your region
2. **Extract road edges** using tools like [osmnx](https://osmnx.readthedocs.io/), osmium, or a custom pipeline
3. **Export to Parquet** with the schema above, ensuring:
    - Bidirectional roads have two directed edges
    - WKB geometries follow edge direction (start → end)
    - `highway` values match standard OSM tagging
