# openlr-decoder

[![PyPI](https://img.shields.io/pypi/v/openlr-decoder)](https://pypi.org/project/openlr-decoder/)
[![Python](https://img.shields.io/pypi/pyversions/openlr-decoder)](https://pypi.org/project/openlr-decoder/)
[![License](https://img.shields.io/github/license/replicahq/openlr-decoder)](https://github.com/replicahq/openlr-decoder/blob/main/LICENSE)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://replicahq.github.io/openlr-decoder)

Fast OpenLR location reference decoder for Python. Decodes HERE-encoded OpenLR onto OSM-based road networks using a Rust backend.

**[Documentation](https://replicahq.github.io/openlr-decoder)**

## Installation

```bash
pip install openlr-decoder
```

## Quick Start

```python
from openlr_decoder import RoadNetwork, Decoder

# Load road network from Parquet file
network = RoadNetwork.from_parquet("network.parquet")
decoder = Decoder(network)

# Decode a single OpenLR code
result = decoder.decode("CwRbWyNG9RpsCQCb/jsboAD/6/+E")
print(result.edge_ids, result.length)

# Batch decode (parallel, returns Arrow RecordBatch)
codes = ["CwRbWyNG9RpsCQCb/jsboAD/6/+E", "C7yVyRun2SOPAAAD/94jDw=="]
batch = decoder.decode_batch(codes)

# Convert to Polars
import polars as pl
df = pl.from_arrow(batch)
```

## Zero-Copy Loading

Load networks from Polars DataFrames or PyArrow Tables with no serialization overhead:

```python
import polars as pl
from openlr_decoder import RoadNetwork

df = pl.read_parquet("network.parquet")
network = RoadNetwork.from_arrow(df)  # zero-copy
```

## Parquet Schema

The road network Parquet file must contain these columns:

| Column | Type | Required | Description |
|--------|------|----------|-------------|
| `stableEdgeId` | uint64 | Yes | Unique edge identifier |
| `startVertex` | int64 | Yes | Start node ID |
| `endVertex` | int64 | Yes | End node ID |
| `startLat` | float64 | Yes | Start point latitude |
| `startLon` | float64 | Yes | Start point longitude |
| `endLat` | float64 | Yes | End point latitude |
| `endLon` | float64 | Yes | End point longitude |
| `highway` | string | Yes | OSM highway tag |
| `lanes` | int64 | No | Lane count (for FOW inference) |
| `geometry` | binary (WKB) | No | LineString geometry |

## Configuration

```python
from openlr_decoder import DecoderConfig

config = DecoderConfig(
    search_radius_m=100.0,        # meters, default 100
    max_candidates=10,             # per LRP, default 10
    length_tolerance=0.35,         # fraction, default 0.35
    frc_tolerance=2,               # FRC class difference, default 2
    max_candidate_distance_m=35.0, # meters, default 35
)
decoder = Decoder(network, config)
```

See the [Configuration Guide](https://replicahq.github.io/openlr-decoder/configuration/) for all 14 parameters.

## License

MIT
