# openlr-decoder

Fast OpenLR location reference decoder for Python and Rust.

## Installation

```bash
pip install openlr-decoder
```

## Usage

```python
from openlr_decoder import RoadNetwork, Decoder

# Load road network from Parquet file
network = RoadNetwork.from_parquet("edges.parquet")
decoder = Decoder(network)

# Decode a single OpenLR code
result = decoder.decode("C7yVyRun2SOPAAAD/94jDw==")
print(result.edge_ids, result.length)

# Batch decode (parallel processing)
codes = ["C7yVyRun2SOPAAAD/94jDw==", "CwRbWyNG9RpsCQCb/jsboAD/6/+E"]
results = decoder.decode_batch(codes)
for path, error in results:
    if path:
        print(path.edge_ids, path.length)
    else:
        print(f"Decode error: {error}")
```

## Parquet Schema

The road network Parquet file must contain these columns:

| Column | Type | Description |
|--------|------|-------------|
| `edge_id` | uint64 | Unique edge identifier |
| `start_node` | int64 | Start node identifier |
| `end_node` | int64 | End node identifier |
| `length` | float64 | Edge length in meters |
| `frc` | int32 | Functional road class (0-7) |
| `fow` | int32 | Form of way (0-7) |
| `geometry` | binary (WKB) | LineString geometry |

## Configuration

```python
from openlr_decoder import DecoderConfig

config = DecoderConfig(
    search_radius=30.0,        # meters, default 30
    max_candidates=5,          # per LRP, default 5
    length_tolerance=0.35,     # fraction, default 0.35
    frc_tolerance=2,           # FRC difference, default 2
    fow_tolerance=2,           # FOW difference, default 2
)
decoder = Decoder(network, config)
```

## License

MIT
