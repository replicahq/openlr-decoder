# openlr-decoder

**Fast OpenLR location reference decoder for Python.**

[![PyPI](https://img.shields.io/pypi/v/openlr-decoder)](https://pypi.org/project/openlr-decoder/)
[![Python](https://img.shields.io/pypi/pyversions/openlr-decoder)](https://pypi.org/project/openlr-decoder/)
[![License](https://img.shields.io/github/license/replicahq/openlr-decoder)](https://github.com/replicahq/openlr-decoder/blob/main/LICENSE)
[![CI](https://img.shields.io/github/actions/workflow/status/replicahq/openlr-decoder/ci.yml?label=CI)](https://github.com/replicahq/openlr-decoder/actions/workflows/ci.yml)

Decode HERE-encoded OpenLR location references onto OSM-based road networks. Built in Rust with Python bindings via PyO3 for high performance.

## Features

- **Fast**: Rust core with parallel batch decoding via Rayon
- **Zero-copy data loading**: Pass Arrow tables, Polars DataFrames, or PyArrow RecordBatches directly — no serialization overhead
- **Cross-provider**: Tuned for decoding HERE-encoded OpenLR onto OSM road networks
- **Configurable**: 14 parameters for scoring weights, tolerances, and search behavior
- **Batch-friendly**: `decode_batch()` returns an Arrow RecordBatch for seamless integration with Polars, Pandas, and DuckDB

## Quick Start

```bash
pip install openlr-decoder
```

```python
from openlr_decoder import RoadNetwork, Decoder

network = RoadNetwork.from_parquet("network.parquet")
decoder = Decoder(network)

result = decoder.decode("CwRbWyNG9RpsCQCb/jsboAD/6/+E")
print(result.edge_ids)    # [12345, 12346, 12347]
print(result.length)       # 542.3
```

## Next Steps

- [Getting Started](getting-started.md) — installation, loading data, single and batch decoding
- [OpenLR Concepts](concepts.md) — background on the OpenLR standard and cross-provider challenges
- [Preparing Network Data](preparing-network-data.md) — parquet schema and data sourcing
- [API Reference](api-reference.md) — complete reference for all classes and functions
- [Configuration Guide](configuration.md) — tuning the decoder for your use case
- [Troubleshooting](troubleshooting.md) — common errors and debugging workflow
