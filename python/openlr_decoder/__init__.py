"""
OpenLR decoder for Python.

Fast OpenLR location reference decoding using a Rust backend.

Example:
    >>> from openlr_decoder import RoadNetwork, Decoder

    # Load from Parquet file
    >>> network = RoadNetwork.from_parquet("/path/to/network.parquet")

    # Or load from Arrow data (zero-copy, works with PyArrow and Polars)
    >>> import pyarrow.parquet as pq
    >>> table = pq.read_table("/path/to/network.parquet")
    >>> network = RoadNetwork.from_arrow(table)

    # With Polars (also zero-copy)
    >>> import polars as pl
    >>> df = pl.read_parquet("/path/to/network.parquet")
    >>> network = RoadNetwork.from_arrow(df)

    >>> decoder = Decoder(network)

    # Single decode
    >>> result = decoder.decode("CwRbWyNG9RpsCQCb/jsboAD/6/+E")
    >>> print(result.edge_ids, result.length)

    # Batch decode (parallel, much faster for many codes)
    >>> results = decoder.decode_batch(codes)
    >>> for path, error in results:
    ...     if path:
    ...         print(path.length)
"""

from importlib.metadata import version

from openlr_decoder._openlr_decoder import (
    RoadNetwork,
    Decoder,
    DecoderConfig,
    DecodedPath,
    road_network_schema,
)

__all__ = [
    "RoadNetwork",
    "Decoder",
    "DecoderConfig",
    "DecodedPath",
    "road_network_schema",
    "__version__",
]

__version__ = version("openlr-decoder")
