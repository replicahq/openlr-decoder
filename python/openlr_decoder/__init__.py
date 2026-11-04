"""
Decode HERE-encoded OpenLR line locations onto OSM road networks.

Example:
    >>> from openlr_decoder import RoadNetwork, Decoder
    >>> network = RoadNetwork.from_parquet("/path/to/network.parquet")

    # Or from any Arrow data, such as a PyArrow Table or Polars DataFrame
    >>> import polars as pl
    >>> network = RoadNetwork.from_arrow(pl.read_parquet("/path/to/network.parquet"))

    >>> decoder = Decoder(network)
    >>> result = decoder.decode("CwRbWyNG9RpsCQCb/jsboAD/6/+E")
    >>> print(result.edge_ids, result.length)

    # Many codes in parallel, returned as a pyarrow.RecordBatch
    >>> batch = decoder.decode_batch(pl.Series(codes))
    >>> df = pl.from_arrow(batch)
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
