"""
OpenLR decoder for Python.

Fast OpenLR location reference decoding using a Rust backend.

Example:
    >>> from openlr_decoder import RoadNetwork, Decoder
    >>> network = RoadNetwork.from_parquet("/path/to/network.parquet")
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

from openlr_decoder._openlr_decoder import (
    RoadNetwork,
    Decoder,
    DecoderConfig,
    DecodedPath,
)

__all__ = [
    "RoadNetwork",
    "Decoder",
    "DecoderConfig",
    "DecodedPath",
]

__version__ = "0.1.1"
