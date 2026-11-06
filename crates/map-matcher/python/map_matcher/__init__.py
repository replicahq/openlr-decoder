"""
HMM map matching of GPS traces onto a road network.

Example:
    >>> from map_matcher import RoadNetwork, Matcher, MatchConfig
    >>> network = RoadNetwork.from_parquet("/path/to/network.parquet")
    >>> matcher = Matcher(network, MatchConfig(sigma_m=10.0))

    # pings: trace_id, time, lat, lon, and optionally speed (m/s) and heading
    >>> segments = matcher.match_traces(pings)
    >>> import polars as pl
    >>> pl.from_arrow(segments)
"""

from map_matcher._map_matcher import MatchConfig, Matcher, RoadNetwork, road_network_schema

__all__ = ["MatchConfig", "Matcher", "RoadNetwork", "road_network_schema"]
