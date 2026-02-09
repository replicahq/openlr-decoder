# Changelog

## 0.1.11

- Configurable scoring weights exposed in Python `DecoderConfig`
- FRC5 residential road mapping with A* access road cost penalty
- Slip road cost penalty for A* path search
- Offset fractions added to `DecodedPath`

## 0.1.10

- LFRCNP path constraint now uses actual traversed edges

## 0.1.9

- Parallel processing in Parquet loader for faster network loading
- Memory and performance optimizations in loader

## 0.1.8

- Improved LFRCNP (Lowest FRC to Next Point) handling for cross-provider decoding
- Two-pass LFRCNP filtering to prevent mixed-class paths
- Slip road exemption in LFRCNP filtering

## 0.1.7

- Reduced memory usage by eliminating geometry duplication and dropping build-only maps

## 0.1.6

- LFRCNP filtering added to path search

## 0.1.5

- FRC mapping aligned with HERE's 5-level Functional Class (FC1-FC5)
- Bearing tolerance widened from 30 degrees to 90 degrees for cross-provider decoding

## 0.1.4

- Offset units fixed: fractions correctly converted to meters

## 0.1.3

- Cross-provider decoding improved with relaxed FOW tolerance and distance threshold

## 0.1.2

- `from_arrow()` interface added with WKT and GeoArrow support for zero-copy loading

## 0.1.1

- Primary link match (edge with most coverage) exposed in decoded path results
