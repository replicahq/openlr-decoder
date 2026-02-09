# OpenLR Concepts

## What is OpenLR?

[OpenLR](https://www.openlr-association.com/) is an open standard for encoding locations on road networks. Originally developed by TomTom, it is widely used by HERE for traffic data exchange. An OpenLR **location reference** encodes a road segment as a compact binary string (typically represented as base64) that can be decoded against any compatible road network.

## Location Reference Points (LRPs)

Each OpenLR location reference consists of a sequence of **Location Reference Points** (LRPs). The first and last LRPs mark the start and end of the location. Intermediate LRPs provide waypoints that guide the decoder through the correct path.

Each LRP carries these attributes:

| Attribute | Description |
|-----------|-------------|
| **Coordinate** | Latitude/longitude position on the road |
| **Bearing** | Direction of travel in degrees (0-360) |
| **FRC** | Functional Road Class — importance level of the road (0-7) |
| **FOW** | Form of Way — physical type of road (motorway, roundabout, etc.) |
| **DNP** | Distance to Next Point — meters to the next LRP (non-terminal LRPs only) |
| **LFRCNP** | Lowest FRC to Next Point — minimum road importance on the path to the next LRP |

## Functional Road Class (FRC)

FRC ranks roads by importance on a scale from 0 (most important) to 7 (least important).

HERE uses a 1-indexed system called Functional Class (FC1-FC5), which maps to OpenLR's 0-indexed FRC:

| FRC | HERE FC | Description | OSM `highway` tags |
|-----|---------|-------------|-------------------|
| 0 | FC1 | Motorways, controlled-access highways | `motorway` |
| 1 | FC2 | Major routes between/through cities | `trunk` |
| 2 | FC3 | High-volume roads connecting to major routes | `primary` |
| 3 | FC4 | Moderate-speed roads between neighborhoods | `secondary`, `*_link` |
| 4 | FC5 | Local and residential roads | `tertiary`, `residential`, `unclassified`, `service` |
| 5-7 | — | Not used by HERE | — |

!!! note
    OpenLR's binary format allocates 3 bits for FRC (values 0-7), but HERE only uses FRC 0-4. The decoder maps OSM `highway` tags into this range.

## Form of Way (FOW)

FOW describes the physical form of a road:

| FOW | Type | Description |
|-----|------|-------------|
| 0 | Undefined | No form of way information |
| 1 | Motorway | Controlled-access highway, no at-grade crossings |
| 2 | Multiple Carriageway | Physically separated dual carriageway |
| 3 | Single Carriageway | Single roadway for both directions |
| 4 | Roundabout | Circular junction with one-way flow |
| 5 | Traffic Square | Open area with intersecting roads |
| 6 | Slip Road | Ramp or connector between roads |
| 7 | Other | None of the above |

The decoder infers FOW from OSM tags: `junction=roundabout` becomes Roundabout, `*_link` tags become Slip Road, `motorway` becomes Motorway, and so on.

## Cross-Provider Decoding

This decoder is specifically designed for decoding **HERE-encoded OpenLR onto OSM road networks**. This is fundamentally a cross-provider problem: the OpenLR codes were created using HERE's road network, but must be resolved against a structurally different OSM network.

Key challenges:

- **Different classification systems**: HERE's Functional Class doesn't map 1:1 to OSM's `highway` tag. A road that HERE classifies as FC3 might be tagged as `secondary` or `primary` in OSM.

- **Different network topology**: HERE and OSM segment roads differently. One HERE link may correspond to multiple OSM edges (or vice versa). Intersections may be modeled at different locations.

- **Different geometry**: Road centerlines are digitized independently, so bearing and position can differ by several meters.

The decoder handles these challenges by:

1. **Weighting distance heavily** (10x) over road classification attributes (0.1x each) — the closest OSM edge to an LRP coordinate is almost always correct
2. **Using wide bearing tolerance** (±90°) to avoid filtering valid candidates due to geometry differences
3. **Allowing FRC mismatches** (±2 levels) since classifications won't match exactly
4. **Generous length tolerance** (35% + 100m) to handle segmentation and quantization differences

## The Decoding Algorithm

At a high level, the decoder:

1. **Parses** the base64 OpenLR string into a sequence of LRPs
2. **Finds candidates** for each LRP — nearby edges scored by distance, bearing, FRC, and FOW
3. **Searches for paths** between consecutive LRP candidate pairs using bounded A* search
4. **Selects the best path** based on combined candidate scores and how well the path length matches the expected DNP
5. **Computes offsets** showing where the location starts and ends within the first/last edges
