# CLAUDE.md

## What This Library Does

`openlr-decoder` is a Rust library (with Python bindings via PyO3/maturin) that decodes OpenLR location references to road network paths. OpenLR is an open standard for encoding locations on road networks, commonly used for traffic data exchange between map providers.

The core problem: An OpenLR code encodes a location as a sequence of **Location Reference Points (LRPs)** with attributes (lat/lon, bearing, road class, form of way), but the decoder must find the actual edges on the *target* road network that correspond to this encoded location. This is challenging because:
- The encoder and decoder may use different map providers (e.g., HERE → OSM)
- Road classifications (FRC/FOW) don't map 1:1 between providers
- Coordinates have limited precision (~1m accuracy)
- Road network topology differs between maps

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Load parquet   │───▶│   RoadNetwork    │◀───│  SpatialIndex   │
│  (loader.rs)    │    │   (graph.rs)     │    │  (spatial.rs)   │
└─────────────────┘    └────────┬─────────┘    └────────┬────────┘
                                │                       │
                                ▼                       │
                       ┌─────────────────┐              │
                       │   Candidates    │◀─────────────┘
                       │ (candidates.rs) │
                       └────────┬────────┘
                                │
                                ▼
                       ┌─────────────────┐
                       │    Decoder      │
                       │  (decoder.rs)   │
                       └─────────────────┘
```

### Key Components

1. **RoadNetwork** (`graph.rs`): petgraph-based directed graph with `Node` (intersections) and `Edge` (road segments). Edges have FRC, FOW, bearing, and WKB geometry.

2. **SpatialIndex** (`spatial.rs`): R-tree index for fast radius queries. Wraps edge geometries in `EdgeEnvelope` for bounding box queries.

3. **Candidate Finding** (`candidates.rs`): For each LRP, finds nearby edges and scores them by:
   - **Distance** (primary): LRP should be very close to the correct edge
   - **Bearing**: Edge direction should match LRP bearing (±30°)
   - **FRC**: Functional Road Class should roughly match (±2 levels)
   - **FOW**: Form of Way must be compatible

4. **Decoder** (`decoder.rs`): Uses bounded A* search to find paths between consecutive LRP candidate pairs, then selects the best overall path based on combined candidate scores and path length match.

## OpenLR Concepts

### Location Reference Points (LRPs)
Each LRP has:
- **Coordinate**: lat/lon position
- **Bearing**: direction of travel in degrees (0-360)
- **FRC**: Functional Road Class (0=motorway ... 7=other)
- **FOW**: Form of Way (motorway, slip road, roundabout, etc.)
- **DNP**: Distance to Next Point (meters, only on non-terminal LRPs)
- **LFRCNP**: Lowest FRC to Next Point

### Functional Road Class (FRC)
```
FRC0 = Motorway/freeway
FRC1 = Major routes (trunk)
FRC2 = Regional routes (primary)
FRC3 = Local connecting (secondary, motorway_link, trunk_link)
FRC4 = Local roads (tertiary, service, unclassified)
FRC5 = Minor local (residential, track)
FRC6 = Low importance (living_street)
FRC7 = Other
```

### Form of Way (FOW)
```
0 = Undefined
1 = Motorway
2 = Multiple Carriageway
3 = Single Carriageway
4 = Roundabout
5 = Traffic Square
6 = Slip Road
7 = Other
```

## Cross-Provider Decoding Challenges

This decoder is optimized for decoding HERE-encoded OpenLR on OSM maps:

1. **FRC mappings are approximate**: HERE and OSM classify roads differently. The decoder uses loose FRC tolerance (±2) and low FRC weight in scoring.

2. **Distance dominates scoring**: Since LRPs are placed directly on roads, the closest matching edge is almost always correct.

3. **Length tolerance is generous**: 35% relative + 100m absolute tolerance handles digitization differences and DNP quantization errors.

4. **Same-edge handling**: Short segments where both LRPs fall on the same edge need special relaxed length checking.

## Testing & Debugging Bad Matches

### Visualization Web App

The `~/openlr_benchmark` repo contains a FastAPI visualization app for debugging:

```bash
cd ~/openlr_benchmark
uv run uvicorn app:app --reload
```

The app shows:
- LRP markers on the map with bearing/FRC/FOW info
- Decoded path highlighted in blue
- Background network edges (hover to see edge IDs)
- Success/failure status with error messages

### Common Failure Modes

1. **"No candidates found for LRP N"**
   - LRP coordinate is too far from any matching edge
   - Bearing mismatch (>30°) filtering out all candidates
   - FRC/FOW incompatibility
   - **Debug**: Check if road exists in network, verify bearing direction

2. **"No valid path found between LRPs"**
   - No connected path within length tolerance
   - Candidate edges don't connect (missing road in network)
   - One-way restrictions blocking path
   - **Debug**: Examine candidate edges, check if they're connected

3. **Path matches wrong road**
   - Parallel roads (e.g., service road vs main road)
   - Scoring weights may need tuning
   - **Debug**: Compare candidate scores, adjust weights

### Debugging Approach

1. **Visualize the LRPs**: See where they land on the map
2. **Check candidate edges**: Are the correct roads being found?
3. **Verify connectivity**: Can A* find a path between candidate pairs?
4. **Compare path length**: Does actual length match DNP expectations?

### Configuration Tuning

```python
from openlr_decoder import DecoderConfig

config = DecoderConfig(
    search_radius=100.0,      # Increase if LRPs far from roads
    max_candidates=10,        # More candidates = more path options
    length_tolerance=0.35,    # 35% relative tolerance
    frc_tolerance=2,          # Allow ±2 FRC class difference
)
```

## Building & Testing

Use `uv` for Python environment management.

```bash
# Rust library
cargo build --release
cargo test

# Python bindings (into current venv)
maturin develop --release

# Run visualization app
cd ~/openlr_benchmark
uv sync
uv run uvicorn app:app
```

## Parquet Schema (Network Input)

| Column | Type | Description |
|--------|------|-------------|
| `stableEdgeId` | uint64 | Unique edge identifier |
| `startVertex` | int64 | Start node ID |
| `endVertex` | int64 | End node ID |
| `startLat`, `startLon` | float64 | Start coordinates |
| `endLat`, `endLon` | float64 | End coordinates |
| `highway` | string | OSM highway tag |
| `lanes` | int64 | Lane count (optional, for FOW) |
| `geometry` | binary (WKB) | LineString geometry |
