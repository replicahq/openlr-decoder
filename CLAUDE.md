# CLAUDE.md

## What This Library Does

`openlr-decoder` is a Rust library (with Python bindings via PyO3/maturin) that decodes **HERE-encoded OpenLR** location references onto **OSM-based road networks**. OpenLR is a standard originally developed by TomTom for encoding locations on road networks, widely used by HERE for traffic data exchange.

The core problem: A HERE OpenLR code encodes a location as a sequence of **Location Reference Points (LRPs)** with attributes (lat/lon, bearing, FRC, FOW) using HERE's road classification ontology, but the decoder must find the actual edges on an OSM road network that correspond to this encoded location. This is challenging because:
- HERE and OSM have fundamentally different road classification systems (FRC/FOW don't map 1:1)
- HERE's network topology and geometry differ from OSM's
- Coordinates have limited precision (~1m accuracy)
- Road segmentation differs (one HERE link may correspond to multiple OSM edges and vice versa)

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
   - **Distance** (primary, weight 10.0): LRP should be very close to the correct edge
   - **Bearing** (weight 0.2): Edge direction should match LRP bearing (±90°, relaxed for cross-provider)
   - **FRC** (weight 0.1): Functional Road Class should roughly match (±2 levels, low weight since HERE→OSM FRC mapping is approximate)
   - **FOW** (weight 0.1): Form of Way compatibility via substitution score matrix

4. **Decoder** (`decoder.rs`): Uses bounded A* search to find paths between consecutive LRP candidate pairs, then selects the best overall path based on combined candidate scores and path length match.

## OpenLR Concepts

### Location Reference Points (LRPs)
Each LRP has:
- **Coordinate**: lat/lon position
- **Bearing**: direction of travel in degrees (0-360)
- **FRC**: Functional Road Class (0-indexed: 0=highest importance ... 4=lowest, for HERE)
- **FOW**: Form of Way (motorway, slip road, roundabout, etc.)
- **DNP**: Distance to Next Point (meters, only on non-terminal LRPs)
- **LFRCNP**: Lowest FRC to Next Point

### Functional Road Class (FRC)

**Two numbering systems:**
- **HERE**: Uses 1-indexed Functional Class (FC1–FC5), where FC1 is highest importance
- **OpenLR**: Encodes these as 0-indexed FRC values (0–7), where FRC0 is highest importance

**The mapping:** HERE's FC1→FRC0, FC2→FRC1, FC3→FRC2, FC4→FRC3, FC5→FRC4

OpenLR's binary format has 3 bits for FRC (values 0–7), but HERE only uses FRC0–FRC4:

```
FRC0 = Motorways, controlled-access highways (highest importance) [HERE: FC1]
FRC1 = Major routes for travel between/through cities            [HERE: FC2]
FRC2 = High volume roads interconnecting with major routes        [HERE: FC3]
FRC3 = Moderate speed roads between neighborhoods                 [HERE: FC4]
FRC4 = Local/residential roads (lowest importance)                [HERE: FC5]
FRC5–FRC7 = Not used by HERE
```

The loader maps OSM `highway` tags into the FRC0–FRC4 range:

| FRC Value | HERE FC | OSM highway tags |
|-----------|---------|------------------|
| FRC0 | FC1 | `motorway` |
| FRC1 | FC2 | `trunk` |
| FRC2 | FC3 | `primary` |
| FRC3 | FC4 | `secondary`, `motorway_link`, `trunk_link`, `primary_link` |
| FRC4 | FC5 | `tertiary`, `secondary_link`, `tertiary_link`, `unclassified`, `residential`, `living_street`, `service`, `track` |
| FRC7 | — | everything else (non-navigable, won't match any HERE LRP) |

### Form of Way (FOW)

FOW describes the physical form of a road, as defined by HERE/OpenLR:

```
0 = Undefined (no FOW information available)
1 = Motorway (controlled-access highway with no at-grade crossings)
2 = Multiple Carriageway (physically separated dual carriageway, not motorway)
3 = Single Carriageway (single roadway for both directions of traffic)
4 = Roundabout (circular junction where traffic flows one way)
5 = Traffic Square (open area with intersecting roads, not a roundabout)
6 = Slip Road (ramp or connector between roads at different levels)
7 = Other (none of the above)
```

The loader infers FOW from OSM tags: `junction=roundabout` → Roundabout, `*_link` → SlipRoad, `motorway` → Motorway, major roads with ≥4 lanes or `oneway=yes` → MultipleCarriageway, everything else → SingleCarriageway.

## Cross-Provider Decoding: HERE → OSM

This decoder is purpose-built for decoding **HERE-encoded OpenLR onto OSM road networks**. The OpenLR codes originate from HERE's map and use HERE's road classification ontology, but must be resolved against a structurally different OSM network.

Key design decisions driven by this:

1. **FRC/FOW have low scoring weight (0.1 each)**: HERE and OSM classify roads fundamentally differently. A HERE "first class road" may not align with the same OSM `highway` tag the mapping table expects. The decoder compensates by relying primarily on spatial proximity.

2. **Distance dominates scoring (weight 10.0)**: Since HERE LRPs are placed directly on roads, the closest matching OSM edge is almost always correct regardless of classification mismatches.

3. **Bearing tolerance is wide (±90°)**: HERE and OSM may digitize road geometry differently, especially at intersections. The wider tolerance prevents valid candidates from being filtered out.

4. **Length tolerance is generous**: 35% relative + 100m absolute tolerance handles differences in road segmentation and DNP quantization errors between HERE and OSM.

5. **Same-edge handling**: Short segments where both LRPs fall on the same OSM edge need special relaxed length checking.

## Known Deviations from the OpenLR Whitepaper (v1.5)

This decoder is tuned for HERE-encoded data and intentionally departs from the whitepaper
(`openlr-whitepaper_v1.5.pdf`, Chapter G §12) in several places. Some deviations follow HERE's
encoder behavior rather than the spec; others are unimplemented spec features. Keep these in mind
when interpreting results or when feeding in codes from a non-HERE encoder.

### HERE-isms (code matches HERE, not the spec)

1. **Last-LRP bearing direction.** Spec §5.4.2 says the last LRP's attributes describe the
   *incoming* line, and §5.2.4 measures its bearing from the LRP coordinate back along that line
   (worked example, Table 74: bearing 290° for a line travelling ~110°). HERE instead encodes the
   last LRP's bearing in the **direction of travel** — verified on all 47,931 codes in
   `qa/test_codes.csv` (last-LRP bearing within 30° of the LRP1→LRP2 chord in 100% of cases).
   `find_candidates` therefore compares *every* LRP against the edge's travel-direction bearing.
   Codes from a spec-conformant encoder (TomTom, the `openlr` Python/Java encoders) will have the
   last LRP ~180° off and fail the ±90° bearing filter with "No candidates found for LRP N".

2. **Binary offsets are ignored.** Spec §7.5.2 / §12.10 / §12.1 Step 7 trim the path by the
   encoded positive/negative offsets (a 1/256 fraction of the distance between the first two /
   last two LRPs). HERE always encodes them as 0 (0 of 47,931 corpus codes have a non-zero offset),
   so `decode_line` never reads `line.offsets`. **`DecodedPath.positive_offset` /
   `negative_offset` do not mean what the spec means**: they are the distance from the start of
   the first edge to the first LRP's projection (and from the last LRP's projection to the end of
   the last edge). Any code that does carry a non-zero offset will decode without trimming.

3. **FRC range.** Only FRC0–FRC4 appear in HERE data (HERE FC1–FC5); the loader maps non-navigable
   OSM ways to FRC7 so they never match. The corpus is ~80% FRC4, so FRC contributes little
   discrimination in practice.

### Spec features not implemented

4. **No cross-pair consistency for ≥3 LRPs.** §12.1 Step 5 notes that changing the candidate for
   an intermediate LRP requires re-solving the previous segment. `decode_line` solves each
   consecutive LRP pair independently; the edge chosen for LRP *i* as the *end* of segment
   (i-1, i) is not forced to be the *start* of segment (i, i+1). Concatenation only dedupes when
   both segments happen to pick the same edge. The corpus is 100% two-LRP codes, so this is
   untested on real data and may produce gaps or overlaps at intermediate LRPs.

5. **PointAlongLine returns a path, not a point.** §12.3 Step 7 resolves a point via the positive
   offset plus side-of-road / orientation. `decode_point_along_line` runs the line pipeline and
   returns the full path; offset, SOR and ORI are not applied or exposed.

6. **Bearing is measured on one geometry segment, not over BEARDIST.** §5.2.4 defines bearing to
   the point 20 m along the line. `spatial::bearing_at_projection` uses the bearing of the single
   segment containing the projection point, which is noisier on densely-noded OSM curves.

7. **FOW inference is partial.** `Fow::from_osm_tags` supports `junction=roundabout` and
   `oneway`, but the parquet schema has neither column, so the loader passes `None` for both.
   The network never contains `Roundabout`; `MultipleCarriageway` arises only from `lanes >= 4`.

8. **Unsupported location types.** GeoCoordinate, POI-with-access-point, Circle, Rectangle, Grid,
   Polygon and ClosedLine return `UnsupportedType`.

9. **Lengths are floats.** Spec Rule 2 says integer metres. Harmless given the tolerances.

### Implementation notes worth knowing

- **Shortest path is not strictly shortest.** A* adds soft penalties (slip road 20 m, access
  road 10 m) so route choice prefers main roads; reported `length` is the real, unpenalised length.
- **LFRCNP filter is widened** by `frc_tolerance` and exempts `SlipRoad` edges, since OSM
  `*_link` ways map to FRC3 even when they connect motorways. The spec allows this
  ("this value might be altered if the decoder anticipates different FRC values").
- **`max_candidate_distance_m` is not a per-LRP filter.** Only *one* LRP in the reference must
  have a candidate within this distance; other LRPs may match anything within `search_radius_m`.
  This tolerates one LRP falling outside network coverage.
- **Start/end edges contributing <3% of the path are dropped from `edge_ids`** but their partial
  length is still included in `length`, so the edge list can cover less than `length`.

## Testing & Debugging Bad Matches

### Visualization Web App

The `openlr_web` repo contains a FastAPI visualization app for debugging:

```bash
cd ~/openlr_web
uv run uvicorn app:app --reload
```

The app shows:
- LRP markers on the map with bearing/FRC/FOW info
- Decoded path highlighted in blue
- Background network edges (hover to see edge IDs)
- Success/failure status with error messages

### Common Failure Modes

1. **"No candidates found for LRP N"**
   - LRP coordinate is too far from any matching OSM edge (>35m)
   - Bearing mismatch (>90°) filtering out all candidates
   - FRC incompatibility (>±2 levels after HERE→OSM mapping)
   - Road simply doesn't exist in the OSM network
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

1. IMPORTANT: Never manually decode OpenLR references to LRPs using custom code: only use the Python or Rust libraries.
   - **CLI**: `uvx --with openlr python -m openlr <base64_openlr_code>`
   - **Python**: `import openlr; ref = openlr.binary_decode("<base64_openlr_code>")` — pass the base64 string directly (not raw bytes). Returns a `LineLocationReference` with `.points` (list of `LocationReferencePoint` with lon, lat, frc, fow, bear, lfrcnp, dnp) and `.poffs`/`.noffs`.
2. **Visualize the LRPs**: See where they land on the map
3. **Check candidate edges**: Are the correct roads being found?
4. **Verify connectivity**: Can A* find a path between candidate pairs?
5. **Compare path length**: Does actual length match DNP expectations?
6. **Inspect the network graph** with networkx to check connectivity, find shortest paths between vertices, or look up edges by ID:

```python
import networkx as nx
import polars as pl

df = pl.read_parquet("openlr_test_edges.parquet")

G = nx.from_pandas_edgelist(
    df,
    source="startVertex",
    target="endVertex",
    edge_attr=["stableEdgeId", "highway", "lanes"],
    create_using=nx.DiGraph,
)
```

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
cd ~/openlr-web
uv sync
uv run uvicorn app:app
```

## QA Benchmarking

The `qa/` directory contains tooling for measuring decode quality across a 48K-code corpus from the Kansas City metro area. It compares decoded OSM paths against HERE reference geometries using Hausdorff and Fréchet distance.

**IMPORTANT**: The benchmark must use `~/openlr-web/.venv/bin/python` directly — do NOT use `uv run` from the openlr-web directory, as it will reinstall openlr-decoder from PyPI and overwrite your local build.

### Workflow

```bash
# 1. Build the version you want to test and install into the openlr-web venv
maturin build --release -i python3.12
cd ~/openlr-web && uv pip install --reinstall ~/openlr-decoder/target/wheels/openlr_decoder-*-cp312-*.whl

# 2. Run benchmark (saves results keyed by commit hash)
~/openlr-web/.venv/bin/python qa/benchmark.py \
  --save qa/results/$(git rev-parse --short HEAD).parquet

# 3. Compare two runs
~/openlr-web/.venv/bin/python qa/benchmark.py \
  --diff qa/results/BEFORE.parquet qa/results/AFTER.parquet
```

### What the diff report shows

- **Winners/Losers**: codes where Hausdorff distance improved/worsened by >5m
- **Quality buckets**: good (<30m), ok (30–100m), bad (>100m) and transitions between them
- **Top regressions**: specific codes that got worse — inspect these in the visualization app
- Hausdorff/Fréchet p50/p90 aggregate trends

### Test corpus

`qa/test_codes.csv` contains ~48K OpenLR codes with HERE reference geometries (from `bqutils.geo.openlr_to_geography`), randomly sampled from `model-159019.michelin.arity_link_matches_usa_v2` within the KC bounding box (38.53–39.11°N, 95.15–94.47°W). Network: `openlr_test_edges.parquet` (458K edges).

## Parquet Schema (Network Input)

| Column | Type | Description |
|--------|------|-------------|
| `stableEdgeId` | uint64 | Unique edge identifier |
| `startOsmNode` | int64 | Start node ID (OSM node; resolves barrier splits) |
| `endOsmNode` | int64 | End node ID (OSM node; resolves barrier splits) |
| `startLat`, `startLon` | float64 | Start coordinates |
| `endLat`, `endLon` | float64 | End coordinates |
| `highway` | string | OSM highway tag |
| `lanes` | int64 | Lane count (optional, for FOW) |
| `geometry` | binary (WKB) | LineString geometry |
