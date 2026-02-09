# Configuration Guide

The decoder ships with defaults tuned for cross-provider decoding (HERE-encoded OpenLR onto OSM networks). This page explains when and how to adjust them.

## Default Rationale

| Parameter | Default | Why |
|-----------|---------|-----|
| `distance_weight` | 10.0 | Spatial proximity is the strongest signal when networks differ |
| `bearing_weight` | 0.2 | HERE/OSM digitize roads differently — wide tolerance needed |
| `frc_weight` | 0.1 | Road classification systems don't map 1:1 |
| `fow_weight` | 0.1 | Same reason as FRC |
| `max_bearing_diff` | 90° | Cross-provider geometry differences can be large |
| `frc_tolerance` | 2 | Allow ±2 FRC class difference |
| `length_tolerance` | 0.35 | 35% handles segmentation and quantization differences |
| `absolute_length_tolerance` | 100m | Catches short segments where percentages are too strict |
| `slip_road_cost_penalty` | 20m | Prefer main roads over ramps when both are candidates |
| `access_road_cost_penalty` | 10m | Prefer classified roads over residential/service |

## When to Adjust

### Low match rates in rural areas

Rural networks may have sparser OSM coverage, placing LRP coordinates further from any edge.

```python
config = DecoderConfig(
    search_radius_m=200.0,           # wider search (default 100)
    max_candidate_distance_m=75.0,   # accept more distant candidates (default 35)
    max_candidates=20,               # more options per LRP (default 10)
)
```

### Wrong roads selected (parallel road problem)

When the decoder picks a frontage road instead of the highway (or vice versa), tighten classification scoring:

```python
config = DecoderConfig(
    frc_weight=0.5,        # increase from 0.1
    fow_weight=0.3,        # increase from 0.1
    bearing_weight=0.5,    # increase from 0.2
)
```

### Path length mismatches

If you're seeing `Path length mismatch` errors, relax tolerance:

```python
config = DecoderConfig(
    length_tolerance=0.50,             # 50% relative (default 35%)
    absolute_length_tolerance=200.0,   # 200m absolute (default 100)
)
```

### Same-provider decoding

When both the encoder and decoder use the same network (or very similar networks), you can tighten parameters for better precision:

```python
config = DecoderConfig(
    search_radius_m=50.0,
    max_candidate_distance_m=20.0,
    max_bearing_diff=45.0,
    frc_tolerance=1,
    length_tolerance=0.20,
    absolute_length_tolerance=50.0,
    frc_weight=0.5,
    fow_weight=0.3,
)
```

### Slip road avoidance

If the decoder is routing through ramps when it shouldn't, increase the penalty:

```python
config = DecoderConfig(
    slip_road_cost_penalty=50.0,   # default 20
)
```

## Example Configurations

### Cross-provider (default)

```python
config = DecoderConfig()  # all defaults are tuned for this
```

### Relaxed (maximize match rate)

```python
config = DecoderConfig(
    search_radius_m=200.0,
    max_candidate_distance_m=75.0,
    max_candidates=20,
    max_bearing_diff=120.0,
    frc_tolerance=3,
    length_tolerance=0.50,
    absolute_length_tolerance=200.0,
)
```

!!! warning
    Relaxed settings increase match rates but may produce less accurate paths. Use these when coverage matters more than precision.

### Strict (maximize accuracy)

```python
config = DecoderConfig(
    search_radius_m=50.0,
    max_candidate_distance_m=20.0,
    max_candidates=5,
    max_bearing_diff=45.0,
    frc_tolerance=1,
    length_tolerance=0.20,
    absolute_length_tolerance=50.0,
    frc_weight=0.5,
    fow_weight=0.3,
    bearing_weight=0.5,
)
```
