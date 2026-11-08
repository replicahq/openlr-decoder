from datetime import datetime, timedelta

import pyarrow as pa
import pytest

from map_matcher import MatchConfig, Matcher, RoadNetwork

LAT = 39.0
DEG_PER_M = 1 / 86_500  # degrees of longitude per meter at LAT


def lon(m):
    return -94.5 + m * DEG_PER_M


@pytest.fixture(scope="module")
def network():
    """A two-way east-west road with nodes every 100 m from 0 to 300 m, eastbound
    edges 1-3 and westbound edges 11-13, plus a separate one-way road 2 km east
    (edge 21)."""
    rows = []
    for i in range(3):
        a, b = lon(100 * i), lon(100 * (i + 1))
        rows.append((1 + i, i, i + 1, a, b))
        rows.append((11 + i, i + 1, i, b, a))
    rows.append((21, 100, 101, lon(2000), lon(2100)))
    table = pa.table(
        {
            "stableEdgeId": pa.array([r[0] for r in rows], pa.uint64()),
            "startOsmNode": pa.array([r[1] for r in rows], pa.int64()),
            "endOsmNode": pa.array([r[2] for r in rows], pa.int64()),
            "startLat": [LAT] * len(rows),
            "startLon": [r[3] for r in rows],
            "endLat": [LAT] * len(rows),
            "endLon": [r[4] for r in rows],
            "highway": ["secondary"] * len(rows),
        }
    )
    return RoadNetwork.from_arrow(table)


def pings(trace_ids, xs, seconds, **extra):
    t0 = datetime(2026, 1, 1)
    return pa.table(
        {
            "trace_id": trace_ids,
            "time": pa.array([t0 + timedelta(seconds=s) for s in seconds], pa.timestamp("ms")),
            "lat": [LAT] * len(xs),
            "lon": [lon(x) for x in xs],
            **extra,
        }
    )


def test_network_counts(network):
    assert network.edge_count == 7
    assert network.node_count == 6


def test_matches_traces_in_parallel_with_heading(network):
    matcher = Matcher(network, MatchConfig(sigma_m=5.0))
    table = pings(
        ["east", "east", "east", "west", "west", "west"],
        [20, 120, 220, 280, 180, 80],
        [0, 10, 20, 0, 10, 20],
        heading=[90.0, 90.0, None, 270.0, 270.0, 270.0],
        speed=[10.0, 10.0, 10.0, None, 10.0, 10.0],
    )
    out = matcher.match_traces(table).to_pylist()
    assert [r["trace_id"] for r in out] == ["east", "west"]
    assert out[0]["edge_ids"] == [1, 2, 3]
    assert out[1]["edge_ids"] == [13, 12, 11]
    assert (out[0]["first_row"], out[0]["last_row"]) == (0, 2)
    assert (out[1]["first_row"], out[1]["last_row"]) == (3, 5)
    assert out[0]["start_offset_m"] == pytest.approx(20, abs=1)
    assert out[0]["end_offset_m"] == pytest.approx(20, abs=1)
    assert out[0]["unmatched_rows"] == []


def test_break_and_unmatched_rows(network):
    matcher = Matcher(network, MatchConfig(sigma_m=5.0))
    # Row 1 is 500 m off the road; rows 3-4 are on the disconnected road.
    table = pings([7] * 5, [20, 120, 220, 2020, 2080], [0, 10, 20, 80, 85])
    table = table.set_column(
        2, "lat", pa.array([LAT, LAT + 500 / 111_000, LAT, LAT, LAT])
    )
    out = matcher.match_traces(table).to_pylist()
    assert [r["segment"] for r in out] == [0, 1]
    assert out[0]["trace_id"] == 7
    assert out[0]["edge_ids"] == [1, 2, 3]
    assert out[0]["unmatched_rows"] == [1]
    assert out[1]["edge_ids"] == [21]
    assert out[1]["first_row"] == 3


def test_numeric_seconds_and_schema(network):
    table = pa.table(
        {
            "trace_id": pa.array([1, 1], pa.int32()),
            "time": [0.0, 10.0],
            "lat": [LAT, LAT],
            "lon": [lon(20), lon(120)],
        }
    )
    batch = Matcher(network).match_traces(table)
    assert batch.schema.field("trace_id").type == pa.int32()
    assert batch.schema.field("edge_ids").type == pa.list_(pa.uint64())
    assert batch.num_rows == 1


def test_errors(network):
    matcher = Matcher(network)
    with pytest.raises(ValueError, match="Missing column 'lat'"):
        matcher.match_traces(pa.table({"trace_id": [1], "time": [0.0], "lon": [0.0]}))
    with pytest.raises(ValueError, match="time decreases at row 1"):
        matcher.match_traces(pings([1, 1], [20, 30], [10, 5]))
    with pytest.raises(ValueError, match="'lat' is null at row 0"):
        bad = pings([1], [20], [0]).set_column(2, "lat", pa.array([None], pa.float64()))
        matcher.match_traces(bad)
    with pytest.raises(TypeError, match="'lat' must be numeric"):
        bad = pings([1], [20], [0]).set_column(2, "lat", pa.array(["x"]))
        matcher.match_traces(bad)
    with pytest.raises(ValueError, match="'lon' is not a finite number at row 0"):
        bad = pings([1], [20], [0]).set_column(3, "lon", pa.array([float("nan")]))
        matcher.match_traces(bad)
    with pytest.raises(ValueError, match="'time' is not a finite number at row 1"):
        bad = pa.table({"trace_id": [1, 1], "time": [0.0, float("inf")], "lat": [LAT] * 2, "lon": [lon(20)] * 2})
        matcher.match_traces(bad)


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"sigma_m": 0}, "sigma_m must be a positive number"),
        ({"beta_m": -1}, "beta_m must be a positive number"),
        ({"max_detour_m": float("nan")}, "max_detour_m must be a positive number"),
        ({"backward_slack_m": -5}, "backward_slack_m must be a non-negative number"),
        ({"uturn_penalty": -1}, "uturn_penalty must be a non-negative number"),
        ({"max_candidates": 0}, "max_candidates must be at least 1"),
        ({"max_frc": 8}, "max_frc must be 0 to 7"),
    ],
)
def test_config_validation(kwargs, message):
    with pytest.raises(ValueError, match=message):
        MatchConfig(**kwargs)


def test_nan_and_all_null_speed_heading_are_unknown(network):
    matcher = Matcher(network, MatchConfig(sigma_m=5.0))
    nan = float("nan")
    table = pings([1, 1, 1], [20, 120, 220], [0, 10, 20], speed=[10.0, nan, 10.0], heading=[90.0, 90.0, nan])
    out = matcher.match_traces(table).to_pylist()
    assert len(out) == 1 and out[0]["edge_ids"] == [1, 2, 3]
    table = pings([1, 1, 1], [20, 120, 220], [0, 10, 20], speed=[None] * 3, heading=[None] * 3)
    assert table.schema.field("speed").type == pa.null()
    out = matcher.match_traces(table).to_pylist()
    assert out[0]["edge_ids"] == [1, 2, 3]


def test_config_repr_and_defaults():
    r = repr(MatchConfig(sigma_m=20))
    assert "search_radius_m=80" in r
    assert "backward_slack_m=60" in r
    assert "max_detour_m=2000" in r
    assert "uturn_penalty=2" in r
    assert "use_heading=True" in r


def test_every_row_belongs_to_one_output_row(network):
    matcher = Matcher(network, MatchConfig(sigma_m=5.0))
    off_road = LAT + 500 / 111_000
    table = pings([1] * 5 + [2] * 2, [20, 20, 120, 220, 220, 20, 120], [0, 5, 10, 20, 25, 0, 10])
    lats = [off_road, LAT, LAT, LAT, off_road, off_road, off_road]
    table = table.set_column(2, "lat", pa.array(lats))
    out = matcher.match_traces(table).to_pylist()
    # Trace 1: leading and trailing unmatched rows fall in its only segment's span.
    assert (out[0]["first_row"], out[0]["last_row"]) == (0, 4)
    assert out[0]["unmatched_rows"] == [0, 4]
    assert out[0]["edge_ids"] == [1, 2, 3]
    # Trace 2: nothing matched, so one row with no path.
    assert out[1]["trace_id"] == 2
    assert (out[1]["first_row"], out[1]["last_row"]) == (5, 6)
    assert out[1]["edge_ids"] == []
    assert out[1]["start_offset_m"] is None
    assert out[1]["unmatched_rows"] == [5, 6]


def test_trace_id_must_be_contiguous(network):
    table = pings(["a", "b", "a"], [20, 20, 120], [0, 0, 10])
    with pytest.raises(ValueError, match="trace_id at row 2 also appears in earlier rows"):
        Matcher(network).match_traces(table)


def test_first_bad_row_is_reported(network):
    # Several traces with decreasing time; the error names the earliest.
    ids = [i // 2 for i in range(200)]
    secs = [0 if i % 2 else 10 for i in range(200)]
    table = pings(ids, [20] * 200, secs)
    for _ in range(5):
        with pytest.raises(ValueError, match="time decreases at row 1 "):
            Matcher(network).match_traces(table)


def test_road_network_schema():
    from map_matcher import road_network_schema

    assert "stableEdgeId" in road_network_schema().names
