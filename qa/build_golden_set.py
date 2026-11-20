"""Build qa/golden_portions.csv: Arity geoid_portion codes with reference geometry.

The codes come from `model-159019.arity.geoid_pairs`, the table production
decodes. Every code there has two LRPs. A `geoid_portion` repeats its parent
`geo_id`'s LRPs and adds encoded offsets that cut out one slice. Parents are
sampled by stratum and all their portions are kept, so sibling slices can be
checked against each other.

Arity's bearings are the start-to-end chord rather than the road direction,
and `geoid_pairs.openlr_geom` is only that chord. So the reference geometry is
the shortest path between the two LRPs on TomTom Orbis, which is a second map
independent of our OSM network:

- Each LRP snaps to Orbis road segments within SNAP_M. Bearings are ignored.
- Slip-road codes (FOW 6) pay COST_OFF_LINK for every non-link segment, so a
  ramp isn't swapped for the mainline it joins.
- The path is kept only if its length falls in the code's DNP bucket, widened
  by DNP_SLACK. The code is ambiguous if another snap combination, snapping
  within RIVAL_SNAP_M as well, gives a fitting path whose interior differs by
  more than AMBIGUOUS_M.
- Otherwise the reference falls back to the chord, and `reference_source`
  says which was used.

Portions take the parent's reference, trimmed by their offset fractions.

Usage (BigQuery pulls are cached in --cache; delete the files to refresh):
    uvx --with openlr --with polars --with shapely --with numpy --with pyarrow \\
        --with google-cloud-bigquery --with google-cloud-bigquery-storage \\
        python qa/build_golden_set.py --cache /tmp/golden --out qa/golden_portions.csv
"""

import argparse
import csv
import heapq
import math
from pathlib import Path

import numpy as np
import openlr
import polars as pl
import shapely
from shapely.geometry import LineString
from shapely.ops import substring

# Same bbox as qa/test_codes.csv and ~/openlr-web/openlr_test_edges.parquet.
BBOX = (-95.15, 38.53, -94.47, 39.11)  # min lon, min lat, max lon, max lat
ORBIS_TABLE = "model-159019.tomtom.orbis_ways_2026_09_21"
PAIRS_TABLE = "model-159019.arity.geoid_pairs"

DNP_BUCKET_M = 58.6
SNAP_M = 6.0
DNP_SLACK = (10.0, 0.10)  # meters, fraction: the looser one applies
AMBIGUOUS_M = 5.0
RIVAL_SNAP_M = 2.0  # alternatives snapping this much worse than the best are ignored
COST_OFF_LINK = 4.0
SEED = 20261120

# Parents to sample per stratum. Slip roads are rare, so all of them are kept.
STRATA = {
    "slip_road": None,
    "dnp_0_59m": 4000,
    "dnp_59_117m": 3000,
    "dnp_117_176m": 2000,
    "dnp_176_234m": 1500,
    "dnp_234m_plus": 1500,
}
DRIVABLE = {
    "motorway",
    "trunk",
    "primary",
    "secondary",
    "tertiary",
    "unclassified",
    "residential",
    "living_street",
    "service",
    "road",
    "track",
    "motorway_link",
    "trunk_link",
    "primary_link",
    "secondary_link",
    "tertiary_link",
}

LAT0 = math.radians((BBOX[1] + BBOX[3]) / 2)
KX, KY = 111_320.0 * math.cos(LAT0), 110_540.0  # meters per degree


def to_m(lon, lat):
    return np.column_stack([np.asarray(lon) * KX, np.asarray(lat) * KY])


def pull(cache: Path) -> tuple[pl.DataFrame, pl.DataFrame]:
    pairs_path, orbis_path = cache / "geoid_pairs.parquet", cache / "orbis_ways.parquet"
    box = ", ".join(map(str, BBOX))
    queries = {
        pairs_path: f"""SELECT DISTINCT geo_id, geoid_portion FROM `{PAIRS_TABLE}`
            WHERE ST_INTERSECTSBOX(openlr_geom, {box})""",
        orbis_path: f"""SELECT way_id, JSON_VALUE(tags_json, '$.highway') highway,
            JSON_VALUE(tags_json, '$.oneway') oneway, ST_ASBINARY(geometry) wkb
            FROM `{ORBIS_TABLE}` WHERE ST_INTERSECTSBOX(geometry, {box})""",
    }
    missing = [p for p in queries if not p.exists()]
    if missing:
        import pyarrow.parquet as pq
        from google.cloud import bigquery

        client = bigquery.Client(project="model-159019")
        for path in missing:
            print(f"Querying {path.name}...")
            pq.write_table(client.query(queries[path]).to_arrow(), path)
    return pl.read_parquet(pairs_path), pl.read_parquet(orbis_path)


class OrbisGraph:
    """Directed graph of drivable Orbis segments, one edge per vertex pair."""

    def __init__(self, ways: pl.DataFrame):
        ways = ways.filter(pl.col("highway").is_in(DRIVABLE))
        geoms = shapely.from_wkb(ways["wkb"].to_numpy())
        coords, way_idx = shapely.get_coordinates(geoms, return_index=True)
        keys = np.round(coords * 1e7).astype(np.int64)
        uniq, node = np.unique(keys, axis=0, return_inverse=True)
        self.node_xy = to_m(uniq[:, 0] / 1e7, uniq[:, 1] / 1e7)
        self.node_lonlat = uniq / 1e7

        same_way = way_idx[:-1] == way_idx[1:]
        a, b = node[:-1][same_way], node[1:][same_way]
        w = way_idx[:-1][same_way]
        length = np.hypot(*(self.node_xy[b] - self.node_xy[a]).T)
        oneway = ways["oneway"].to_numpy()[w]
        is_link = ways["highway"].str.ends_with("_link").to_numpy()[w]
        fwd = oneway != "-1"
        rev = ~np.isin(oneway, ["yes", "1", "true", "-1"]) | (oneway == "-1")

        src = np.concatenate([a[fwd], b[rev]])
        dst = np.concatenate([b[fwd], a[rev]])
        self.length = np.concatenate([length[fwd], length[rev]])
        self.is_link = np.concatenate([is_link[fwd], is_link[rev]])
        order = np.argsort(src, kind="stable")
        self.src, self.dst = src[order], dst[order]
        self.length, self.is_link = self.length[order], self.is_link[order]
        self.offsets = np.searchsorted(self.src, np.arange(len(uniq) + 1))
        lines = shapely.linestrings(
            np.stack([self.node_xy[self.src], self.node_xy[self.dst]], axis=1)
        )
        self.tree = shapely.STRtree(lines)
        print(f"Orbis graph: {len(uniq)} nodes, {len(self.src)} directed segments")

    def snaps(self, lon, lat):
        """(edge, t, distance_m) for each directed segment within SNAP_M."""
        p = shapely.points(to_m([lon], [lat]))[0]
        out = []
        for e in self.tree.query(p, predicate="dwithin", distance=SNAP_M):
            a, b = self.node_xy[self.src[e]], self.node_xy[self.dst[e]]
            ab = b - a
            denom = ab @ ab
            t = (
                0.0
                if denom == 0
                else float(np.clip((np.array([p.x, p.y]) - a) @ ab / denom, 0, 1))
            )
            out.append((int(e), t, float(np.hypot(*(a + t * ab - [p.x, p.y])))))
        return out

    def cost(self, e, slip):
        c = self.length[e]
        return c * COST_OFF_LINK if slip and not self.is_link[e] else c

    def path(self, start, end, limit_m, slip):
        """Lowest-cost path from snap `start` to snap `end` as (length_m, nodes),
        or None. Partial first/last segments are counted by their t. U-turns
        are not allowed, including onto the last segment."""
        (es, ts, _), (ee, te, _) = start, end
        if es == ee and te >= ts:
            return (te - ts) * self.length[es], [es, es]
        first = int(self.dst[es])
        target = int(self.src[ee])
        head = (1 - ts) * self.length[es]
        tail = te * self.length[ee]
        dist = {first: (self.cost(es, slip) * (1 - ts), head)}
        prev = {first: None}
        came_from = {first: int(self.src[es])}
        heap = [(dist[first][0], first)]
        while heap:
            c, n = heapq.heappop(heap)
            if c > dist[n][0]:
                continue
            if n == target:
                if came_from[n] == int(self.dst[ee]):
                    return None
                nodes, m = [], n
                while m is not None:
                    nodes.append(m)
                    m = prev[m]
                return dist[n][1] + tail, nodes[::-1]
            if dist[n][1] > limit_m:
                continue
            for e in range(self.offsets[n], self.offsets[n + 1]):
                m = int(self.dst[e])
                if m == came_from[n]:
                    continue
                nc = c + self.cost(e, slip)
                if m not in dist or nc < dist[m][0]:
                    dist[m] = (nc, dist[n][1] + self.length[e])
                    prev[m] = n
                    came_from[m] = n
                    heapq.heappush(heap, (nc, m))
        return None

    def line(self, start, end, nodes):
        """Lon/lat LineString from the start snap point through `nodes` to the end."""

        def point(e, t):
            a, b = self.node_lonlat[self.src[e]], self.node_lonlat[self.dst[e]]
            return tuple(a + t * (b - a))

        pts = [point(*start[:2])]
        if start[0] != end[0] or len(nodes) != 2 or nodes[0] != nodes[1]:
            pts += [tuple(self.node_lonlat[n]) for n in nodes]
        pts.append(point(*end[:2]))
        dedup = [pts[0]] + [p for i, p in enumerate(pts[1:]) if p != pts[i]]
        return LineString(dedup) if len(dedup) >= 2 else None


def stratum(points) -> str:
    if points[0].fow == 6:
        return "slip_road"
    return list(STRATA)[1 + min(int(points[0].dnp // DNP_BUCKET_M), 4)]


def interior_hausdorff_m(a: LineString, b: LineString) -> float:
    """Hausdorff distance in meters with SNAP_M + 2 m cut from each end."""

    def inner(g):
        g = shapely.transform(g, lambda c: c * [KX, KY])
        cut = SNAP_M + 2.0
        return substring(g, cut, g.length - cut) if g.length > 2 * cut + 1 else g

    return shapely.hausdorff_distance(inner(a), inner(b))


def reference(graph: OrbisGraph, points):
    """(LineString, source, detail) for a parent's two LRPs."""
    start, end = points[0], points[-1]
    chord = LineString([(start.lon, start.lat), (end.lon, end.lat)])
    bucket = int(start.dnp // DNP_BUCKET_M)
    lo, hi = bucket * DNP_BUCKET_M, (bucket + 1) * DNP_BUCKET_M
    slack = max(DNP_SLACK[0], DNP_SLACK[1] * hi)
    lo, hi = lo - slack, hi + slack
    slip = start.fow == 6

    starts, ends = graph.snaps(start.lon, start.lat), graph.snaps(end.lon, end.lat)
    if not starts or not ends:
        return chord, "chord", "no_snap"
    fits = []
    for s in starts:
        for e in ends:
            found = graph.path(s, e, hi, slip)
            if found and lo <= found[0] <= hi:
                line = graph.line(s, e, found[1])
                if line is not None:
                    fits.append((s[2] + e[2], found[0], line))
    if not fits:
        return chord, "chord", "no_path_in_dnp"
    fits.sort(key=lambda f: (round(f[0], 1), f[1]))
    best = fits[0]
    # A junction LRP snaps to every segment there, and snaps onto a crossing
    # road add a short stub. Only alternatives that snap about as well count,
    # compared away from their ends.
    rivals = [f for f in fits[1:] if f[0] <= best[0] + RIVAL_SNAP_M]
    if any(interior_hausdorff_m(best[2], f[2]) > AMBIGUOUS_M for f in rivals):
        return chord, "chord", "ambiguous"
    return best[2], "orbis", f"{best[1]:.1f}"


def trim(line: LineString, pos: float, neg: float) -> LineString:
    if pos <= 0 and neg <= 0:
        return line
    return substring(line, pos, 1 - neg, normalized=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--cache", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=Path("qa/golden_portions.csv"))
    args = ap.parse_args()
    args.cache.mkdir(parents=True, exist_ok=True)

    pairs, ways = pull(args.cache)
    print(
        f"{pairs.height} (geo_id, geoid_portion) rows, {pairs['geo_id'].n_unique()} geo_ids"
    )

    parents = pl.DataFrame({"geo_id": pairs["geo_id"].unique().sort()})
    parents = parents.with_columns(
        pl.col("geo_id")
        .map_elements(
            lambda c: stratum(openlr.binary_decode(c).points), return_dtype=pl.Utf8
        )
        .alias("stratum")
    )
    sizes = parents.group_by("stratum").len()
    picked = []
    for name, n in STRATA.items():
        pool = parents.filter(pl.col("stratum") == name)
        take = pool if n is None or pool.height <= n else pool.sample(n, seed=SEED)
        picked.append(
            take.with_columns(pl.lit(pool.height / take.height).alias("weight"))
        )
    # Portions shared between geo_ids are the duplicate case Robert reported.
    shared = pairs.group_by("geoid_portion").agg(pl.col("geo_id").n_unique().alias("n"))
    shared_parents = pairs.join(shared.filter(pl.col("n") > 1), on="geoid_portion")[
        "geo_id"
    ]
    picked.append(
        parents.filter(pl.col("geo_id").is_in(shared_parents.implode())).with_columns(
            pl.lit(1.0).alias("weight"), pl.lit("shared_portion").alias("stratum")
        )
    )
    sample = pl.concat(picked).unique("geo_id", keep="first")
    print(sizes.sort("stratum"))
    print(sample.group_by("stratum").len().sort("stratum"))

    graph = OrbisGraph(ways)
    rows, sources = {}, {}
    portions = pairs.join(sample, on="geo_id").sort(["geo_id", "geoid_portion"])
    by_parent = portions.partition_by("geo_id", as_dict=True)
    for i, ((geo_id,), group) in enumerate(by_parent.items()):
        if i % 2000 == 0:
            print(f"  {i}/{len(by_parent)} parents")
        points = openlr.binary_decode(geo_id).points
        line, source, detail = reference(graph, points)
        sources[source + ":" + (detail if source == "chord" else "ok")] = (
            sources.get(source + ":" + (detail if source == "chord" else "ok"), 0) + 1
        )
        codes = set(group["geoid_portion"].to_list()) | {geo_id}
        meta = group.row(0, named=True)
        for code in sorted(codes):
            if code in rows:
                continue
            ref = openlr.binary_decode(code)
            portion = trim(line, ref.poffs, ref.noffs)
            rows[code] = {
                "openlr_code": code,
                "geo_id": geo_id,
                "is_parent": code == geo_id,
                "positive_offset_frac": round(ref.poffs, 6),
                "negative_offset_frac": round(ref.noffs, 6),
                "stratum": meta["stratum"],
                "weight": round(meta["weight"], 4),
                "frc": points[0].frc,
                "fow": points[0].fow,
                "dnp_m": round(points[0].dnp, 1),
                "reference_source": source,
                "reference_detail": detail,
                "reference_wkt": shapely.to_wkt(portion, rounding_precision=7),
            }
    print("Parent references:", dict(sorted(sources.items())))

    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(next(iter(rows.values()))))
        writer.writeheader()
        writer.writerows(rows.values())
    print(f"Wrote {len(rows)} codes to {args.out}")


if __name__ == "__main__":
    main()
