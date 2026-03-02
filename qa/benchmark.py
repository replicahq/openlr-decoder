#!/usr/bin/env python3
"""
OpenLR decoder QA benchmark.

Decodes a corpus of OpenLR codes, compares decoded paths against HERE reference
geometries using Hausdorff and Fréchet distance, and saves results for
before/after comparison.

Usage:
    # Run benchmark and save results
    python qa/benchmark.py --save results_baseline.parquet

    # Run again after a change
    python qa/benchmark.py --save results_after.parquet

    # Compare two runs
    python qa/benchmark.py --diff results_baseline.parquet results_after.parquet
"""

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from shapely import wkt, wkb, LineString
from shapely import frechet_distance, Point
from shapely.ops import substring

import openlr_decoder


def load_test_codes(csv_path: str) -> list[dict]:
    """Load test codes + reference WKT from CSV."""
    codes = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["reference_wkt"]:
                codes.append(row)
    return codes


def build_edge_geom_lookup(parquet_path: str) -> dict[int, bytes]:
    """Build edge_id -> WKB geometry lookup from network parquet."""
    t = pq.read_table(parquet_path, columns=["stableEdgeId", "geometry"])
    edge_ids = t.column("stableEdgeId").to_pylist()
    geoms = t.column("geometry").to_pylist()
    return {eid: g for eid, g in zip(edge_ids, geoms) if g is not None}


def reconstruct_path_geometry(
    edge_ids: list[int],
    edge_geoms: dict[int, bytes],
    pos_offset_frac: float,
    neg_offset_frac: float,
) -> LineString | None:
    """Reconstruct the decoded path geometry from edge IDs and offsets."""
    if not edge_ids:
        return None

    coords = []
    for eid in edge_ids:
        wkb_bytes = edge_geoms.get(eid)
        if wkb_bytes is None:
            continue
        try:
            geom = wkb.loads(wkb_bytes)
            edge_coords = list(geom.coords)
        except Exception:
            continue

        if coords:
            # Skip duplicate junction point
            if edge_coords[0] == coords[-1]:
                edge_coords = edge_coords[1:]
        coords.extend(edge_coords)

    if len(coords) < 2:
        return None

    line = LineString(coords)

    # Apply offsets
    total_length = line.length
    if total_length == 0:
        return line

    start_frac = pos_offset_frac
    end_frac = 1.0 - neg_offset_frac

    if start_frac > 0 or end_frac < 1.0:
        start_dist = start_frac * total_length
        end_dist = end_frac * total_length
        if end_dist > start_dist:
            line = substring(line, start_dist, end_dist)

    return line


def compute_metrics(decoded_geom: LineString, reference_geom) -> dict:
    """Compute geometric similarity metrics between decoded and reference paths."""
    if decoded_geom is None or decoded_geom.is_empty:
        return {"hausdorff_m": None, "frechet_m": None, "length_ratio": None}

    # Convert to approximate meters using center latitude
    # 1 degree lat ≈ 111,320m, 1 degree lon ≈ 111,320 * cos(lat)
    center_lat = (decoded_geom.centroid.y + reference_geom.centroid.y) / 2
    lat_scale = 111_320.0
    lon_scale = 111_320.0 * np.cos(np.radians(center_lat))

    def to_meters(geom):
        coords = list(geom.coords)
        return LineString([(c[0] * lon_scale, c[1] * lat_scale) for c in coords])

    dec_m = to_meters(decoded_geom)
    ref_m = to_meters(reference_geom)

    # Directed Hausdorff: decoded → reference (point-to-line, not point-to-point)
    # "How far does the decoded path stray from the reference?"
    h = max(ref_m.distance(Point(p)) for p in dec_m.coords)
    f = frechet_distance(dec_m, ref_m)
    length_ratio = dec_m.length / ref_m.length if ref_m.length > 0 else None

    return {"hausdorff_m": h, "frechet_m": f, "length_ratio": length_ratio}


def run_benchmark(
    codes_csv: str,
    network_parquet: str,
    output_path: str,
    config=None,
):
    """Run the full benchmark: decode all codes, compute metrics, save results."""
    print(f"Loading test codes from {codes_csv}...")
    test_cases = load_test_codes(codes_csv)
    print(f"  {len(test_cases)} codes with reference geometries")

    print(f"Loading network from {network_parquet}...")
    network = openlr_decoder.RoadNetwork.from_parquet(network_parquet)
    decoder = openlr_decoder.Decoder(network, config)
    print(f"  {network.edge_count} edges, {network.node_count} nodes")

    print("Building edge geometry lookup...")
    edge_geoms = build_edge_geom_lookup(network_parquet)
    print(f"  {len(edge_geoms)} edges with geometry")

    # Decode all codes
    openlr_codes = [tc["openlr_code"] for tc in test_cases]
    print(f"Decoding {len(openlr_codes)} codes...")
    t0 = time.time()
    batch = decoder.decode_batch(openlr_codes)
    decode_time = time.time() - t0
    print(
        f"  Decoded in {decode_time:.1f}s ({len(openlr_codes) / decode_time:.0f} codes/s)"
    )

    df = batch.to_pandas()
    cols = set(df.columns)

    # Handle schema differences across versions
    pos_off_col = (
        "positive_offset_fraction"
        if "positive_offset_fraction" in cols
        else "positive_offset"
    )
    neg_off_col = (
        "negative_offset_fraction"
        if "negative_offset_fraction" in cols
        else "negative_offset"
    )
    has_primary = "primary_edge_id" in cols

    def safe_float(val):
        try:
            return None if np.isnan(val) else val
        except (TypeError, ValueError):
            return None

    # Compute geometric metrics
    print("Computing geometric metrics...")
    results = []
    for i, tc in enumerate(test_cases):
        row = df.iloc[i]
        edge_ids = list(row["edge_ids"])
        error = row["error"] if isinstance(row["error"], str) else None

        pos_frac = safe_float(row[pos_off_col]) or 0.0
        neg_frac = safe_float(row[neg_off_col]) or 0.0

        result = {
            "openlr_code": tc["openlr_code"],
            "success": error is None and len(edge_ids) > 0,
            "error": error,
            "edge_ids": ",".join(str(e) for e in edge_ids),
            "n_edges": len(edge_ids),
            "length_m": safe_float(row["length"]),
            "primary_edge_id": str(int(row["primary_edge_id"]))
            if has_primary and safe_float(row.get("primary_edge_id"))
            else None,
            "positive_offset_frac": pos_frac,
            "negative_offset_frac": neg_frac,
        }

        # Compute geometric metrics for successful decodes
        if result["success"]:
            ref_geom = wkt.loads(tc["reference_wkt"])
            decoded_geom = reconstruct_path_geometry(
                [int(e) for e in edge_ids],
                edge_geoms,
                pos_frac,
                neg_frac,
            )
            metrics = compute_metrics(decoded_geom, ref_geom)
        else:
            metrics = {"hausdorff_m": None, "frechet_m": None, "length_ratio": None}

        result.update(metrics)
        results.append(result)

    # Save results
    out_table = pa.table(
        {
            "openlr_code": [r["openlr_code"] for r in results],
            "success": [r["success"] for r in results],
            "error": [r["error"] for r in results],
            "edge_ids": [r["edge_ids"] for r in results],
            "n_edges": [r["n_edges"] for r in results],
            "length_m": [r["length_m"] for r in results],
            "primary_edge_id": [r["primary_edge_id"] for r in results],
            "positive_offset_frac": [r["positive_offset_frac"] for r in results],
            "negative_offset_frac": [r["negative_offset_frac"] for r in results],
            "hausdorff_m": [r["hausdorff_m"] for r in results],
            "frechet_m": [r["frechet_m"] for r in results],
            "length_ratio": [r["length_ratio"] for r in results],
        }
    )

    pq.write_table(out_table, output_path)
    print(f"\nResults saved to {output_path}")
    print_summary(results)


def print_summary(results: list[dict]):
    """Print a summary of benchmark results."""
    total = len(results)
    successes = [r for r in results if r["success"]]
    failures = [r for r in results if not r["success"]]

    hausdorff_vals = [
        r["hausdorff_m"] for r in successes if r["hausdorff_m"] is not None
    ]
    frechet_vals = [r["frechet_m"] for r in successes if r["frechet_m"] is not None]
    length_ratios = [
        r["length_ratio"] for r in successes if r["length_ratio"] is not None
    ]

    print(f"\n{'=' * 60}")
    print("BENCHMARK SUMMARY")
    print(f"{'=' * 60}")
    print(f"Total codes:      {total}")
    print(f"Decoded:          {len(successes)} ({100 * len(successes) / total:.1f}%)")
    print(f"Failed:           {len(failures)} ({100 * len(failures) / total:.1f}%)")

    if hausdorff_vals:
        h = np.array(hausdorff_vals)
        f = np.array(frechet_vals)
        lr = np.array(length_ratios)

        print("\nGeometric Quality (decoded codes only):")
        print(
            f"  Hausdorff distance (m):  p50={np.median(h):.1f}  p90={np.percentile(h, 90):.1f}  p99={np.percentile(h, 99):.1f}  max={h.max():.1f}"
        )
        print(
            f"  Fréchet distance (m):    p50={np.median(f):.1f}  p90={np.percentile(f, 90):.1f}  p99={np.percentile(f, 99):.1f}  max={f.max():.1f}"
        )
        print(
            f"  Length ratio:            p50={np.median(lr):.2f}  p10={np.percentile(lr, 10):.2f}  p90={np.percentile(lr, 90):.2f}"
        )

        # Quality buckets based on Hausdorff distance
        good = sum(1 for v in h if v < 30)
        ok = sum(1 for v in h if 30 <= v < 100)
        bad = sum(1 for v in h if v >= 100)
        print("\n  Quality buckets (by Hausdorff):")
        print(f"    Good  (<30m):   {good:5d} ({100 * good / len(h):.1f}%)")
        print(f"    OK    (30-100m):{ok:5d} ({100 * ok / len(h):.1f}%)")
        print(f"    Bad   (>100m):  {bad:5d} ({100 * bad / len(h):.1f}%)")

    print(f"{'=' * 60}")


def diff_results(before_path: str, after_path: str):
    """Compare two benchmark results and report regressions/improvements."""
    before = pq.read_table(before_path).to_pandas()
    after = pq.read_table(after_path).to_pandas()

    # Join on openlr_code
    merged = before.merge(after, on="openlr_code", suffixes=("_before", "_after"))

    total = len(merged)
    print(f"\n{'=' * 60}")
    print(f"DIFF REPORT: {Path(before_path).name} → {Path(after_path).name}")
    print(f"{'=' * 60}")
    print(f"Codes compared: {total}")

    # Success changes
    gained = merged[~merged["success_before"] & merged["success_after"]]
    lost = merged[merged["success_before"] & ~merged["success_after"]]
    print("\nDecode changes:")
    print(f"  Newly decoded:  {len(gained)}")
    print(f"  Newly failed:   {len(lost)}")

    if len(lost) > 0:
        print("\n  Newly failed codes:")
        for _, row in lost.head(10).iterrows():
            print(f"    {row['openlr_code']}  error: {row['error_after']}")

    # Geometric quality changes (for codes that succeeded in both)
    both_ok = merged[merged["success_before"] & merged["success_after"]].copy()
    both_ok = both_ok.dropna(subset=["hausdorff_m_before", "hausdorff_m_after"])

    if len(both_ok) > 0:
        h_before = both_ok["hausdorff_m_before"].values
        h_after = both_ok["hausdorff_m_after"].values
        f_before = both_ok["frechet_m_before"].values
        f_after = both_ok["frechet_m_after"].values

        print(f"\nGeometric quality ({len(both_ok)} codes decoded in both):")
        print(
            f"  Hausdorff p50: {np.median(h_before):.1f}m → {np.median(h_after):.1f}m"
        )
        print(
            f"  Hausdorff p90: {np.percentile(h_before, 90):.1f}m → {np.percentile(h_after, 90):.1f}m"
        )
        print(
            f"  Fréchet p50:   {np.median(f_before):.1f}m → {np.median(f_after):.1f}m"
        )
        print(
            f"  Fréchet p90:   {np.percentile(f_before, 90):.1f}m → {np.percentile(f_after, 90):.1f}m"
        )

        # Winners/losers (>5m Hausdorff change)
        delta = h_after - h_before
        THRESHOLD = 5.0
        winners = both_ok[delta < -THRESHOLD]
        losers = both_ok[delta > THRESHOLD]
        unchanged = both_ok[abs(delta) <= THRESHOLD]

        print(f"\n  Winners (Hausdorff improved by >{THRESHOLD}m): {len(winners)}")
        print(f"  Losers  (Hausdorff worsened by >{THRESHOLD}m): {len(losers)}")
        print(f"  Unchanged (within ±{THRESHOLD}m):              {len(unchanged)}")

        if len(winners) > 0:
            print("\n  Top 10 winners:")
            top_w = winners.copy()
            top_w["delta"] = top_w["hausdorff_m_before"] - top_w["hausdorff_m_after"]
            top_w = top_w.nlargest(10, "delta")
            for _, row in top_w.iterrows():
                print(
                    f"    {row['openlr_code']}  {row['hausdorff_m_before']:.1f}m → {row['hausdorff_m_after']:.1f}m  (Δ{-row['delta']:.1f}m)"
                )

        if len(losers) > 0:
            print("\n  Top 10 losers:")
            top_l = losers.copy()
            top_l["delta"] = top_l["hausdorff_m_after"] - top_l["hausdorff_m_before"]
            top_l = top_l.nlargest(10, "delta")
            for _, row in top_l.iterrows():
                print(
                    f"    {row['openlr_code']}  {row['hausdorff_m_before']:.1f}m → {row['hausdorff_m_after']:.1f}m  (Δ+{row['delta']:.1f}m)"
                )

        # Quality bucket transitions
        def bucket(h):
            if h < 30:
                return "good"
            elif h < 100:
                return "ok"
            else:
                return "bad"

        both_ok["bucket_before"] = both_ok["hausdorff_m_before"].apply(bucket)
        both_ok["bucket_after"] = both_ok["hausdorff_m_after"].apply(bucket)
        changed_bucket = both_ok[both_ok["bucket_before"] != both_ok["bucket_after"]]

        if len(changed_bucket) > 0:
            print("\n  Quality bucket transitions:")
            transitions = changed_bucket.groupby(
                ["bucket_before", "bucket_after"]
            ).size()
            for (b, a), count in transitions.items():
                print(f"    {b} → {a}: {count}")

    print(f"{'=' * 60}")


def main():
    parser = argparse.ArgumentParser(description="OpenLR decoder QA benchmark")
    parser.add_argument(
        "--codes", default="qa/test_codes.csv", help="Path to test codes CSV"
    )
    parser.add_argument(
        "--network", default="openlr_test_edges.parquet", help="Path to network parquet"
    )
    parser.add_argument(
        "--save", metavar="OUTPUT", help="Run benchmark and save results to parquet"
    )
    parser.add_argument(
        "--diff", nargs=2, metavar=("BEFORE", "AFTER"), help="Compare two result files"
    )
    parser.add_argument(
        "--config",
        metavar="KEY=VALUE",
        nargs="+",
        help="DecoderConfig overrides, e.g. --config frc_tolerance=3 search_radius_m=150",
    )

    args = parser.parse_args()

    config = None
    if args.config:
        config_kwargs = {}
        for kv in args.config:
            k, v = kv.split("=", 1)
            # Auto-cast numeric values
            try:
                v = int(v)
            except ValueError:
                try:
                    v = float(v)
                except ValueError:
                    pass
            config_kwargs[k] = v
        config = openlr_decoder.DecoderConfig(**config_kwargs)
        print(f"Using config overrides: {config_kwargs}")

    if args.diff:
        diff_results(args.diff[0], args.diff[1])
    elif args.save:
        run_benchmark(args.codes, args.network, args.save, config=config)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
