"""Render before/after PNGs of decoded OpenLR codes via the openlr-web screenshot endpoint.

Requires a running openlr-web server with the network loaded, e.g.:
    cd ~/openlr-web && .venv/bin/uvicorn app:app --port 8000
    curl -X POST localhost:8000/api/network/load -H 'content-type: application/json' \
         -d '{"path":"openlr_test_edges.parquet"}'

Usage:
    python qa/screenshot.py CODE [CODE ...] --out DIR \
        --before '{"snap_to_valid_nodes": false}' --after '{"snap_to_valid_nodes": true}' \
        [--titles "before|after"] [--zoom-lrps 0 1] [--span-m 60] [--server http://localhost:8000]

For each code this writes `<slug>_overview.png` plus `<slug>_lrp<N>.png` for each
requested zoom LRP. The HERE reference geometry is drawn when the code is found in
qa/test_codes.csv. Pass only --before to get single-panel images.
"""
import argparse
import csv
import json
import re
import urllib.request
from pathlib import Path


def load_references(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    with path.open() as f:
        return {row["openlr_code"]: row["reference_wkt"] for row in csv.DictReader(f)}


def fetch(server: str, body: dict) -> bytes:
    req = urllib.request.Request(
        f"{server}/api/screenshot.png",
        data=json.dumps(body).encode(),
        headers={"content-type": "application/json"},
    )
    with urllib.request.urlopen(req) as r:
        return r.read()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("codes", nargs="+")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--before", type=json.loads, default={}, help="DecoderConfig JSON for the first panel")
    ap.add_argument("--after", type=json.loads, default=None, help="DecoderConfig JSON for the second panel")
    ap.add_argument("--titles", default=None, help="'|'-separated panel titles")
    ap.add_argument("--zoom-lrps", type=int, nargs="*", default=[], help="LRP indices to render zoomed views for")
    ap.add_argument("--span-m", type=float, default=60.0, help="Width of zoomed views in meters")
    ap.add_argument("--width-px", type=int, default=700)
    ap.add_argument("--server", default="http://localhost:8000")
    ap.add_argument("--codes-csv", type=Path, default=Path(__file__).parent / "test_codes.csv")
    args = ap.parse_args()

    refs = load_references(args.codes_csv)
    args.out.mkdir(parents=True, exist_ok=True)

    for code in args.codes:
        slug = re.sub(r"[^A-Za-z0-9]", "", code)[:16]
        base = {
            "code": code,
            "config": args.before,
            "compare_config": args.after,
            "titles": args.titles.split("|") if args.titles else None,
            "reference_wkt": refs.get(code),
            "width_px": args.width_px,
        }
        out = args.out / f"{slug}_overview.png"
        out.write_bytes(fetch(args.server, base))
        print(out)
        for lrp in args.zoom_lrps:
            out = args.out / f"{slug}_lrp{lrp}.png"
            out.write_bytes(fetch(args.server, {**base, "focus_lrp": lrp, "span_m": args.span_m, "padding_m": 0}))
            print(out)


if __name__ == "__main__":
    main()
