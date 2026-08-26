"""Capture before/after screenshots of the real openlr-web UI for OpenLR codes.

Uses the app's /api/screenshot.png endpoint (headless Chromium via Playwright), so
the images are exactly what the web app shows. Requires a running server with the
network loaded, e.g.:

    cd ~/openlr-web && .venv/bin/uvicorn app:app --port 8000
    curl -X POST localhost:8000/api/network/load -H 'content-type: application/json' \
         -d '{"path":"openlr_test_edges.parquet"}'

Usage:
    python qa/screenshot.py CODE [CODE ...] --out DIR \
        --before '{"snap_to_valid_nodes": false}' --after '{"snap_to_valid_nodes": true}' \
        [--zoom-lrps 0 1] [--span-m 60] [--map-only] [--server http://localhost:8000]

For each code this writes `<slug>_overview.jpg` plus `<slug>_lrp<N>.jpg` for each
requested zoom LRP (zoom shots are cropped to the map). With --after, each image is the before and after shots stitched
side by side (before on the left). Pass only --before for single shots.
"""
import argparse
import io
import json
import re
import urllib.request
from pathlib import Path


def fetch(server: str, body: dict) -> bytes:
    req = urllib.request.Request(
        f"{server}/api/screenshot.png",
        data=json.dumps(body).encode(),
        headers={"content-type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.read()


def combine(pngs: list[bytes], jpeg_quality: int | None, gap: int = 16) -> bytes:
    """Stitch shots side by side (before on the left) and encode as JPEG or PNG."""
    from PIL import Image

    imgs = [Image.open(io.BytesIO(p)).convert("RGB") for p in pngs]
    h = max(i.height for i in imgs)
    w = sum(i.width for i in imgs) + gap * (len(imgs) - 1)
    out = Image.new("RGB", (w, h), "white")
    x = 0
    for im in imgs:
        out.paste(im, (x, 0))
        x += im.width + gap
    buf = io.BytesIO()
    if jpeg_quality:
        out.save(buf, format="JPEG", quality=jpeg_quality, optimize=True)
    else:
        out.save(buf, format="PNG", optimize=True)
    return buf.getvalue()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("codes", nargs="+")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--before", type=json.loads, default=None, help="DecoderConfig JSON for the left/only shot")
    ap.add_argument("--after", type=json.loads, default=None, help="DecoderConfig JSON for the right shot")
    ap.add_argument("--zoom-lrps", type=int, nargs="*", default=[], help="LRP indices to render zoomed views for")
    ap.add_argument("--span-m", type=float, default=60.0, help="Width of zoomed views in meters")
    ap.add_argument("--width", type=int, default=1100)
    ap.add_argument("--height", type=int, default=750)
    ap.add_argument("--map-only", action="store_true", help="Crop overview shots to the map too (zoom shots always are)")
    ap.add_argument("--scale", type=int, default=1, help="Device pixel ratio (2 = retina)")
    ap.add_argument("--server", default="http://localhost:8000")
    ap.add_argument("--png", action="store_true", help="Write lossless PNG instead of JPEG (much larger with satellite imagery)")
    ap.add_argument("--jpeg-quality", type=int, default=85)
    args = ap.parse_args()

    configs = [args.before] + ([args.after] if args.after is not None else [])
    args.out.mkdir(parents=True, exist_ok=True)

    for code in args.codes:
        slug = re.sub(r"[^A-Za-z0-9]", "", code)[:16]
        views = [("overview", {"map_only": args.map_only})] + [
            (f"lrp{i}", {"focus_lrp": i, "span_m": args.span_m, "map_only": True}) for i in args.zoom_lrps
        ]
        for name, view in views:
            shots = [
                fetch(
                    args.server,
                    {"code": code, "config": cfg, "width": args.width, "height": args.height, "scale": args.scale, **view},
                )
                for cfg in configs
            ]
            out = args.out / f"{slug}_{name}.{'png' if args.png else 'jpg'}"
            out.write_bytes(combine(shots, None if args.png else args.jpeg_quality))
            print(out)


if __name__ == "__main__":
    main()
