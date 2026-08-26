"""Regenerate ~/openlr-web/openlr_test_edges.parquet (KC metro test network).

Pulls ALLOWS_CAR edges in the KC bounding box from the Replica street export in
BigQuery via the Storage Read API, with startOsmNode/endOsmNode as the node ids
(required since #25). Reuses the web app's loader with the node columns swapped.

Usage:
    cd ~/openlr-web && .venv/bin/python ~/openlr-decoder/qa/regenerate_network.py [TABLE]
"""
import inspect
import sys
from pathlib import Path

import pyarrow.parquet as pq

WEB = Path.home() / "openlr-web"
sys.path.insert(0, str(WEB))
import app  # noqa: E402

TABLE = sys.argv[1] if len(sys.argv) > 1 else app.DEFAULT_BQ_TABLE
# KC bbox used for qa/test_codes.csv
BBOX = dict(min_lat=38.53, max_lat=39.11, min_lon=-95.15, max_lon=-94.47)
OUT = WEB / "openlr_test_edges.parquet"

code = inspect.getsource(app.load_from_bigquery).replace(
    '"startVertex",\n        "endVertex",', '"startOsmNode",\n        "endOsmNode",'
)
assert '"startOsmNode"' in code, "app.load_from_bigquery column list changed; update this script"
ns = dict(vars(app))
exec(code, ns)

table = ns["load_from_bigquery"](TABLE, **BBOX)
if OUT.exists():
    backup = OUT.with_suffix(".bak.parquet")
    OUT.rename(backup)
    print(f"Previous network moved to {backup}")
pq.write_table(table, OUT)
print(f"Wrote {table.num_rows} edges to {OUT}")
