#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import fmean
from typing import Any

from drone_sims.provenance import collect_metadata


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("records", nargs="+", type=Path)
    parser.add_argument("--minimum-nodes", type=int, default=4)
    parser.add_argument("--minimum-delivery-ratio", type=float, default=0.90)
    parser.add_argument("--output", type=Path, default=Path("results/bench/rf-mesh-acceptance.json"))
    args = parser.parse_args()
    records = [load(path) for path in args.records]
    node_ids = [int(record["node"]) for record in records]
    ratios = [float(record.get("estimated_delivery_ratio", 0.0)) for record in records]
    checks = {
        "unique_node_count": len(set(node_ids)) >= args.minimum_nodes,
        "all_node_runs_passed": all(record.get("passed") is True for record in records),
        "delivery_ratio": bool(ratios) and min(ratios) >= args.minimum_delivery_ratio,
        "all_expected_neighbors_seen": all(
            int(record.get("neighbors", 0)) >= int(record.get("minimum_neighbors", args.minimum_nodes - 1))
            for record in records
        ),
        "link_metadata": all(record.get("link_metadata_available") is True for record in records),
        "observation_only": all(
            int(record.get("stats", {}).get("commands_sent", 1)) == 0 for record in records
        ),
        "protocol_integrity": all(
            int(record.get("stats", {}).get("protocol_rejections", 1)) == 0
            and int(record.get("stats", {}).get("crc_rejections", 1)) == 0
            for record in records
        ),
    }
    result = {
        "metadata": collect_metadata(
            "tools/evaluate_rf_mesh.py",
            parameters={
                "records": [str(path) for path in args.records],
                "minimum_nodes": args.minimum_nodes,
                "minimum_delivery_ratio": args.minimum_delivery_ratio,
            },
        ),
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "nodes": sorted(set(node_ids)),
        "delivery_ratio_min": min(ratios, default=0.0),
        "delivery_ratio_mean": fmean(ratios) if ratios else 0.0,
        "records": [str(path) for path in args.records],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
