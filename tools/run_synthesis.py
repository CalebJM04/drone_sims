#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

from drone_sims.provenance import collect_metadata


MODULES = {
    "packet_parser": "rtl/packet_parser.sv",
    "neighbor_table": "rtl/neighbor_table.sv",
    "collision_predictor": "rtl/collision_predictor.sv",
}


def main() -> int:
    parser = argparse.ArgumentParser(description="Elaborate and synthesize-check all project RTL")
    parser.add_argument("--output", type=Path, default=Path("results/full/fpga_synthesis.json"))
    args = parser.parse_args()
    yosys = shutil.which("yosys")
    if not yosys:
        raise SystemExit("yosys is not installed")
    project = Path(__file__).resolve().parents[1]
    report: dict[str, object] = {
        "metadata": collect_metadata("tools/run_synthesis.py"),
        "tool": "yosys",
        "modules": {},
        "passed": True,
    }
    with tempfile.TemporaryDirectory(prefix="drone-synth-") as temporary:
        for module, relative in MODULES.items():
            stats_path = Path(temporary) / f"{module}.json"
            command = (
                f"read_verilog -sv {project / relative}; hierarchy -check -top {module}; "
                f"proc; memory; opt; check; tee -q -o {stats_path} stat -json"
            )
            completed = subprocess.run(
                [yosys, "-Q", "-p", command], text=True, capture_output=True, check=False,
            )
            entry: dict[str, object] = {
                "passed": completed.returncode == 0,
                "warnings": [line.strip() for line in completed.stderr.splitlines() if "warning" in line.lower()],
            }
            if completed.returncode == 0:
                raw = json.loads(stats_path.read_text(encoding="utf-8"))
                design = raw["design"]
                entry.update({
                    "wire_bits": design["num_wire_bits"],
                    "memory_bits": design["num_memory_bits"],
                    "cells": design["num_cells"],
                    "cell_types": design["num_cells_by_type"],
                })
            else:
                entry["error"] = (completed.stdout + completed.stderr)[-4000:]
                report["passed"] = False
            report["modules"][module] = entry
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
