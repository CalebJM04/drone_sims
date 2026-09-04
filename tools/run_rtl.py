#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import random
import shutil
import subprocess
import tempfile

from drone_sims.collision import KinematicState, Vec3
from drone_sims.fixedpoint import fixed_point_assess


def signed_literal(width: int, value: int) -> str:
    return f"-{width}'sd{abs(value)}" if value < 0 else f"{width}'sd{value}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Compile and verify the collision RTL using Icarus Verilog")
    parser.add_argument("--cases", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=404)
    args = parser.parse_args()
    compiler = shutil.which("iverilog")
    runtime = shutil.which("vvp")
    if not compiler or not runtime:
        raise SystemExit("SKIP: iverilog/vvp is not installed")
    rng = random.Random(args.seed)
    assignments: list[str] = []
    for case in range(args.cases):
        pa = [rng.randint(-20_000, 20_000) for _ in range(3)]
        pb = [rng.randint(-20_000, 20_000) for _ in range(3)]
        va = [rng.randint(-2_000, 2_000) for _ in range(3)]
        vb = [rng.randint(-2_000, 2_000) for _ in range(3)]
        safety, horizon = rng.randint(100, 1000), rng.randint(1000, 15000)
        own = KinematicState(1, Vec3(*(x / 100 for x in pa)), Vec3(*(x / 100 for x in va)), 0)
        peer = KinematicState(2, Vec3(*(x / 100 for x in pb)), Vec3(*(x / 100 for x in vb)), 0)
        expected = fixed_point_assess(own, peer, safety_distance_cm=safety, horizon_ms=horizon)
        values = [*pa, *pb, *va, *vb]
        names = ["pa_x_cm", "pa_y_cm", "pa_z_cm", "pb_x_cm", "pb_y_cm", "pb_z_cm", "va_x_cms", "va_y_cms", "va_z_cms", "vb_x_cms", "vb_y_cms", "vb_z_cms"]
        widths = [32] * 6 + [16] * 6
        statement = " ".join(f"{name}={signed_literal(width, value)};" for name, width, value in zip(names, widths, values))
        expected_valid = int(expected.tcpa_ms is not None)
        expected_tcpa = expected.tcpa_ms or 0
        assignments.append(
            f"{statement} safety_cm=16'd{safety}; horizon_ms=32'd{horizon}; #1; "
            f"if (risk !== 1'b{int(expected.risk)} || tcpa_valid !== 1'b{expected_valid} || "
            f"(tcpa_valid && (tcpa_ms > 32'd{expected_tcpa + 1} || tcpa_ms + 1 < 32'd{expected_tcpa}))) begin "
            f"$display(\"FAIL case {case}: risk=%0d tcpa=%0d valid=%0d\", risk, tcpa_ms, tcpa_valid); failures=failures+1; end"
        )
    testbench = """`timescale 1ns/1ps
module tb;
  logic signed [31:0] pa_x_cm,pa_y_cm,pa_z_cm,pb_x_cm,pb_y_cm,pb_z_cm;
  logic signed [15:0] va_x_cms,va_y_cms,va_z_cms,vb_x_cms,vb_y_cms,vb_z_cms;
  logic [15:0] safety_cm; logic [31:0] horizon_ms;
  logic risk; logic [31:0] tcpa_ms; logic tcpa_valid; integer failures=0;
  collision_predictor dut(.*);
  initial begin
""" + "\n".join(f"    {line}" for line in assignments) + f"""
    if (failures == 0) $display("PASS: {args.cases} RTL vectors");
    else $display("FAILURES: %0d", failures);
    $finish(failures != 0);
  end
endmodule
"""
    project = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="drone-rtl-") as temp:
        temp_path = Path(temp)
        tb_path, output_path = temp_path / "tb.sv", temp_path / "sim.out"
        tb_path.write_text(testbench, encoding="utf-8")
        subprocess.run([compiler, "-g2012", "-o", output_path, project / "rtl/collision_predictor.sv", tb_path], check=True)
        completed = subprocess.run([runtime, output_path], check=False, text=True, capture_output=True)
        print(completed.stdout, end="")
        if completed.returncode != 0:
            return completed.returncode
        pipeline_output = temp_path / "pipeline.out"
        subprocess.run(
            [compiler, "-g2012", "-o", pipeline_output,
             project / "rtl/packet_parser.sv", project / "rtl/neighbor_table.sv",
             project / "rtl/tb_packet_pipeline.sv"],
            check=True,
        )
        pipeline = subprocess.run([runtime, pipeline_output], check=False, text=True, capture_output=True)
        print(pipeline.stdout, end="")
        return pipeline.returncode


if __name__ == "__main__":
    raise SystemExit(main())
