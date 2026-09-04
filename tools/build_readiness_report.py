#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


SAFE_SCENARIOS = (
    "head_on", "noisy", "limited_dynamics", "crossing", "vertical_clear",
    "partition", "multi_threat", "gps_jump", "clock_skew", "node_restart",
    "telemetry_dropout",
)


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate frozen pre-hardware acceptance gates")
    parser.add_argument("--results", type=Path, default=Path("results/full"))
    parser.add_argument("--requirements", type=Path, default=Path("requirements/pre_hardware_acceptance.json"))
    args = parser.parse_args()
    requirements = load(args.requirements)
    verification = load(args.results / "verification.json")
    network = load(args.results / "network_matrix.json")
    synthesis = load(args.results / "fpga_synthesis.json")
    px4 = load(args.results / "px4_sitl.json")
    closed_loop = load(args.results / "px4_closed_loop.json")

    campaign = verification["network_and_system_campaign"]["scenarios"]
    recommendation = network["recommendation"]
    tracking = verification["tracking"]["filtered_uncertainty_aware"]
    endurance = verification["endurance_300s"]
    gates = {
        "protocol_crc": verification["protocol"]["detection_rate"] >= requirements["protocol_crc_detection_rate_min"],
        "fixed_point_equivalence": verification["fixed_point"]["mismatch_rate"] <= requirements["fixed_point_mismatch_rate_max"],
        "uncertainty_recall": tracking["recall"] >= requirements["uncertainty_aware_recall_min"],
        "uncertainty_precision": tracking["precision"] >= requirements["uncertainty_aware_precision_min"],
        "deterministic_scenarios": verification["deterministic_acceptance"]["passed"] == verification["deterministic_acceptance"]["total"],
        "campaign_safety": all(
            campaign[name]["safety_distance_success_rate"] >= requirements["safe_scenario_success_rate_min"]
            for name in SAFE_SCENARIOS
        ),
        "endurance_delivery": verification.get("endurance_campaign", {}).get(
            "delivery_ratio_min", endurance["network"]["delivery_ratio"]
        ) >= requirements["endurance_delivery_ratio_min"],
        "network_delivery": recommendation["delivery_ratio_mean"] >= requirements["network_recommendation_delivery_ratio_min"],
        "network_latency": recommendation["latency_ms_mean"] <= requirements["network_recommendation_latency_ms_max"],
        "rtl_behavior": verification["rtl_execution"]["passed"],
        "rtl_synthesis": synthesis["passed"],
        "px4_lifecycle": px4["passed"] == px4["total"],
        "px4_closed_loop": closed_loop["passed"] == closed_loop["total"],
        "px4_actual_separation": closed_loop["actual_minimum_separation_m"] >= requirements["minimum_separation_m"],
    }
    passed = sum(gates.values())
    report = {
        "status": "PASS" if passed == len(gates) else "FAIL",
        "passed": passed,
        "total": len(gates),
        "gates": gates,
        "recommended_network": recommendation,
        "tracking": verification["tracking"],
        "metadata": verification.get("metadata"),
        "campaign_safety": {name: campaign[name]["safety_distance_success_rate"] for name in SAFE_SCENARIOS},
        "remaining_hardware_only": [
            "RF range, interference, antenna placement and regional duty-cycle compliance",
            "GNSS multipath/jamming behavior with the selected receiver and airframe",
            "FPGA timing closure, power, pin constraints and board-level I/O",
            "Autopilot/FPGA electrical integration and real flight-controller failsafes",
            "Propulsion, battery, vibration, mass, weather and flight-test validation"
        ],
    }
    (args.results / "pre_hardware_readiness.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    lines = [
        "# Pre-hardware readiness", "",
        f"Overall simulated status: **{report['status']} ({passed}/{len(gates)} gates)**", "",
        "## Acceptance gates", "",
    ]
    lines.extend(f"- {'PASS' if value else 'FAIL'} — `{name}`" for name, value in gates.items())
    lines += [
        "", "## Selected network baseline", "",
        f"- {recommendation['phy']} LoRa profile, {recommendation['mac'].upper()}, {recommendation['routing']} routing",
        f"- {recommendation['telemetry_interval_s']:.1f} s telemetry; {recommendation['time_on_air_ms']:.3f} ms/frame",
        f"- Mean PDR {recommendation['delivery_ratio_mean']:.3f}; mean latency {recommendation['latency_ms_mean']:.1f} ms",
        "", "## Hardware-only work still required", "",
    ]
    lines.extend(f"- {item}" for item in report["remaining_hardware_only"])
    lines += ["", "A PASS means the available software, network, RTL behavioral, synthesis-elaboration, and PX4 SIH gates passed. It is not flight certification.", ""]
    (args.results / "PRE_HARDWARE_READINESS.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
