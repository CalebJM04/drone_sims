#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from drone_sims.provenance import source_digest
from drone_sims.reporting import final_results_text


SAFE_SCENARIOS = (
    "head_on", "noisy", "limited_dynamics", "crossing", "vertical_clear",
    "partition", "multi_threat", "gps_jump", "clock_skew", "node_restart",
    "telemetry_dropout",
)


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate frozen software acceptance gates")
    parser.add_argument("--results", type=Path, default=Path("results/full"))
    parser.add_argument("--requirements", type=Path, default=Path("requirements/pre_hardware_acceptance.json"))
    args = parser.parse_args()
    requirements = load(args.requirements)
    verification = load(args.results / "verification.json")
    network = load(args.results / "network_matrix.json")
    px4 = load(args.results / "px4_sitl.json")
    closed_loop = load(args.results / "px4_closed_loop.json")
    current_source_digest = source_digest()
    evidence = (verification, network, px4, closed_loop)

    campaign = verification["network_and_system_campaign"]["scenarios"]
    recommendation = network["recommendation"]
    tracking = verification["tracking"]["filtered_uncertainty_aware"]
    endurance = verification["endurance_300s"]
    mesh = verification["mesh_demo"]
    evidence = (*evidence, mesh)
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
        "companion_service": verification["companion_service"]["passed"] == verification["companion_service"]["total"],
        "px4_lifecycle": px4["passed"] == px4["total"],
        "px4_closed_loop": closed_loop["passed"] == closed_loop["total"],
        "px4_actual_separation": closed_loop["actual_minimum_separation_m"] >= requirements["minimum_separation_m"],
        "evidence_source_match": all(
            isinstance(item.get("metadata"), dict)
            and item["metadata"].get("source_sha256") == current_source_digest
            for item in evidence
        ),
        "mesh_node_count": mesh["checks"]["minimum_nodes"],
        "mesh_delivery": mesh["checks"]["delivery_ratio"],
        "mesh_latency": mesh["checks"]["p95_latency"],
        "mesh_state_age": mesh["checks"]["state_age"],
        "proactive_route_handoff": mesh["checks"]["preemptive_route"],
        "collision_awareness": mesh["checks"]["collision_awareness"],
        "mesh_observation_only": mesh["checks"]["observation_only"],
        "mesh_protocol_integrity": mesh["checks"]["protocol_integrity"],
    }
    passed = sum(gates.values())
    report = {
        "status": "PASS" if passed == len(gates) else "FAIL",
        "passed": passed,
        "total": len(gates),
        "gates": gates,
        "recommended_network": recommendation,
        "tracking": verification["tracking"],
        "mesh_demo": mesh["summary"],
        "metadata": verification.get("metadata"),
        "evidence_source_sha256": current_source_digest,
        "campaign_safety": {name: campaign[name]["safety_distance_success_rate"] for name in SAFE_SCENARIOS},
        "outside_software_scope": [
            "RF range, interference, antenna placement and regional duty-cycle compliance",
            "GNSS multipath/jamming behavior with the selected receiver and airframe",
            "Raspberry Pi power, thermal, storage and process-watchdog validation",
            "Companion/PX4 electrical integration and real flight-controller failsafes",
            "Propulsion, battery, vibration, mass, weather and flight-test validation"
        ],
    }
    (args.results / "pre_hardware_readiness.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (args.results / "results.txt").write_text(
        final_results_text(report, verification, network, px4, closed_loop),
        encoding="utf-8",
    )
    print(f"{report['status']}: {passed}/{len(gates)} checks passed")
    print(f"Results: {args.results / 'results.txt'}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
