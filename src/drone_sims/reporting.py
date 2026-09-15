from __future__ import annotations

from typing import Any


def verification_text(report: dict[str, Any]) -> str:
    checks = report["deterministic_acceptance"]
    campaign = report["network_and_system_campaign"]
    protocol = report["protocol"]
    fixed_point = report["fixed_point"]
    companion = report["companion_service"]
    endurance = report["endurance_campaign"]

    lines = [
        "Drone simulation results",
        "",
        f"Simulation checks: {checks['passed']}/{checks['total']} passed",
        f"Scenario runs: {campaign['runs']}",
        (
            "Corrupted packets caught: "
            f"{protocol['single_bit_corruptions_detected']}/{protocol['cases']}"
        ),
        (
            "Fixed-point mismatches: "
            f"{fixed_point['classification_mismatches']}/"
            f"{fixed_point['classification_cases_away_from_boundaries']}"
        ),
        f"Companion checks: {companion['passed']}/{companion['total']} passed",
        f"Worst endurance packet delivery: {endurance['delivery_ratio_min'] * 100:.2f}%",
        "",
        "Scenario results",
    ]
    for name, values in campaign["scenarios"].items():
        lines.append(
            f"{name}: {values['minimum_separation_worst_m']:.2f} m minimum separation, "
            f"{values['safety_distance_success_rate'] * 100:.0f}% safe runs"
        )

    if "px4_sitl" in report:
        px4 = report["px4_sitl"]
        lines += ["", f"PX4 test: {px4['passed']}/{px4['total']} passed"]

    lines += [
        "",
        "These are simulation results. Hardware and real flight testing are still needed.",
        "",
    ]
    return "\n".join(lines)


def final_results_text(
    readiness: dict[str, Any],
    verification: dict[str, Any],
    network: dict[str, Any],
    px4: dict[str, Any],
    closed_loop: dict[str, Any],
) -> str:
    deterministic = verification["deterministic_acceptance"]
    campaign = verification["network_and_system_campaign"]
    protocol = verification["protocol"]
    fixed_point = verification["fixed_point"]
    companion = verification["companion_service"]
    endurance = verification["endurance_campaign"]
    tracking = verification["tracking"]["filtered_uncertainty_aware"]
    recommendation = network["recommendation"]
    mesh = verification.get("mesh_demo")

    required = (
        "head_on", "noisy", "limited_dynamics", "crossing", "vertical_clear",
        "partition", "multi_threat", "gps_jump", "clock_skew", "node_restart",
        "telemetry_dropout",
    )
    closest_name = min(
        required,
        key=lambda name: campaign["scenarios"][name]["minimum_separation_worst_m"],
    )
    closest = campaign["scenarios"][closest_name]["minimum_separation_worst_m"]

    lines = [
        "Drone simulation results",
        "",
        f"Overall: {readiness['status']} ({readiness['passed']}/{readiness['total']} checks passed)",
        f"Simulation checks: {deterministic['passed']}/{deterministic['total']} passed",
        f"Scenario runs: {campaign['runs']}",
        (
            "Corrupted packets caught: "
            f"{protocol['single_bit_corruptions_detected']}/{protocol['cases']}"
        ),
        (
            "Fixed-point mismatches: "
            f"{fixed_point['classification_mismatches']}/"
            f"{fixed_point['classification_cases_away_from_boundaries']}"
        ),
        f"Companion checks: {companion['passed']}/{companion['total']} passed",
        f"Worst endurance packet delivery: {endurance['delivery_ratio_min'] * 100:.2f}%",
        f"Tracking recall: {tracking['recall'] * 100:.1f}%",
        f"Closest required scenario: {closest_name} at {closest:.2f} m",
        "",
        "Selected network setup",
        (
            f"{recommendation['phy']} LoRa, {recommendation['mac'].upper()}, "
            f"{recommendation['routing']}, {recommendation['telemetry_interval_s']:.1f} second updates"
        ),
        f"Average packet delivery: {recommendation['delivery_ratio_mean'] * 100:.1f}%",
        f"Average delay: {recommendation['latency_ms_mean']:.0f} ms",
        "",
        *(
            [
                "Primary 4+ node mesh demo",
                (
                    f"{mesh['summary']['nodes']} nodes, "
                    f"{mesh['summary']['delivery_ratio'] * 100:.1f}% delivery, "
                    f"{mesh['summary']['p95_latency_ms']:.0f} ms p95, "
                    f"{mesh['summary']['maximum_state_age_s']:.2f} s maximum state age"
                ),
                (
                    f"Handoffs: {mesh['summary']['preemptive_route_handoffs']}; "
                    f"collision alerts: {mesh['summary']['collision_awareness_alerts']}; "
                    f"control commands: {mesh['summary']['control_commands']}"
                ),
                "",
            ]
            if mesh else []
        ),
        f"PX4 basic test: {px4['passed']}/{px4['total']} passed",
        f"PX4 collision test: {closed_loop['passed']}/{closed_loop['total']} passed",
        f"PX4 minimum separation: {closed_loop['actual_minimum_separation_m']:.2f} m",
        "",
        "The software and simulation checks passed. This does not mean the system is ready",
        "for real flights. Radio range, GNSS, Raspberry Pi power, Pixhawk wiring, and the",
        "actual aircraft still need to be tested.",
        "",
    ]
    return "\n".join(lines)
