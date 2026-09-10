from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import random
from statistics import fmean
from typing import Any

from .campaign import DEFAULT_OPERATIONAL_MINIMUM_SEPARATION_M, run_campaign
from .collision import KinematicState, Vec3, assess, brute_force_closest_approach, noisy_state
from .companion import CompanionConfig, CompanionService, global_to_local_ned
from .fixedpoint import fixed_point_assess
from .integrations import readiness
from .protocol import CrcError, ProtocolError, TelemetryFrame, decode, encode
from .scenarios import CAMPAIGN_SCENARIOS, build
from .tracking import AlphaBetaTracker, assess_uncertain
from .provenance import collect_metadata
from .reporting import verification_text


def protocol_stress(cases: int, seed: int = 401) -> dict[str, Any]:
    rng = random.Random(seed)
    detected = 0
    for sequence in range(cases):
        state = KinematicState(
            rng.randrange(65536),
            Vec3(*(rng.uniform(-10_000, 10_000) for _ in range(3))),
            Vec3(*(rng.uniform(-200, 200) for _ in range(3))),
            rng.uniform(0, 1_000_000),
        )
        raw = encode(TelemetryFrame.from_state(state, sequence=sequence))
        corrupt = bytearray(raw)
        bit = rng.randrange(len(raw) * 8)
        corrupt[bit // 8] ^= 1 << (bit % 8)
        try:
            decode(bytes(corrupt))
        except (CrcError, ProtocolError):
            detected += 1
    return {"cases": cases, "single_bit_corruptions_detected": detected, "detection_rate": detected / cases}


def prediction_stress(cases: int, seed: int = 402) -> dict[str, Any]:
    rng = random.Random(seed)
    oracle_max_error = 0.0
    tp = fp = tn = fn = 0
    for case in range(cases):
        if case % 2 == 0:
            own_velocity = Vec3(*(rng.uniform(-5, 5) for _ in range(3)))
            relative_velocity = Vec3(*(rng.uniform(-5, 5) for _ in range(3)))
            if relative_velocity.norm() < 0.5:
                relative_velocity = Vec3(1, 0, 0)
            trial = Vec3(*(rng.uniform(-1, 1) for _ in range(3)))
            perpendicular = trial - relative_velocity * (trial.dot(relative_velocity) / relative_velocity.norm2())
            if perpendicular.norm() < 1e-6:
                perpendicular = Vec3(-relative_velocity.y, relative_velocity.x, 0)
            miss = perpendicular * (rng.uniform(0, 6) / perpendicular.norm())
            target_time = rng.uniform(0.5, 7.5)
            relative_position = relative_velocity * -target_time + miss
            own = KinematicState(1, Vec3(0, 0, 0), own_velocity, 0)
            peer = KinematicState(2, own.position - relative_position, own_velocity - relative_velocity, 0)
        else:
            own = KinematicState(1, Vec3(*(rng.uniform(-30, 30) for _ in range(3))), Vec3(*(rng.uniform(-5, 5) for _ in range(3))), 0)
            peer = KinematicState(2, Vec3(*(rng.uniform(-30, 30) for _ in range(3))), Vec3(*(rng.uniform(-5, 5) for _ in range(3))), 0)
        truth = assess(own, peer, now=0, safety_distance=3, horizon=8, max_age=1)
        measured = assess(
            noisy_state(own, rng, position_sigma=0.7, velocity_sigma=0.15, clock_sigma=0),
            noisy_state(peer, rng, position_sigma=0.7, velocity_sigma=0.15, clock_sigma=0),
            now=0, safety_distance=3, horizon=8, max_age=1,
        )
        if truth.risk and measured.risk: tp += 1
        elif measured.risk: fp += 1
        elif truth.risk: fn += 1
        else: tn += 1
        if case < min(cases, 500):
            _, sample_distance = brute_force_closest_approach(own, peer, horizon=8, step=0.01)
            analytic_distance = min(
                (own.position - peer.position).norm(),
                truth.dcpa if truth.tcpa is not None and 0 <= truth.tcpa <= 8 else float("inf"),
                (own.position + own.velocity * 8 - peer.position - peer.velocity * 8).norm(),
            )
            oracle_max_error = max(oracle_max_error, abs(analytic_distance - sample_distance))
    total = tp + fp + tn + fn
    return {
        "cases": cases, "true_positive": tp, "false_positive": fp,
        "true_negative": tn, "false_negative": fn,
        "noisy_accuracy": (tp + tn) / total,
        "precision": tp / (tp + fp) if tp + fp else 1.0,
        "recall": tp / (tp + fn) if tp + fn else 1.0,
        "sampling_oracle_max_distance_error_m": oracle_max_error,
    }


def tracking_stress(cases: int, seed: int = 405) -> dict[str, Any]:
    rng = random.Random(seed)
    raw_tp = raw_fp = raw_fn = 0
    filtered_tp = filtered_fp = filtered_fn = 0
    for case in range(cases):
        miss = rng.uniform(0, 6)
        peer_initial = KinematicState(2, Vec3(12, miss, 0), Vec3(-2, 0, 0), 0)
        own = KinematicState(1, Vec3(0, 0, 0), Vec3(0, 0, 0), 1.0)
        truth = assess(own, peer_initial.at(1.0), now=1.0, safety_distance=3, horizon=6, max_age=1)
        tracker = AlphaBetaTracker()
        raw = None
        for timestamp in (0.0, 0.5, 1.0):
            raw = noisy_state(
                peer_initial.at(timestamp), rng,
                position_sigma=0.8, velocity_sigma=0.18, clock_sigma=0,
            )
            tracker.update(
                raw, timestamp + 0.12,
                position_sigma=0.8, velocity_sigma=0.18, clock_sigma=0.03,
            )
        assert raw is not None
        raw_risk = assess(own, raw, now=1.0, safety_distance=3, horizon=6, max_age=1).risk
        filtered_risk = assess_uncertain(
            own, tracker.tracks[2], now=1.0, safety_distance=3,
            horizon=6, max_age=1, sigma_multiplier=2.5,
        ).risk
        if truth.risk and raw_risk: raw_tp += 1
        elif raw_risk: raw_fp += 1
        elif truth.risk: raw_fn += 1
        if truth.risk and filtered_risk: filtered_tp += 1
        elif filtered_risk: filtered_fp += 1
        elif truth.risk: filtered_fn += 1
    return {
        "cases": cases,
        "raw": {
            "true_positive": raw_tp, "false_positive": raw_fp, "false_negative": raw_fn,
            "recall": raw_tp / (raw_tp + raw_fn) if raw_tp + raw_fn else 1.0,
            "precision": raw_tp / (raw_tp + raw_fp) if raw_tp + raw_fp else 1.0,
        },
        "filtered_uncertainty_aware": {
            "true_positive": filtered_tp, "false_positive": filtered_fp, "false_negative": filtered_fn,
            "recall": filtered_tp / (filtered_tp + filtered_fn) if filtered_tp + filtered_fn else 1.0,
            "precision": filtered_tp / (filtered_tp + filtered_fp) if filtered_tp + filtered_fp else 1.0,
        },
    }


def fixed_point_stress(cases: int, seed: int = 403) -> dict[str, Any]:
    rng = random.Random(seed)
    compared = mismatches = 0
    max_time_error_ms = max_distance_error_cm = 0
    actionable_time_error_ms = actionable_distance_error_cm = 0
    for _ in range(cases):
        own = KinematicState(1, Vec3(*(rng.uniform(-200, 200) for _ in range(3))), Vec3(*(rng.uniform(-20, 20) for _ in range(3))), 0)
        peer = KinematicState(2, Vec3(*(rng.uniform(-200, 200) for _ in range(3))), Vec3(*(rng.uniform(-20, 20) for _ in range(3))), 0)
        floating = assess(own, peer, now=0, safety_distance=3, horizon=10, max_age=1)
        fixed = fixed_point_assess(own, peer, safety_distance_cm=300, horizon_ms=10_000)
        if fixed.tcpa_ms is not None and floating.tcpa is not None and floating.tcpa >= 0:
            time_error = abs(fixed.tcpa_ms - round(floating.tcpa * 1000))
            distance_error = abs(fixed.dcpa_cm - round(floating.dcpa * 100))
            max_time_error_ms = max(max_time_error_ms, time_error)
            max_distance_error_cm = max(max_distance_error_cm, distance_error)
            if floating.tcpa <= 10:
                actionable_time_error_ms = max(actionable_time_error_ms, time_error)
                actionable_distance_error_cm = max(actionable_distance_error_cm, distance_error)
        near_boundary = abs(floating.dcpa - 3) < 0.1 or (floating.tcpa is not None and abs(floating.tcpa - 10) < 0.05)
        if not near_boundary:
            compared += 1
            mismatches += fixed.risk != floating.risk
    return {
        "cases": cases, "classification_cases_away_from_boundaries": compared,
        "classification_mismatches": mismatches,
        "mismatch_rate": mismatches / compared if compared else 0,
        "maximum_tcpa_quantization_error_ms": max_time_error_ms,
        "maximum_dcpa_quantization_error_cm": max_distance_error_cm,
        "actionable_tcpa_quantization_error_ms": actionable_time_error_ms,
        "actionable_dcpa_quantization_error_cm": actionable_distance_error_cm,
    }


def companion_stress() -> dict[str, Any]:
    class Radio:
        def __init__(self) -> None:
            self.incoming: list[bytes] = []
            self.outgoing: list[bytes] = []

        def receive(self) -> list[bytes]:
            packets, self.incoming = self.incoming, []
            return packets

        def send(self, payload: bytes) -> None:
            self.outgoing.append(payload)

        def close(self) -> None:
            pass

    class Px4:
        def __init__(self) -> None:
            self.states: list[KinematicState | None] = [
                KinematicState(1, Vec3(0, 0, 0), Vec3(1, 0, 0), 0), None,
            ]
            self.commands: list[Any] = []

        def receive_state(self, timestamp_s: float) -> KinematicState | None:
            return self.states.pop(0) if self.states else None

        def send_velocity(self, command: Any, time_boot_ms: int) -> None:
            self.commands.append((command, time_boot_ms))

        def close(self) -> None:
            pass

    radio, px4 = Radio(), Px4()
    peer = TelemetryFrame.from_state(
        KinematicState(2, Vec3(8, 0, 0), Vec3(-1, 0, 0), 0),
        sequence=1,
        boot_id=2,
    )
    valid = encode(peer)
    corrupt = bytearray(valid)
    corrupt[-1] ^= 1
    radio.incoming.extend((valid, bytes(corrupt)))
    service = CompanionService(
        CompanionConfig(1, 1, control_enabled=True, own_state_timeout_s=0.2),
        px4,
        radio,
    )
    service.step(0.0, 0.0)
    commands_while_fresh = len(px4.commands)
    service.step(0.3, 0.3)
    origin = global_to_local_ned(
        35.001, -83.999, 315,
        reference_latitude_deg=35,
        reference_longitude_deg=-84,
        reference_altitude_m=300,
    )
    checks = {
        "common_ned_conversion": abs(origin.x - 111.32) < 0.2 and abs(origin.z + 15) < 1e-9,
        "packet_received_and_forwarded": service.stats.received == 1 and service.stats.forwarded == 1,
        "corruption_rejected": service.stats.crc_rejections == 1,
        "risk_command_generated": service.stats.risk_events >= 1 and commands_while_fresh == 1,
        "stale_px4_suppresses_commands": len(px4.commands) == commands_while_fresh,
    }
    return {"checks": checks, "passed": sum(checks.values()), "total": len(checks)}


def _deterministic_acceptance(
    minimum_separation_m: float = DEFAULT_OPERATIONAL_MINIMUM_SEPARATION_M,
) -> dict[str, Any]:
    head_on_sim = build("head_on", 7)
    head_on = head_on_sim.run()
    baseline = build("no_avoidance", 7, event_logging=False).run()
    command_loss = build("command_loss", 7, event_logging=False).run()
    vertical = build("vertical_clear", 7, event_logging=False).run()
    congested = build("congested", 7, event_logging=False).run()
    crossing = build("crossing", 7, event_logging=False).run()
    multi_threat = build("multi_threat", 7, event_logging=False).run()
    gps_jump = build("gps_jump", 7, event_logging=False).run()
    clock_skew = build("clock_skew", 7, event_logging=False).run()
    node_restart = build("node_restart", 7, event_logging=False).run()
    telemetry_dropout = build("telemetry_dropout", 7, event_logging=False).run()
    alternate = any(
        event["type"] == "packet_delivered" and event["source"] == 1
        and event["receiver"] == 5 and event["time"] >= 3
        and event["path"] == [1, 3, 4, 5]
        for event in head_on_sim.events
    )
    checks = {
        "head_on_detected": head_on["prediction"]["alarms"] > 0,
        "head_on_avoided": head_on["avoidance"]["minimum_separation_m"] >= minimum_separation_m,
        "unmitigated_head_on_collides": baseline["avoidance"]["minimum_separation_m"] < 0.1,
        "three_hop_failover": alternate,
        "command_loss_times_out": command_loss["avoidance"]["command_failures"] == 1,
        "vertical_separation_no_alarm": vertical["prediction"]["alarms"] == 0,
        "aloha_congestion_detected": congested["network"]["delivery_ratio"] < 0.1,
        "crossing_avoided": crossing["avoidance"]["minimum_separation_m"] >= minimum_separation_m,
        "simultaneous_threats_avoided": multi_threat["avoidance"]["minimum_separation_m"] >= minimum_separation_m,
        "gps_jump_recovered_safely": gps_jump["avoidance"]["minimum_separation_m"] >= minimum_separation_m,
        "clock_skew_recovered_safely": clock_skew["avoidance"]["minimum_separation_m"] >= minimum_separation_m,
        "reboot_sequence_reset_accepted": (
            node_restart["network"]["node_reboots"] == 1
            and node_restart["avoidance"]["minimum_separation_m"] >= minimum_separation_m
        ),
        "telemetry_dropout_survived": (
            telemetry_dropout["network"]["telemetry_suppressed"] > 0
            and telemetry_dropout["avoidance"]["minimum_separation_m"] >= minimum_separation_m
        ),
    }
    return {
        "minimum_separation_requirement_m": minimum_separation_m,
        "checks": checks,
        "passed": sum(checks.values()),
        "total": len(checks),
    }


def run_full_verification(
    cases: int,
    campaign_seeds: int,
    workers: int,
    output: str | Path,
    endurance_seeds: int = 3,
) -> dict[str, Any]:
    output_path = Path(output)
    output_path.mkdir(parents=True, exist_ok=True)
    tools = readiness()
    endurance_runs = [
        build("endurance", seed, event_logging=False).run()
        for seed in range(7, 7 + endurance_seeds)
    ]
    report = {
        "metadata": collect_metadata(
            "drone-sims verify",
            seeds=range(1, campaign_seeds + 1),
            parameters={
                "cases": cases,
                "campaign_seeds": campaign_seeds,
                "endurance_seeds": endurance_seeds,
                "workers": workers,
                "operational_minimum_separation_m": DEFAULT_OPERATIONAL_MINIMUM_SEPARATION_M,
            },
        ),
        "protocol": protocol_stress(cases),
        "prediction": prediction_stress(cases),
        "tracking": tracking_stress(cases),
        "fixed_point": fixed_point_stress(cases),
        "companion_service": companion_stress(),
        "deterministic_acceptance": _deterministic_acceptance(),
        "network_and_system_campaign": run_campaign(list(CAMPAIGN_SCENARIOS), campaign_seeds, workers),
        "endurance_300s": endurance_runs[0],
        "endurance_campaign": {
            "runs": endurance_runs,
            "seeds": list(range(7, 7 + endurance_seeds)),
            "delivery_ratio_min": min(run["network"]["delivery_ratio"] for run in endurance_runs),
            "delivery_ratio_mean": fmean(run["network"]["delivery_ratio"] for run in endurance_runs),
            "minimum_separation_worst_m": min(
                run["avoidance"]["minimum_separation_m"] for run in endurance_runs
            ),
        },
        "integration_readiness": tools,
    }
    px4_report_path = output_path / "px4_sitl.json"
    if px4_report_path.exists():
        report["px4_sitl"] = json.loads(px4_report_path.read_text(encoding="utf-8"))
    (output_path / "verification.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_path / "results.txt").write_text(verification_text(report), encoding="utf-8")
    return report
