#!/usr/bin/env python3
from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path
import random
import subprocess
import time
import uuid

from pymavlink import mavutil

from drone_sims.collision import KinematicState, Vec3
from drone_sims.campaign import DEFAULT_OPERATIONAL_MINIMUM_SEPARATION_M
from drone_sims.lora_phy import COMMON_PROFILES
from drone_sims.mavlink_codec import VelocityCommand, encode_velocity_command
from drone_sims.planner import PlannerConfig, plan_maneuver
from drone_sims.protocol import TelemetryFrame, decode, encode
from drone_sims.provenance import collect_metadata
from drone_sims.tracking import AlphaBetaTracker, assess_uncertain
from run_px4_sitl import ACCEPTED, PX4_IMAGE, sample_position, send_command, wait_for_landing


def run(seed: int = 401) -> dict[str, object]:
    rng = random.Random(seed)
    container = f"drone-sims-closed-loop-{uuid.uuid4().hex[:8]}"
    subprocess.run(
        ["docker", "run", "--rm", "-d", "--network", "host", "--name", container, PX4_IMAGE],
        check=True, text=True, capture_output=True,
    )
    link = None
    try:
        link = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=241)
        if link.wait_heartbeat(timeout=15) is None:
            raise RuntimeError("PX4 heartbeat timeout")
        initial, _ = sample_position(link, 5.0)
        global_position = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5.0)
        if initial is None or global_position is None:
            raise RuntimeError("PX4 position telemetry timeout")
        arm_ack = send_command(link, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
        takeoff_ack = send_command(
            link, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0.0, 0.0, 0.0, 0.0, math.nan, math.nan,
            global_position.alt / 1000.0 + 3.0,
        )
        airborne, _ = sample_position(link, 10.0)
        if airborne is None:
            raise RuntimeError("PX4 did not become airborne")
        hold = VelocityCommand(0, 0, 0)
        sample_position(link, 2.0, hold)
        link.set_mode("OFFBOARD")
        scenario_origin, modes = sample_position(link, 1.0, hold)
        if scenario_origin is None:
            raise RuntimeError("PX4 local position unavailable before scenario")

        modem = COMMON_PROFILES["fast"]
        tracker = AlphaBetaTracker()
        pending: list[tuple[float, bytes]] = []
        scenario_start = time.monotonic()
        end_at = scenario_start + 9.0
        next_telemetry = 0.0
        sequence = 0
        sent = delivered = dropped = 0
        latencies: list[float] = []
        current_command = hold
        chosen_name = None
        alarm_at = None
        command_at = None
        latest = scenario_origin
        minimum_separation = math.inf
        planner_separation = None

        while time.monotonic() < end_at:
            elapsed = time.monotonic() - scenario_start
            if elapsed + 1e-9 >= next_telemetry:
                threat = KinematicState(
                    2,
                    Vec3(scenario_origin[0] + 12.0 - 2.0 * next_telemetry,
                         scenario_origin[1], scenario_origin[2]),
                    Vec3(-2.0, 0.0, 0.0),
                    next_telemetry,
                )
                packet = encode(TelemetryFrame.from_state(
                    threat, sequence=sequence, boot_id=0x401, ttl=3,
                ))
                sent += 1
                if rng.random() < 0.05:
                    dropped += 1
                else:
                    delay = modem.airtime_seconds(len(packet)) + 0.010 + rng.uniform(0, 0.040)
                    heapq.heappush(pending, (next_telemetry + delay, packet))
                sequence = (sequence + 1) & 0xFFFF
                next_telemetry += 1.0

            while pending and pending[0][0] <= elapsed:
                arrived_at, packet = heapq.heappop(pending)
                frame = decode(packet)
                delivered += 1
                latencies.append(arrived_at - frame.timestamp_ms / 1000.0)
                tracked = tracker.update(
                    frame.to_state(), arrived_at,
                    position_sigma=0.20, velocity_sigma=0.08, clock_sigma=0.02,
                )
                own = KinematicState(
                    1, Vec3(latest[0], latest[1], latest[2]),
                    Vec3(latest[3], latest[4], latest[5]), elapsed,
                )
                risk = assess_uncertain(
                    own, tracked, now=elapsed, safety_distance=3.0,
                    horizon=5.0, max_age=1.5, sigma_multiplier=2.5,
                )
                if risk.risk and alarm_at is None:
                    alarm_at = elapsed
                    chosen, _ = plan_maneuver(
                        own, [tracked.state.at(elapsed)],
                        PlannerConfig(horizon=5.0, safety_distance=3.0),
                    )
                    chosen_name = chosen.name
                    planner_separation = chosen.worst_separation
                    current_command = VelocityCommand(
                        chosen.velocity.x, chosen.velocity.y, chosen.velocity.z,
                    )
                    command_at = time.monotonic() - scenario_start

            link.write(encode_velocity_command(current_command, int(time.monotonic() * 1000) & 0xFFFFFFFF))
            interval_end = time.monotonic() + 0.05
            while time.monotonic() < interval_end:
                message = link.recv_match(blocking=False)
                if message is not None and message.get_type() == "LOCAL_POSITION_NED":
                    latest = [message.x, message.y, message.z, message.vx, message.vy, message.vz]
                time.sleep(0.002)
            elapsed = time.monotonic() - scenario_start
            threat_now = Vec3(
                scenario_origin[0] + 12.0 - 2.0 * elapsed,
                scenario_origin[1], scenario_origin[2],
            )
            minimum_separation = min(
                minimum_separation,
                (Vec3(latest[0], latest[1], latest[2]) - threat_now).norm(),
            )

        land_ack = send_command(link, mavutil.mavlink.MAV_CMD_NAV_LAND)
        landed_position, landed, landing_modes = wait_for_landing(link)
        disarm_ack = send_command(link, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        lateral_response = math.hypot(
            latest[0] - scenario_origin[0], latest[1] - scenario_origin[1]
        )
        checks = {
            "arm_accepted": arm_ack == ACCEPTED,
            "takeoff_accepted": takeoff_ack == ACCEPTED,
            "offboard_observed": any(mode == 6 for mode, _ in modes),
            "lora_packets_delivered": delivered > 0,
            "uncertainty_aware_alarm": alarm_at is not None,
            "planner_selected_maneuver": chosen_name not in (None, "continue"),
            "mavlink_command_streamed": command_at is not None,
            "px4_lateral_response_at_least_1m": lateral_response >= 1.0,
            "actual_separation_at_least_4m": (
                minimum_separation >= DEFAULT_OPERATIONAL_MINIMUM_SEPARATION_M
            ),
            "land_accepted": land_ack == ACCEPTED,
            "landed_state_observed": landed,
            "disarm_accepted": disarm_ack == ACCEPTED,
        }
        return {
            "metadata": collect_metadata(
                "tools/run_px4_closed_loop.py", seeds=[seed],
                parameters={"seed": seed}, px4_image=PX4_IMAGE,
            ),
            "image": PX4_IMAGE,
            "seed": seed,
            "checks": checks,
            "passed": sum(checks.values()),
            "total": len(checks),
            "network": {
                "profile": "fast",
                "time_on_air_ms": modem.airtime_seconds(36) * 1000,
                "sent": sent, "delivered": delivered, "dropped": dropped,
                "mean_latency_ms": sum(latencies) / len(latencies) * 1000 if latencies else None,
            },
            "alarm_time_s": alarm_at,
            "command_time_s": command_at,
            "selected_maneuver": chosen_name,
            "planner_predicted_separation_m": planner_separation,
            "actual_minimum_separation_m": minimum_separation,
            "actual_lateral_response_m": lateral_response,
            "scenario_origin": scenario_origin,
            "final_airborne_position": latest,
            "landed_position": landed_position,
        }
    finally:
        if link is not None:
            link.close()
        subprocess.run(["docker", "stop", container], check=False, text=True, capture_output=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run LoRa-to-planner-to-MAVLink closed loop against PX4 SIH")
    parser.add_argument("--seed", type=int, default=401)
    parser.add_argument("--output", type=Path, default=Path("results/full/px4_closed_loop.json"))
    args = parser.parse_args()
    report = run(args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] == report["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
