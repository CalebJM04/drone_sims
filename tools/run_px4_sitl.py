#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import uuid

from pymavlink import mavutil

from drone_sims.mavlink_codec import VelocityCommand, encode_velocity_command
from drone_sims.provenance import PINNED_PX4_IMAGE, collect_metadata


PX4_IMAGE = PINNED_PX4_IMAGE
ACCEPTED = mavutil.mavlink.MAV_RESULT_ACCEPTED


def wait_command_ack(link, command: int, timeout: float = 5.0) -> int | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.25)
        if message is not None and message.command == command:
            return int(message.result)
    return None


def send_command(link, command: int, *parameters: float) -> int | None:
    values = list(parameters) + [0.0] * (7 - len(parameters))
    link.mav.command_long_send(link.target_system, 1, command, 0, *values)
    return wait_command_ack(link, command)


def sample_position(link, seconds: float, velocity: VelocityCommand | None = None) -> tuple[list[float] | None, list[tuple[int, bool]]]:
    deadline = time.monotonic() + seconds
    position = None
    modes: list[tuple[int, bool]] = []
    while time.monotonic() < deadline:
        if velocity is not None:
            packet = encode_velocity_command(
                velocity,
                int(time.monotonic() * 1000) & 0xFFFFFFFF,
            )
            link.write(packet)
        interval_end = time.monotonic() + 0.05
        while time.monotonic() < interval_end:
            message = link.recv_match(blocking=False)
            if message is not None and message.get_type() == "LOCAL_POSITION_NED":
                position = [message.x, message.y, message.z, message.vx, message.vy, message.vz]
            elif message is not None and message.get_type() == "HEARTBEAT":
                modes.append(
                    (
                        (int(message.custom_mode) >> 16) & 0xFF,
                        bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                    )
                )
            time.sleep(0.003)
    return position, modes


def wait_for_landing(link, timeout: float = 20.0) -> tuple[list[float] | None, bool, list[tuple[int, bool]]]:
    deadline = time.monotonic() + timeout
    position = None
    landed = False
    modes: list[tuple[int, bool]] = []
    while time.monotonic() < deadline and not landed:
        message = link.recv_match(blocking=True, timeout=0.25)
        if message is None:
            continue
        if message.get_type() == "LOCAL_POSITION_NED":
            position = [message.x, message.y, message.z, message.vx, message.vy, message.vz]
        elif message.get_type() == "HEARTBEAT":
            modes.append(
                (
                    (int(message.custom_mode) >> 16) & 0xFF,
                    bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                )
            )
        elif message.get_type() == "EXTENDED_SYS_STATE":
            landed = message.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    return position, landed, modes


def run() -> dict[str, object]:
    container = f"drone-sims-px4-{uuid.uuid4().hex[:8]}"
    subprocess.run(
        ["docker", "run", "--rm", "-d", "--network", "host", "--name", container, PX4_IMAGE],
        check=True,
        text=True,
        capture_output=True,
    )
    link = None
    try:
        link = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=240)
        heartbeat = link.wait_heartbeat(timeout=15)
        if heartbeat is None:
            raise RuntimeError("PX4 did not produce a MAVLink heartbeat")
        initial, startup_modes = sample_position(link, 5.0)
        if initial is None:
            raise RuntimeError("PX4 did not produce LOCAL_POSITION_NED")
        global_position = link.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=5.0
        )
        if global_position is None:
            raise RuntimeError("PX4 did not produce GLOBAL_POSITION_INT")
        takeoff_altitude_amsl = global_position.alt / 1000.0 + 3.0

        arm_ack = None
        armed_modes: list[tuple[int, bool]] = []
        for _ in range(5):
            arm_ack = send_command(link, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
            _, new_modes = sample_position(link, 2.0)
            armed_modes.extend(new_modes)
            if arm_ack == ACCEPTED and any(armed for _, armed in armed_modes):
                break
        takeoff_ack = send_command(
            link,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0.0, 0.0, 0.0, 0.0, math.nan, math.nan, takeoff_altitude_amsl,
        )
        airborne, takeoff_modes = sample_position(link, 10.0)
        if airborne is None:
            raise RuntimeError("PX4 stopped producing local position during takeoff")

        zero = VelocityCommand(0.0, 0.0, 0.0)
        sample_position(link, 2.0, zero)
        link.set_mode("OFFBOARD")
        before_avoidance, offboard_modes = sample_position(link, 1.0, zero)
        after_avoidance, avoidance_modes = sample_position(
            link, 4.0, VelocityCommand(0.0, 1.5, 0.0)
        )

        land_ack = send_command(link, mavutil.mavlink.MAV_CMD_NAV_LAND)
        landed_position, landed_state, land_modes = wait_for_landing(link)
        disarm_ack = send_command(
            link, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0
        )

        altitude_gain = initial[2] - airborne[2]
        lateral_response = None
        if before_avoidance is not None and after_avoidance is not None:
            lateral_response = math.hypot(
                after_avoidance[0] - before_avoidance[0],
                after_avoidance[1] - before_avoidance[1],
            )
        observed_modes = startup_modes + armed_modes + takeoff_modes + offboard_modes + avoidance_modes + land_modes
        checks = {
            "heartbeat_received": True,
            "local_position_received": True,
            "arm_accepted": arm_ack == ACCEPTED,
            "takeoff_accepted": takeoff_ack == ACCEPTED,
            "became_armed": any(armed for _, armed in observed_modes),
            "takeoff_altitude_gain_at_least_1m": altitude_gain >= 1.0,
            "offboard_mode_observed": any(main_mode == 6 for main_mode, _ in observed_modes),
            "avoidance_setpoints_streamed": after_avoidance is not None,
            "lateral_response_at_least_0_5m": lateral_response is not None and lateral_response >= 0.5,
            "land_accepted": land_ack == ACCEPTED,
            "landed_state_observed": landed_state,
            "disarm_accepted": disarm_ack == ACCEPTED,
        }
        return {
            "metadata": collect_metadata(
                "tools/run_px4_sitl.py", px4_image=PX4_IMAGE,
            ),
            "image": PX4_IMAGE,
            "checks": checks,
            "passed": sum(checks.values()),
            "total": len(checks),
            "arm_ack": arm_ack,
            "takeoff_ack": takeoff_ack,
            "land_ack": land_ack,
            "disarm_ack": disarm_ack,
            "initial_local_position": initial,
            "airborne_local_position": airborne,
            "before_avoidance_local_position": before_avoidance,
            "after_avoidance_local_position": after_avoidance,
            "landed_local_position": landed_position,
            "altitude_gain_m": altitude_gain,
            "commanded_takeoff_altitude_amsl_m": takeoff_altitude_amsl,
            "lateral_response_m": lateral_response,
        }
    finally:
        if link is not None:
            link.close()
        subprocess.run(["docker", "stop", container], check=False, text=True, capture_output=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run live MAVLink checks against official PX4 SIH")
    parser.add_argument("--output", type=Path, default=Path("results/full/px4_sitl.json"))
    args = parser.parse_args()
    report = run()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] == report["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
