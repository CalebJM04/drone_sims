#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import signal
import time

from pymavlink import mavutil


EARTH_RADIUS_M = 6_378_137.0


def local_to_global(
    north_m: float,
    east_m: float,
    down_m: float,
    *,
    reference_latitude_deg: float,
    reference_longitude_deg: float,
    reference_altitude_m: float,
) -> tuple[int, int, int]:
    latitude = reference_latitude_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = reference_longitude_deg + math.degrees(
        east_m
        / (EARTH_RADIUS_M * math.cos(math.radians(reference_latitude_deg)))
    )
    altitude = reference_altitude_m - down_m
    return round(latitude * 1e7), round(longitude * 1e7), round(altitude * 1000)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", required=True, help="companion UDP host:port")
    parser.add_argument("--node-id", type=int, required=True)
    parser.add_argument("--north", type=float, required=True)
    parser.add_argument("--east", type=float, default=0.0)
    parser.add_argument("--down", type=float, default=0.0)
    parser.add_argument("--vn", type=float, required=True)
    parser.add_argument("--ve", type=float, default=0.0)
    parser.add_argument("--vd", type=float, default=0.0)
    parser.add_argument("--reference-latitude", type=float, default=35.9606)
    parser.add_argument("--reference-longitude", type=float, default=-83.9207)
    parser.add_argument("--reference-altitude", type=float, default=300.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument(
        "--start-delay",
        type=float,
        default=0.0,
        help="hold the initial position before beginning the trajectory",
    )
    parser.add_argument("--dropout-start", type=float)
    parser.add_argument("--dropout-duration", type=float, default=0.0)
    parser.add_argument("--gps-jump-at", type=float)
    parser.add_argument("--gps-jump-north", type=float, default=50.0)
    parser.add_argument("--log-all-commands", action="store_true")
    args = parser.parse_args()

    connection = mavutil.mavlink_connection(
        f"udpout:{args.target}",
        source_system=args.node_id,
        source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
    )
    started = time.monotonic()
    running = True
    next_heartbeat = started
    next_status = started
    next_gps = started
    next_position = started
    next_report = started
    commands = 0
    nonzero_commands = 0
    positions_sent = 0
    gps_jump_applied = False
    last_command_velocity: tuple[float, float, float] | None = None

    def stop(_signal: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    gps_bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS

    try:
        while running:
            now = time.monotonic()
            elapsed = now - started
            trajectory_time = max(0.0, elapsed - args.start_delay)
            if args.duration > 0 and elapsed >= args.duration:
                break
            dropout = (
                args.dropout_start is not None
                and args.dropout_start <= elapsed
                < args.dropout_start + args.dropout_duration
            )
            jump = (
                args.gps_jump_at is not None and elapsed >= args.gps_jump_at
            )
            gps_jump_applied = gps_jump_applied or jump
            north = args.north + args.vn * trajectory_time
            if jump:
                north += args.gps_jump_north
            east = args.east + args.ve * trajectory_time
            down = args.down + args.vd * trajectory_time
            latitude, longitude, altitude = local_to_global(
                north,
                east,
                down,
                reference_latitude_deg=args.reference_latitude,
                reference_longitude_deg=args.reference_longitude,
                reference_altitude_m=args.reference_altitude,
            )
            boot_ms = round(elapsed * 1000) & 0xFFFFFFFF

            if now >= next_heartbeat:
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_QUADROTOR,
                    mavutil.mavlink.MAV_AUTOPILOT_PX4,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                next_heartbeat += 1.0
            if now >= next_status:
                connection.mav.sys_status_send(
                    gps_bit,
                    gps_bit,
                    gps_bit,
                    100,
                    12_000,
                    500,
                    90,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                next_status += 1.0
            if not dropout and now >= next_gps:
                connection.mav.gps_raw_int_send(
                    round(time.time() * 1_000_000),
                    3,
                    latitude,
                    longitude,
                    altitude,
                    100,
                    150,
                    round(math.hypot(args.vn, args.ve) * 100),
                    0,
                    16,
                )
                next_gps += 0.2
            if not dropout and now >= next_position:
                connection.mav.global_position_int_send(
                    boot_ms,
                    latitude,
                    longitude,
                    altitude,
                    round(-down * 1000),
                    round(args.vn * 100),
                    round(args.ve * 100),
                    round(args.vd * 100),
                    0,
                )
                positions_sent += 1
                next_position += 0.1

            while True:
                message = connection.recv_match(blocking=False)
                if message is None:
                    break
                if message.get_type() != "SET_POSITION_TARGET_LOCAL_NED":
                    continue
                commands += 1
                velocity = (float(message.vx), float(message.vy), float(message.vz))
                if any(abs(component) > 1e-6 for component in velocity):
                    nonzero_commands += 1
                if args.log_all_commands or velocity != last_command_velocity:
                    print(
                        json.dumps(
                            {
                                "event": "velocity_command",
                                "node": args.node_id,
                                "elapsed_s": round(elapsed, 3),
                                "velocity_ned_mps": velocity,
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                last_command_velocity = velocity

            if now >= next_report:
                print(
                    json.dumps(
                        {
                            "event": "fake_px4",
                            "node": args.node_id,
                            "elapsed_s": round(elapsed, 3),
                            "dropout": dropout,
                            "position_ned_m": [
                                round(north, 3),
                                round(east, 3),
                                round(down, 3),
                            ],
                            "positions_sent": positions_sent,
                            "commands_received": commands,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                next_report += 5.0
            time.sleep(0.005)
    finally:
        connection.close()

    print(
        json.dumps(
            {
                "event": "summary",
                "node": args.node_id,
                "positions_sent": positions_sent,
                "commands_received": commands,
                "nonzero_commands_received": nonzero_commands,
                "gps_jump_applied": gps_jump_applied,
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
