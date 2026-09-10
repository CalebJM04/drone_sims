from __future__ import annotations

import json
import math
from pathlib import Path
import random
import signal
import time
import tomllib
from typing import Any

from .companion import CompanionConfig, CompanionService
from .companion_io import MavlinkPx4, SerialFrameRadio, SystemdNotifier, UdpRadio


def _section(document: dict[str, Any], name: str) -> dict[str, Any]:
    value = document.get(name, {})
    if not isinstance(value, dict):
        raise ValueError(f"[{name}] must be a TOML table")
    return value


def _pair(value: str) -> tuple[str, int]:
    host, separator, port = value.rpartition(":")
    if not separator or not host:
        raise ValueError(f"expected HOST:PORT, received {value!r}")
    try:
        parsed_port = int(port)
    except ValueError as error:
        raise ValueError(f"invalid port in endpoint {value!r}") from error
    if not 1 <= parsed_port <= 65_535:
        raise ValueError(f"port must be between 1 and 65535 in endpoint {value!r}")
    return host, parsed_port


def _coordinate(value: Any, name: str, minimum: float, maximum: float) -> float:
    coordinate = float(value)
    if not math.isfinite(coordinate) or not minimum <= coordinate <= maximum:
        raise ValueError(f"frame.{name} must be between {minimum} and {maximum}")
    return coordinate


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be true or false")
    return value


def build_service(config_path: str | Path, *, enable_control: bool = False) -> CompanionService:
    with Path(config_path).open("rb") as source:
        document = tomllib.load(source)
    node = _section(document, "node")
    frame = _section(document, "frame")
    px4 = _section(document, "px4")
    radio_config = _section(document, "radio")
    safety = _section(document, "safety")
    boot_id = int(node.get("boot_id", 0)) or random.SystemRandom().randrange(1, 0x10000)
    config = CompanionConfig(
        node_id=int(node["id"]),
        boot_id=boot_id,
        mission_epoch_unix_s=float(frame.get("mission_epoch_unix_s", 0.0)),
        telemetry_interval_s=float(safety.get("telemetry_interval_s", 1.0)),
        risk_interval_s=float(safety.get("risk_interval_s", 0.1)),
        command_interval_s=float(safety.get("command_interval_s", 0.1)),
        own_state_timeout_s=float(safety.get("own_state_timeout_s", 0.5)),
        neighbor_expiry_s=float(safety.get("neighbor_expiry_s", 6.0)),
        safety_distance_m=float(safety.get("safety_distance_m", 10.0)),
        horizon_s=float(safety.get("horizon_s", 5.0)),
        uncertainty_sigma_multiplier=float(
            safety.get("uncertainty_sigma_multiplier", 2.5)
        ),
        own_position_sigma_m=float(safety.get("own_position_sigma_m", 2.0)),
        own_velocity_sigma_mps=float(safety.get("own_velocity_sigma_mps", 0.15)),
        measurement_position_sigma_m=float(
            safety.get("measurement_position_sigma_m", 2.0)
        ),
        measurement_velocity_sigma_mps=float(
            safety.get("measurement_velocity_sigma_mps", 0.15)
        ),
        clock_sigma_s=float(safety.get("clock_sigma_s", 0.05)),
        maneuver_duration_s=float(safety.get("maneuver_duration_s", 2.0)),
        maneuver_speed_mps=float(safety.get("maneuver_speed_mps", 3.0)),
        vertical_speed_mps=float(safety.get("vertical_speed_mps", 1.5)),
        min_down_m=float(safety.get("min_down_m", -20.0)),
        max_down_m=float(safety.get("max_down_m", 20.0)),
        packet_ttl=int(safety.get("packet_ttl", 5)),
        table_capacity=int(safety.get("table_capacity", 16)),
        control_enabled=enable_control or _boolean(
            safety.get("control_enabled", False), "safety.control_enabled"
        ),
    )
    reference_latitude = _coordinate(
        frame["reference_latitude_deg"], "reference_latitude_deg", -90.0, 90.0
    )
    reference_longitude = _coordinate(
        frame["reference_longitude_deg"], "reference_longitude_deg", -180.0, 180.0
    )
    reference_altitude = float(frame["reference_altitude_m"])
    if not math.isfinite(reference_altitude):
        raise ValueError("frame.reference_altitude_m must be finite")
    flight_controller = MavlinkPx4(
        str(px4.get("connection", "/dev/serial0")),
        int(px4.get("baud", 57_600)),
        node_id=config.node_id,
        reference_latitude_deg=reference_latitude,
        reference_longitude_deg=reference_longitude,
        reference_altitude_m=reference_altitude,
        heartbeat_timeout_s=float(px4.get("heartbeat_timeout_s", 10.0)),
        minimum_gps_fix_type=int(px4.get("minimum_gps_fix_type", 3)),
    )
    try:
        transport = str(radio_config.get("transport", "serial"))
        if transport == "serial":
            radio = SerialFrameRadio(
                str(radio_config.get("device", "/dev/ttyUSB0")),
                int(radio_config.get("baud", 115_200)),
            )
        elif transport == "udp":
            radio = UdpRadio(
                _pair(str(radio_config.get("bind", "0.0.0.0:14600"))),
                _pair(str(radio_config.get("peer", "127.0.0.1:14601"))),
            )
        else:
            raise ValueError("radio.transport must be 'serial' or 'udp'")
    except Exception:
        flight_controller.close()
        raise
    return CompanionService(config, flight_controller, radio)


def run_service(config_path: str | Path, *, enable_control: bool = False) -> int:
    service = build_service(config_path, enable_control=enable_control)
    notifier = SystemdNotifier()
    running = True

    def stop(_signal: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    # A configured epoch supports multi-day campaigns. Zero selects UTC midnight
    # for convenient single-day tests. Every aircraft must use the same value.
    epoch = service.config.mission_epoch_unix_s
    if epoch == 0.0:
        epoch = (int(time.time()) // 86_400) * 86_400
    notifier.send("READY=1")
    next_status = 0.0
    try:
        while running:
            monotonic_s = time.monotonic()
            mission_time_s = time.time() - epoch
            if not 0.0 <= mission_time_s <= 0xFFFFFFFF / 1000.0:
                raise RuntimeError(
                    "mission time is outside the protocol's uint32 millisecond range; "
                    "choose a newer shared mission_epoch_unix_s"
                )
            service.step(monotonic_s, mission_time_s)
            notifier.send("WATCHDOG=1")
            if monotonic_s >= next_status:
                print(json.dumps(service.snapshot(), sort_keys=True), flush=True)
                next_status = monotonic_s + 5.0
            time.sleep(0.02)
    finally:
        notifier.send("STOPPING=1")
        service.close()
    return 0
