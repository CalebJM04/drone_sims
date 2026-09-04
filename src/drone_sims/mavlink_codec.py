from __future__ import annotations

from dataclasses import dataclass


class MavlinkUnavailable(RuntimeError):
    pass


def _common():
    try:
        from pymavlink.dialects.v20 import common
    except ImportError as error:
        raise MavlinkUnavailable("install the optional 'sitl' dependency to enable MAVLink") from error
    return common


@dataclass(frozen=True, slots=True)
class VelocityCommand:
    north_mps: float
    east_mps: float
    down_mps: float
    target_system: int = 1
    target_component: int = 1


def encode_velocity_command(command: VelocityCommand, time_boot_ms: int, source_system: int = 240) -> bytes:
    common = _common()
    mavlink = common.MAVLink(None, srcSystem=source_system, srcComponent=1)
    ignore_position_acceleration_yaw = 0b0000110111000111
    message = common.MAVLink_set_position_target_local_ned_message(
        time_boot_ms,
        command.target_system,
        command.target_component,
        common.MAV_FRAME_LOCAL_NED,
        ignore_position_acceleration_yaw,
        0.0, 0.0, 0.0,
        command.north_mps, command.east_mps, command.down_mps,
        0.0, 0.0, 0.0,
        0.0, 0.0,
    )
    return message.pack(mavlink)


def encode_global_position(
    *,
    time_boot_ms: int,
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    relative_altitude_m: float,
    north_mps: float,
    east_mps: float,
    down_mps: float,
    heading_deg: float,
    source_system: int,
) -> bytes:
    common = _common()
    mavlink = common.MAVLink(None, srcSystem=source_system, srcComponent=1)
    message = common.MAVLink_global_position_int_message(
        time_boot_ms,
        round(latitude_deg * 1e7),
        round(longitude_deg * 1e7),
        round(altitude_m * 1000),
        round(relative_altitude_m * 1000),
        round(north_mps * 100),
        round(east_mps * 100),
        round(down_mps * 100),
        round(heading_deg * 100) % 36000,
    )
    return message.pack(mavlink)


def decode_message(data: bytes):
    common = _common()
    parser = common.MAVLink(None)
    message = None
    for byte in data:
        parsed = parser.parse_char(bytes((byte,)))
        if parsed is not None:
            message = parsed
    if message is None:
        raise ValueError("incomplete MAVLink message")
    return message
