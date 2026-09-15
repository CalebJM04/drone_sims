from __future__ import annotations

import os
import socket
import struct

from .collision import KinematicState, Vec3
from .companion import global_to_local_ned
from .mavlink_codec import MavlinkUnavailable, VelocityCommand
from .protocol import FRAME_SIZE, MAGIC, ProtocolError, decode
from .radio_types import RadioReception


BRIDGE_TX_MAGIC = 0xA5
BRIDGE_RX_MAGIC = 0xA6
BRIDGE_ENVELOPE_VERSION = 1
_BRIDGE_TX_HEADER = struct.Struct("!BBH")
_BRIDGE_RX_HEADER = struct.Struct("!BBHhh")
BRIDGE_TX_SIZE = _BRIDGE_TX_HEADER.size + FRAME_SIZE
BRIDGE_RX_SIZE = _BRIDGE_RX_HEADER.size + FRAME_SIZE


def encode_bridge_transmit(node_id: int, payload: bytes) -> bytes:
    if not 1 <= node_id <= 0xFFFF:
        raise ValueError("bridge node_id must be between 1 and 65535")
    if len(payload) != FRAME_SIZE:
        raise ValueError(f"radio payload must be exactly {FRAME_SIZE} bytes")
    return _BRIDGE_TX_HEADER.pack(BRIDGE_TX_MAGIC, BRIDGE_ENVELOPE_VERSION, node_id) + payload


def decode_bridge_reception(data: bytes) -> RadioReception:
    if len(data) != BRIDGE_RX_SIZE:
        raise ValueError(f"bridge reception must be {BRIDGE_RX_SIZE} bytes")
    magic, version, sender_id, rssi_tenths, snr_tenths = _BRIDGE_RX_HEADER.unpack(
        data[: _BRIDGE_RX_HEADER.size]
    )
    if magic != BRIDGE_RX_MAGIC or version != BRIDGE_ENVELOPE_VERSION:
        raise ValueError("unsupported bridge reception envelope")
    if sender_id == 0:
        raise ValueError("bridge reception sender_id must be non-zero")
    return RadioReception(
        data[_BRIDGE_RX_HEADER.size :],
        sender_id=sender_id,
        rssi_dbm=rssi_tenths / 10.0,
        snr_db=snr_tenths / 10.0,
    )


class SerialFrameRadio:
    def __init__(self, device: str, baud: int = 115_200, *, node_id: int = 1) -> None:
        try:
            import serial
        except ImportError as error:
            raise RuntimeError("install the 'hardware' extra to use a serial LoRa modem") from error
        self.serial = serial.Serial(device, baudrate=baud, timeout=0)
        self._buffer = bytearray()
        self.node_id = node_id

    def receive(self) -> list[bytes | RadioReception]:
        waiting = self.serial.in_waiting
        if waiting:
            self._buffer.extend(self.serial.read(waiting))
        frames: list[bytes | RadioReception] = []
        while self._buffer:
            starts = [
                index
                for marker in (MAGIC, BRIDGE_RX_MAGIC)
                if (index := self._buffer.find(bytes([marker]))) >= 0
            ]
            if not starts:
                self._buffer.clear()
                break
            start = min(starts)
            if start:
                del self._buffer[:start]
            if self._buffer[0] == BRIDGE_RX_MAGIC:
                if len(self._buffer) < BRIDGE_RX_SIZE:
                    break
                candidate = bytes(self._buffer[:BRIDGE_RX_SIZE])
                try:
                    reception = decode_bridge_reception(candidate)
                    decode(reception.payload)
                except (ProtocolError, ValueError):
                    del self._buffer[0]
                    continue
                frames.append(reception)
                del self._buffer[:BRIDGE_RX_SIZE]
                continue
            if len(self._buffer) < FRAME_SIZE:
                break
            candidate = bytes(self._buffer[:FRAME_SIZE])
            frames.append(candidate)
            try:
                decode(candidate)
            except ProtocolError:
                del self._buffer[0]
            else:
                del self._buffer[:FRAME_SIZE]
        return frames

    def send(self, payload: bytes) -> None:
        envelope = encode_bridge_transmit(self.node_id, payload)
        written = self.serial.write(envelope)
        if written != len(envelope):
            raise OSError(f"short radio write: {written}/{len(envelope)} bytes")

    def close(self) -> None:
        self.serial.close()


class UdpRadio:
    def __init__(self, bind: tuple[str, int], peer: tuple[str, int]) -> None:
        self.peer = peer
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.bind(bind)
        self.socket.setblocking(False)

    def receive(self) -> list[bytes]:
        packets: list[bytes] = []
        while True:
            try:
                payload, _ = self.socket.recvfrom(65_535)
            except BlockingIOError:
                return packets
            packets.append(payload)

    def send(self, payload: bytes) -> None:
        self.socket.sendto(payload, self.peer)

    def close(self) -> None:
        self.socket.close()


class MavlinkPx4:
    def __init__(
        self,
        connection: str,
        baud: int,
        *,
        node_id: int,
        reference_latitude_deg: float,
        reference_longitude_deg: float,
        reference_altitude_m: float,
        heartbeat_timeout_s: float = 10.0,
        minimum_gps_fix_type: int = 3,
    ) -> None:
        if not 0 <= minimum_gps_fix_type <= 8:
            raise ValueError("minimum_gps_fix_type must be between 0 and 8")
        try:
            from pymavlink import mavutil
        except ImportError as error:
            raise MavlinkUnavailable("install the 'hardware' extra to connect to PX4") from error
        self._mavutil = mavutil
        self.connection = mavutil.mavlink_connection(
            connection,
            baud=baud,
            source_system=node_id,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
            autoreconnect=True,
        )
        heartbeat = self.connection.wait_heartbeat(timeout=heartbeat_timeout_s)
        if heartbeat is None:
            raise TimeoutError(f"no PX4 heartbeat received from {connection}")
        self.node_id = node_id
        self.reference = (
            reference_latitude_deg,
            reference_longitude_deg,
            reference_altitude_m,
        )
        self.minimum_gps_fix_type = minimum_gps_fix_type
        self._gps_fix_type = 0
        self._gps_healthy = False
        self._request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10.0
        )
        self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5.0)
        self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1.0)

    def _request_message_interval(self, message_id: int, rate_hz: float) -> None:
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            self._mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            round(1_000_000 / rate_hz),
            0,
            0,
            0,
            0,
            0,
        )

    def receive_state(self, timestamp_s: float) -> KinematicState | None:
        latest = None
        while True:
            message = self.connection.recv_match(
                type=["GLOBAL_POSITION_INT", "GPS_RAW_INT", "SYS_STATUS"],
                blocking=False,
            )
            if message is None:
                break
            message_type = message.get_type()
            if message_type == "GLOBAL_POSITION_INT":
                latest = message
            elif message_type == "GPS_RAW_INT":
                self._gps_fix_type = int(message.fix_type)
            elif message_type == "SYS_STATUS":
                gps_bit = self._mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS
                self._gps_healthy = bool(
                    message.onboard_control_sensors_present & gps_bit
                    and message.onboard_control_sensors_enabled & gps_bit
                    and message.onboard_control_sensors_health & gps_bit
                )
        if (
            latest is None
            or not self._gps_healthy
            or self._gps_fix_type < self.minimum_gps_fix_type
        ):
            return None
        latitude = latest.lat / 1e7
        longitude = latest.lon / 1e7
        altitude = latest.alt / 1000.0
        position = global_to_local_ned(
            latitude,
            longitude,
            altitude,
            reference_latitude_deg=self.reference[0],
            reference_longitude_deg=self.reference[1],
            reference_altitude_m=self.reference[2],
        )
        return KinematicState(
            self.node_id,
            position,
            Vec3(latest.vx / 100.0, latest.vy / 100.0, latest.vz / 100.0),
            timestamp_s,
        )

    def send_velocity(self, command: VelocityCommand, time_boot_ms: int) -> None:
        mavlink = self._mavutil.mavlink
        type_mask = 0b0000110111000111
        self.connection.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self.connection.target_system,
            self.connection.target_component,
            mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0, 0.0, 0.0,
            command.north_mps, command.east_mps, command.down_mps,
            0.0, 0.0, 0.0,
            0.0, 0.0,
        )

    def health_snapshot(self) -> dict[str, object]:
        return {
            "gps_healthy": self._gps_healthy,
            "gps_fix_type": self._gps_fix_type,
            "minimum_gps_fix_type": self.minimum_gps_fix_type,
            "target_system": self.connection.target_system,
            "target_component": self.connection.target_component,
        }

    def close(self) -> None:
        self.connection.close()


class SystemdNotifier:
    def __init__(self) -> None:
        self.address = os.environ.get("NOTIFY_SOCKET")

    def send(self, message: str) -> None:
        if not self.address:
            return
        address = self.address
        if address.startswith("@"):
            address = "\0" + address[1:]
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as notifier:
            notifier.connect(address)
            notifier.sendall(message.encode("utf-8"))
