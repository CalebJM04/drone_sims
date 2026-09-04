from __future__ import annotations

from dataclasses import dataclass, replace
import struct

from .collision import KinematicState, Vec3


MAGIC = 0xD7
VERSION = 2
TYPE_TELEMETRY = 1
FLAG_EMERGENCY = 0x01
_BODY = struct.Struct("!BBBBBBHHHIiiihhh")
_CRC = struct.Struct("!H")
FRAME_SIZE = _BODY.size + _CRC.size


class ProtocolError(ValueError):
    pass


class CrcError(ProtocolError):
    pass


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True, slots=True)
class TelemetryFrame:
    source: int
    sequence: int
    boot_id: int
    timestamp_ms: int
    position_cm: tuple[int, int, int]
    velocity_cms: tuple[int, int, int]
    ttl: int = 5
    hops: int = 0
    flags: int = 0

    @property
    def packet_id(self) -> tuple[int, int, int]:
        return self.source, self.boot_id, self.sequence

    def forwarded(self) -> "TelemetryFrame":
        if self.ttl <= 0:
            raise ProtocolError("cannot forward a frame with exhausted TTL")
        return replace(self, ttl=self.ttl - 1, hops=self.hops + 1)

    def to_state(self) -> KinematicState:
        return KinematicState(
            self.source,
            Vec3(*(component / 100.0 for component in self.position_cm)),
            Vec3(*(component / 100.0 for component in self.velocity_cms)),
            self.timestamp_ms / 1000.0,
        )

    @classmethod
    def from_state(
        cls, state: KinematicState, *, sequence: int, boot_id: int = 0,
        ttl: int = 5, flags: int = 0
    ) -> "TelemetryFrame":
        return cls(
            source=state.node_id,
            sequence=sequence & 0xFFFF,
            boot_id=boot_id & 0xFFFF,
            timestamp_ms=max(0, round(state.timestamp * 1000.0)),
            position_cm=tuple(round(value * 100.0) for value in state.position.as_list()),
            velocity_cms=tuple(round(value * 100.0) for value in state.velocity.as_list()),
            ttl=ttl,
            flags=flags,
        )


def encode(frame: TelemetryFrame) -> bytes:
    try:
        body = _BODY.pack(
            MAGIC,
            VERSION,
            TYPE_TELEMETRY,
            frame.flags,
            frame.ttl,
            frame.hops,
            frame.source,
            frame.sequence,
            frame.boot_id,
            frame.timestamp_ms,
            *frame.position_cm,
            *frame.velocity_cms,
        )
    except struct.error as error:
        raise ProtocolError(f"frame field outside encoded numeric range: {error}") from error
    return body + _CRC.pack(crc16_ccitt(body))


def decode(data: bytes) -> TelemetryFrame:
    if len(data) != FRAME_SIZE:
        raise ProtocolError(f"expected {FRAME_SIZE} bytes, received {len(data)}")
    body, received_crc = data[:-2], _CRC.unpack(data[-2:])[0]
    if crc16_ccitt(body) != received_crc:
        raise CrcError("CRC mismatch")
    fields = _BODY.unpack(body)
    magic, version, message_type = fields[:3]
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if version != VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    if message_type != TYPE_TELEMETRY:
        raise ProtocolError(f"unsupported message type {message_type}")
    _, _, _, flags, ttl, hops, source, sequence, boot_id, timestamp_ms, *vectors = fields
    return TelemetryFrame(
        source,
        sequence,
        boot_id,
        timestamp_ms,
        tuple(vectors[:3]),
        tuple(vectors[3:]),
        ttl,
        hops,
        flags,
    )


def sequence_is_newer(candidate: int, previous: int) -> bool:
    difference = (candidate - previous) & 0xFFFF
    return 0 < difference < 0x8000
