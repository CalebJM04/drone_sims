from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

from drone_sims.collision import KinematicState, Vec3
from drone_sims.companion import CompanionConfig, CompanionService, global_to_local_ned
from drone_sims.companion_io import (
    BRIDGE_RX_MAGIC,
    MavlinkPx4,
    SerialFrameRadio,
    decode_bridge_reception,
    encode_bridge_transmit,
)
from drone_sims.radio_types import RadioReception
from drone_sims.companion_runtime import _boolean, _coordinate, _pair
from drone_sims.protocol import FRAME_SIZE, ProtocolError, TelemetryFrame, decode, encode


class FakeRadio:
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


class FakePx4:
    def __init__(self, states: list[KinematicState | None]) -> None:
        self.states = states
        self.commands = []

    def receive_state(self, timestamp_s: float) -> KinematicState | None:
        return self.states.pop(0) if self.states else None

    def send_velocity(self, command, time_boot_ms: int) -> None:
        self.commands.append((command, time_boot_ms))

    def health_snapshot(self) -> dict[str, object]:
        return {"fake": True}

    def close(self) -> None:
        pass


class FakeSerial:
    def __init__(self, payload: bytes) -> None:
        self.payload = bytearray(payload)

    @property
    def in_waiting(self) -> int:
        return len(self.payload)

    def read(self, length: int) -> bytes:
        result = bytes(self.payload[:length])
        del self.payload[:length]
        return result

    def write(self, payload: bytes) -> int:
        self.written = payload
        return len(payload)


class FakeMavlinkConnection:
    target_system = 1
    target_component = 1

    def __init__(self, messages: list[object]) -> None:
        self.messages = messages

    def recv_match(self, *, type, blocking: bool):
        del type, blocking
        return self.messages.pop(0) if self.messages else None


def state(node: int, x: float, vx: float, timestamp: float = 0.0) -> KinematicState:
    return KinematicState(node, Vec3(x, 0.0, 0.0), Vec3(vx, 0.0, 0.0), timestamp)


class CompanionServiceTests(unittest.TestCase):
    def test_mission_epoch_validation(self) -> None:
        with self.assertRaisesRegex(ValueError, "mission_epoch"):
            CompanionConfig(1, 10, mission_epoch_unix_s=-1.0)

    def test_non_finite_configuration_values_are_rejected(self) -> None:
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "finite"):
                    CompanionConfig(1, 10, safety_distance_m=value)
        with self.assertRaisesRegex(ValueError, "finite"):
            CompanionConfig(1, 10, min_down_m=-math.inf)

    def test_coordinate_and_udp_endpoint_validation(self) -> None:
        self.assertEqual(_pair("127.0.0.1:14600"), ("127.0.0.1", 14600))
        self.assertEqual(_coordinate(35.0, "latitude", -90.0, 90.0), 35.0)
        for endpoint in (
            "127.0.0.1",
            "127.0.0.1:nope",
            "127.0.0.1:0",
            "127.0.0.1:65536",
        ):
            with self.subTest(endpoint=endpoint):
                with self.assertRaises(ValueError):
                    _pair(endpoint)
        for value in (math.nan, 91.0):
            with self.subTest(coordinate=value):
                with self.assertRaises(ValueError):
                    _coordinate(value, "latitude", -90.0, 90.0)
        self.assertFalse(_boolean(False, "control"))
        with self.assertRaises(ValueError):
            _boolean("false", "control")

    def test_shared_global_origin_conversion(self) -> None:
        origin = dict(
            reference_latitude_deg=35.0,
            reference_longitude_deg=-84.0,
            reference_altitude_m=300.0,
        )
        point = global_to_local_ned(35.001, -83.999, 315.0, **origin)
        self.assertAlmostEqual(point.x, 111.32, delta=0.1)
        self.assertAlmostEqual(point.y, 91.19, delta=0.2)
        self.assertEqual(point.z, -15.0)

    def test_broadcast_receive_forward_and_duplicate_suppression(self) -> None:
        px4 = FakePx4([state(1, 0.0, 1.0)])
        radio = FakeRadio()
        service = CompanionService(CompanionConfig(1, 10), px4, radio)
        peer = TelemetryFrame.from_state(state(2, 8.0, -1.0), sequence=4, boot_id=20)
        radio.incoming.extend([encode(peer), encode(peer)])
        service.step(0.0, 0.0)
        self.assertEqual(service.stats.telemetry_sent, 1)
        self.assertEqual(service.stats.received, 1)
        self.assertEqual(service.stats.forwarded, 1)
        self.assertEqual(service.stats.duplicates, 1)
        self.assertEqual(len(radio.outgoing), 2)

    def test_risk_generates_streamed_avoidance_command(self) -> None:
        px4 = FakePx4([state(1, 0.0, 1.0), state(1, 0.1, 1.0)])
        radio = FakeRadio()
        config = CompanionConfig(1, 10, control_enabled=True)
        service = CompanionService(config, px4, radio)
        radio.incoming.append(encode(
            TelemetryFrame.from_state(state(2, 8.0, -1.0), sequence=1, boot_id=20)
        ))
        service.step(0.0, 0.0)
        self.assertIsNotNone(service.active_command)
        self.assertEqual(len(px4.commands), 1)
        command = px4.commands[0][0]
        self.assertNotEqual((command.north_mps, command.east_mps), (1.0, 0.0))
        service.step(0.1, 0.1)
        self.assertEqual(len(px4.commands), 2)

    def test_future_peer_does_not_distort_valid_avoidance_plan(self) -> None:
        def planned_command(extra_frames=()):
            px4 = FakePx4([state(1, 0.0, 1.0)])
            radio = FakeRadio()
            service = CompanionService(
                CompanionConfig(1, 10, control_enabled=True), px4, radio
            )
            valid = TelemetryFrame.from_state(
                state(2, 8.0, -1.0), sequence=1, boot_id=20
            )
            radio.incoming.extend(
                [encode(valid), *(encode(frame) for frame in extra_frames)]
            )
            service.step(0.0, 0.0)
            return px4.commands[0][0]

        baseline = planned_command()
        future = TelemetryFrame.from_state(
            KinematicState(3, Vec3(0.0, 1.0, 0.0), Vec3(0.0, 0.0, 0.0), 10.0),
            sequence=1,
            boot_id=30,
        )
        with_future = planned_command((future,))
        self.assertEqual(with_future, baseline)

    def test_stale_px4_state_suppresses_commands(self) -> None:
        px4 = FakePx4([state(1, 0.0, 1.0), None])
        radio = FakeRadio()
        service = CompanionService(
            CompanionConfig(1, 10, control_enabled=True, own_state_timeout_s=0.2),
            px4,
            radio,
        )
        service.step(0.0, 0.0)
        self.assertEqual(len(px4.commands), 1)
        self.assertEqual(px4.commands[0][0].north_mps, 0.0)
        service.step(0.3, 0.3)
        self.assertEqual(len(px4.commands), 1)
        self.assertEqual(service.stats.stale_own_state_events, 1)
        self.assertFalse(service.snapshot()["own_state_fresh"])

    def test_initial_px4_wait_is_not_counted_as_state_loss(self) -> None:
        px4 = FakePx4([None])
        service = CompanionService(CompanionConfig(1, 10), px4, FakeRadio())
        service.step(0.0, 0.0)
        self.assertEqual(service.stats.stale_own_state_events, 0)
        self.assertFalse(service.snapshot()["own_state_fresh"])

    def test_corrupt_packet_never_updates_state(self) -> None:
        px4 = FakePx4([state(1, 0.0, 0.0)])
        radio = FakeRadio()
        service = CompanionService(CompanionConfig(1, 10), px4, radio)
        packet = bytearray(encode(
            TelemetryFrame.from_state(state(2, 10.0, 0.0), sequence=1, boot_id=20)
        ))
        packet[-1] ^= 1
        radio.incoming.append(bytes(packet))
        service.step(0.0, 0.0)
        self.assertEqual(service.stats.crc_rejections, 1)
        self.assertFalse(service.table.entries)

    def test_companion_exposes_link_topology_and_collision_alerts(self) -> None:
        px4 = FakePx4([state(1, 0.0, 1.0)])
        radio = FakeRadio()
        service = CompanionService(
            CompanionConfig(1, 10, routing_mode="proactive", nominal_radio_range_m=30),
            px4,
            radio,
        )
        payload = encode(
            TelemetryFrame.from_state(state(2, 8.0, -1.0), sequence=1, boot_id=20)
        )
        radio.incoming.append(
            RadioReception(payload, sender_id=2, rssi_dbm=-88.5, snr_db=7.0)
        )
        service.step(0.0, 0.0)
        snapshot = service.snapshot()
        self.assertEqual(snapshot["routing_mode"], "proactive")
        self.assertEqual(snapshot["link_quality"][0]["rssi_dbm"], -88.5)
        self.assertTrue(snapshot["topology"]["links"])
        self.assertTrue(snapshot["collision_alerts"][0]["risk"])
        self.assertEqual(service.stats.commands_sent, 0)

    def test_serial_radio_resynchronizes_after_corrupt_frame(self) -> None:
        valid = encode(
            TelemetryFrame.from_state(state(2, 10.0, 0.0), sequence=3, boot_id=20)
        )
        corrupt = bytearray(valid)
        corrupt[FRAME_SIZE - 1] ^= 1
        radio = object.__new__(SerialFrameRadio)
        radio.serial = FakeSerial(b"garbage" + bytes(corrupt) + valid)
        radio._buffer = bytearray()
        radio.node_id = 1
        packets = radio.receive()
        decoded = []
        for packet in packets:
            try:
                decoded.append(decode(packet))
            except ProtocolError:
                pass
        self.assertEqual([frame.packet_id for frame in decoded], [(2, 20, 3)])

    def test_metadata_bridge_envelope_round_trip(self) -> None:
        payload = encode(
            TelemetryFrame.from_state(state(2, 10.0, 0.0), sequence=3, boot_id=20)
        )
        outbound = encode_bridge_transmit(7, payload)
        self.assertEqual(outbound[:4], bytes((0xA5, 1, 0, 7)))
        inbound = bytes((BRIDGE_RX_MAGIC, 1, 0, 7, 0xFC, 0x72, 0, 35)) + payload
        reception = decode_bridge_reception(inbound)
        self.assertEqual(reception.sender_id, 7)
        self.assertEqual(reception.rssi_dbm, -91.0)
        self.assertEqual(reception.snr_db, 3.5)
        self.assertEqual(reception.payload, payload)

        radio = object.__new__(SerialFrameRadio)
        radio.serial = FakeSerial(b"noise" + inbound)
        radio._buffer = bytearray()
        radio.node_id = 9
        received = radio.receive()
        self.assertEqual(received, [reception])
        radio.send(payload)
        self.assertEqual(radio.serial.written, encode_bridge_transmit(9, payload))

    def test_px4_adapter_requires_healthy_minimum_gps_fix(self) -> None:
        def message(message_type: str, **values):
            return SimpleNamespace(get_type=lambda: message_type, **values)

        px4 = object.__new__(MavlinkPx4)
        px4._mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(MAV_SYS_STATUS_SENSOR_GPS=32)
        )
        px4.node_id = 1
        px4.reference = (35.0, -84.0, 300.0)
        px4.minimum_gps_fix_type = 6
        px4._gps_fix_type = 0
        px4._gps_healthy = False
        healthy = message(
            "SYS_STATUS",
            onboard_control_sensors_present=32,
            onboard_control_sensors_enabled=32,
            onboard_control_sensors_health=32,
        )
        fixed = message("GPS_RAW_INT", fix_type=6)
        position = message(
            "GLOBAL_POSITION_INT",
            lat=350_000_000,
            lon=-840_000_000,
            alt=300_000,
            vx=100,
            vy=0,
            vz=0,
        )
        px4.connection = FakeMavlinkConnection([healthy, fixed, position])
        state_value = px4.receive_state(2.0)
        self.assertIsNotNone(state_value)
        assert state_value is not None
        self.assertEqual(state_value.velocity, Vec3(1.0, 0.0, 0.0))

        px4.connection = FakeMavlinkConnection([
            message("GPS_RAW_INT", fix_type=5), position
        ])
        self.assertIsNone(px4.receive_state(2.1))


if __name__ == "__main__":
    unittest.main()
