from __future__ import annotations

import math
import unittest

from drone_sims.collision import KinematicState, Vec3
from drone_sims.companion import CompanionConfig, CompanionService, global_to_local_ned
from drone_sims.protocol import TelemetryFrame, decode, encode


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


if __name__ == "__main__":
    unittest.main()
