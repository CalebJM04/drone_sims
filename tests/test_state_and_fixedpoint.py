from __future__ import annotations

import random
import unittest

from drone_sims.collision import KinematicState, Vec3, assess
from drone_sims.fixedpoint import fixed_point_assess
from drone_sims.protocol import TelemetryFrame
from drone_sims.state_table import NeighborTable


def frame(source: int, sequence: int, timestamp: float = 0.0) -> TelemetryFrame:
    return TelemetryFrame.from_state(KinematicState(source, Vec3(source, 0), Vec3(0, 0), timestamp), sequence=sequence)


class StateTableTests(unittest.TestCase):
    def test_reordering_and_rollover(self) -> None:
        table = NeighborTable()
        self.assertTrue(table.update(frame(1, 65535), 1.0))
        self.assertTrue(table.update(frame(1, 0), 2.0))
        self.assertFalse(table.update(frame(1, 65534), 3.0))
        self.assertEqual(table.entries[1].sequence, 0)

    def test_capacity_evicts_oldest(self) -> None:
        table = NeighborTable(capacity=2)
        table.update(frame(1, 1), 1.0)
        table.update(frame(2, 1), 2.0)
        table.update(frame(3, 1), 3.0)
        self.assertNotIn(1, table.entries)
        self.assertEqual(table.evictions, 1)

    def test_expiration(self) -> None:
        table = NeighborTable()
        table.update(frame(1, 1), 1.0)
        table.update(frame(2, 1), 3.0)
        self.assertEqual(table.expire(4.0, 2.0), [1])

    def test_reboot_session_accepts_sequence_reset_and_rejects_delayed_old_session(self) -> None:
        table = NeighborTable()
        old = TelemetryFrame.from_state(KinematicState(1, Vec3(1, 0), Vec3(0, 0), 10), sequence=500, boot_id=10)
        rebooted = TelemetryFrame.from_state(KinematicState(1, Vec3(2, 0), Vec3(0, 0), 0), sequence=0, boot_id=11)
        delayed = TelemetryFrame.from_state(KinematicState(1, Vec3(99, 0), Vec3(0, 0), 11), sequence=501, boot_id=10)
        self.assertTrue(table.update(old, 10))
        self.assertTrue(table.update(rebooted, 11))
        self.assertFalse(table.update(delayed, 12))
        self.assertEqual(table.entries[1].boot_id, 11)
        self.assertEqual(table.entries[1].state.position.x, 2)

    def test_replay_metadata_survives_neighbor_expiry(self) -> None:
        table = NeighborTable()
        first = TelemetryFrame.from_state(
            KinematicState(1, Vec3(1, 0), Vec3(0, 0), 10),
            sequence=500,
            boot_id=10,
        )
        newer = TelemetryFrame.from_state(
            KinematicState(1, Vec3(2, 0), Vec3(0, 0), 11),
            sequence=501,
            boot_id=10,
        )
        replay = TelemetryFrame.from_state(
            KinematicState(1, Vec3(99, 0), Vec3(0, 0), 10),
            sequence=500,
            boot_id=10,
        )
        self.assertTrue(table.update(first, 10))
        self.assertEqual(table.expire(20, 5), [1])
        self.assertFalse(table.update(replay, 21))
        self.assertTrue(table.update(newer, 22))
        self.assertEqual(table.entries[1].state.position.x, 2)

    def test_timestamp_allows_recovery_after_large_sequence_gap(self) -> None:
        table = NeighborTable()
        first = TelemetryFrame.from_state(
            KinematicState(1, Vec3(1, 0), Vec3(0, 0), 1),
            sequence=10,
            boot_id=10,
        )
        after_long_outage = TelemetryFrame.from_state(
            KinematicState(1, Vec3(2, 0), Vec3(0, 0), 40_000),
            sequence=40_010,
            boot_id=10,
        )
        self.assertTrue(table.update(first, 1))
        self.assertEqual(table.expire(10, 5), [1])
        self.assertTrue(table.update(after_long_outage, 40_000))


class FixedPointTests(unittest.TestCase):
    def test_known_case(self) -> None:
        own = KinematicState(1, Vec3(0, 0), Vec3(0, 0), 0)
        peer = KinematicState(2, Vec3(10, 0), Vec3(-2, 0), 0)
        result = fixed_point_assess(own, peer, safety_distance_cm=300, horizon_ms=6000)
        self.assertTrue(result.risk)
        self.assertEqual(result.tcpa_ms, 5000)
        self.assertEqual(result.dcpa_cm, 0)

    def test_random_fixed_point_matches_float_away_from_boundaries(self) -> None:
        rng = random.Random(619)
        compared = 0
        for _ in range(5000):
            own = KinematicState(1, Vec3(*(rng.uniform(-100, 100) for _ in range(3))), Vec3(*(rng.uniform(-12, 12) for _ in range(3))), 0)
            peer = KinematicState(2, Vec3(*(rng.uniform(-100, 100) for _ in range(3))), Vec3(*(rng.uniform(-12, 12) for _ in range(3))), 0)
            floating = assess(own, peer, now=0, safety_distance=3, horizon=8, max_age=1)
            fixed = fixed_point_assess(own, peer, safety_distance_cm=300, horizon_ms=8000)
            near_distance = abs(floating.dcpa - 3.0) < 0.08
            near_horizon = floating.tcpa is not None and abs(floating.tcpa - 8.0) < 0.03
            if near_distance or near_horizon:
                continue
            self.assertEqual(fixed.risk, floating.risk)
            compared += 1
        self.assertGreater(compared, 4500)


if __name__ == "__main__":
    unittest.main()
