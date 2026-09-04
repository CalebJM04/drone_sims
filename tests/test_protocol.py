from __future__ import annotations

import random
import unittest

from drone_sims.collision import KinematicState, Vec3
from drone_sims.protocol import CrcError, FRAME_SIZE, ProtocolError, TelemetryFrame, decode, encode, sequence_is_newer


class ProtocolTests(unittest.TestCase):
    def setUp(self) -> None:
        self.frame = TelemetryFrame.from_state(
            KinematicState(42, Vec3(12.34, -5.67, 8.9), Vec3(-1.2, 3.4, 0.0), 123.456),
            sequence=65535, boot_id=0x1234,
        )

    def test_round_trip(self) -> None:
        encoded = encode(self.frame)
        self.assertEqual(len(encoded), FRAME_SIZE)
        self.assertEqual(decode(encoded), self.frame)

    def test_forwarding_updates_ttl_and_hops_only(self) -> None:
        forwarded = self.frame.forwarded()
        self.assertEqual(forwarded.packet_id, self.frame.packet_id)
        self.assertEqual(forwarded.ttl, self.frame.ttl - 1)
        self.assertEqual(forwarded.hops, self.frame.hops + 1)

    def test_every_single_bit_corruption_is_detected(self) -> None:
        original = encode(self.frame)
        for bit in range(len(original) * 8):
            corrupt = bytearray(original)
            corrupt[bit // 8] ^= 1 << (bit % 8)
            with self.assertRaises((CrcError, ProtocolError)):
                decode(bytes(corrupt))

    def test_bad_lengths_are_rejected(self) -> None:
        encoded = encode(self.frame)
        with self.assertRaises(ProtocolError):
            decode(encoded[:-1])
        with self.assertRaises(ProtocolError):
            decode(encoded + b"x")

    def test_sequence_rollover(self) -> None:
        self.assertTrue(sequence_is_newer(0, 65535))
        self.assertTrue(sequence_is_newer(5, 2))
        self.assertFalse(sequence_is_newer(65535, 0))
        self.assertFalse(sequence_is_newer(4, 4))

    def test_frozen_v2_golden_frame(self) -> None:
        frame = TelemetryFrame.from_state(
            KinematicState(0x1234, Vec3(1.23, -4.56, 7.89), Vec3(-1, 2, -3), 12.345),
            sequence=0xABCD, boot_id=0xBEEF, ttl=5, flags=1,
        )
        self.assertEqual(
            encode(frame).hex(),
            "d702010105001234abcdbeef000030390000007bfffffe3800000315ff9c00c8fed4bce9",
        )

    def test_random_round_trips(self) -> None:
        rng = random.Random(81)
        for sequence in range(500):
            state = KinematicState(
                rng.randrange(65536),
                Vec3(*(rng.uniform(-1000, 1000) for _ in range(3))),
                Vec3(*(rng.uniform(-100, 100) for _ in range(3))),
                rng.uniform(0, 100_000),
            )
            frame = TelemetryFrame.from_state(state, sequence=sequence)
            self.assertEqual(decode(encode(frame)), frame)


if __name__ == "__main__":
    unittest.main()
