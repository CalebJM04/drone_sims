from __future__ import annotations

import importlib.util
import unittest

from drone_sims.mavlink_codec import VelocityCommand, decode_message, encode_global_position, encode_velocity_command


@unittest.skipUnless(importlib.util.find_spec("pymavlink"), "optional pymavlink dependency is not installed")
class MavlinkCodecTests(unittest.TestCase):
    def test_velocity_command_round_trip(self) -> None:
        encoded = encode_velocity_command(VelocityCommand(1.25, -2.5, 0.75), 1234)
        message = decode_message(encoded)
        self.assertEqual(message.get_type(), "SET_POSITION_TARGET_LOCAL_NED")
        self.assertAlmostEqual(message.vx, 1.25)
        self.assertAlmostEqual(message.vy, -2.5)
        self.assertAlmostEqual(message.vz, 0.75)
        self.assertEqual(message.type_mask, 3527)

    def test_global_position_round_trip_and_units(self) -> None:
        encoded = encode_global_position(
            time_boot_ms=5000,
            latitude_deg=35.9606,
            longitude_deg=-83.9207,
            altitude_m=312.5,
            relative_altitude_m=12.5,
            north_mps=1.2,
            east_mps=-0.4,
            down_mps=0.1,
            heading_deg=271.25,
            source_system=2,
        )
        message = decode_message(encoded)
        self.assertEqual(message.get_type(), "GLOBAL_POSITION_INT")
        self.assertEqual(message.lat, 359606000)
        self.assertEqual(message.lon, -839207000)
        self.assertEqual(message.relative_alt, 12500)
        self.assertEqual(message.hdg, 27125)


if __name__ == "__main__":
    unittest.main()
