from __future__ import annotations

import random
import unittest

from drone_sims.avoidance import AvoidanceConfig, AvoidanceController, AvoidancePhase
from drone_sims.collision import KinematicState, Vec3


class AvoidanceControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.own = KinematicState(1, Vec3(0, 0), Vec3(0, 0), 0)
        self.threat = KinematicState(2, Vec3(10, 0), Vec3(-2, 0), 0)

    def test_successful_state_machine(self) -> None:
        controller = AvoidanceController(AvoidanceConfig(command_delay=0.1, acknowledgement_delay=0.1, maneuver_duration=1, cooldown=1))
        self.assertTrue(controller.request(self.own, self.threat, 0, random.Random(1)))
        self.assertIsNone(controller.update(self.own, self.threat, 0.19))
        velocity = controller.update(self.own, self.threat, 0.2)
        self.assertEqual(controller.phase, AvoidancePhase.MANEUVERING)
        self.assertEqual(velocity, Vec3(0, 3.0, 0))
        self.assertEqual(controller.update(self.own, self.threat, 1.2), Vec3(0, 0, 0))
        self.assertEqual(controller.phase, AvoidancePhase.COOLDOWN)
        controller.update(self.own, self.threat, 2.2)
        self.assertEqual(controller.phase, AvoidancePhase.NORMAL)

    def test_lost_command_times_out(self) -> None:
        controller = AvoidanceController(AvoidanceConfig(command_loss=1, command_timeout=0.5))
        controller.request(self.own, self.threat, 0, random.Random(1))
        controller.update(self.own, self.threat, 0.5)
        self.assertEqual(controller.phase, AvoidancePhase.FAILED)
        self.assertEqual(controller.failures, 1)

    def test_rejected_command_fails_immediately(self) -> None:
        controller = AvoidanceController(AvoidanceConfig(command_rejection=1))
        controller.request(self.own, self.threat, 0, random.Random(1))
        self.assertEqual(controller.phase, AvoidancePhase.FAILED)


if __name__ == "__main__":
    unittest.main()
