from __future__ import annotations

import random
import unittest

from drone_sims.collision import KinematicState, Vec3
from drone_sims.lora_phy import COMMON_PROFILES, LoRaModem
from drone_sims.planner import PlannerConfig, plan_maneuver
from drone_sims.protocol import FRAME_SIZE
from drone_sims.radio import RadioConfig, RadioMedium
from drone_sims.routing import RoutingPolicy
from drone_sims.tracking import AlphaBetaTracker, TrackedState, assess_uncertain


class LoRaPhyTests(unittest.TestCase):
    def test_known_time_on_air_values(self) -> None:
        expected = {"fast": 0.038528, "balanced": 0.267264, "long_range": 1.974272}
        for name, seconds in expected.items():
            self.assertAlmostEqual(COMMON_PROFILES[name].airtime_seconds(FRAME_SIZE), seconds, places=9)

    def test_low_data_rate_optimization_and_capacity(self) -> None:
        self.assertFalse(COMMON_PROFILES["balanced"].low_rate_enabled)
        self.assertTrue(COMMON_PROFILES["long_range"].low_rate_enabled)
        full = COMMON_PROFILES["balanced"].capacity_packets_per_second(FRAME_SIZE)
        self.assertAlmostEqual(
            COMMON_PROFILES["balanced"].capacity_packets_per_second(FRAME_SIZE, 0.01),
            full * 0.01,
        )

    def test_duty_cycle_defers_second_transmission(self) -> None:
        positions = {1: Vec3(0, 0), 2: Vec3(1, 0)}
        config = RadioConfig(
            modem=LoRaModem(spreading_factor=7), duty_cycle=0.1,
            max_backoff_s=0, processing_s=0, propagation_s=0,
            random_loss=0, burst_enter=0, bit_corruption=0,
        )
        medium = RadioMedium(config, random.Random(1), positions.__getitem__)
        medium.register(1)
        medium.register(2)
        medium.broadcast(1, b"x" * FRAME_SIZE, (1,), 0)
        medium.broadcast(1, b"x" * FRAME_SIZE, (1,), 0)
        first, second = sorted(medium.poll(10), key=lambda item: item.started_at)
        self.assertGreaterEqual(second.started_at, first.ends_at + config.airtime(FRAME_SIZE) * 9 - 1e-9)


class TrackingAndPlannerTests(unittest.TestCase):
    def test_tracker_filters_position_residual_and_reduces_sigma(self) -> None:
        tracker = AlphaBetaTracker(alpha=0.5, beta=0.1)
        tracker.update(KinematicState(2, Vec3(0, 0), Vec3(1, 0), 0), 0, position_sigma=1, velocity_sigma=.2, clock_sigma=.01)
        result = tracker.update(KinematicState(2, Vec3(2, 0), Vec3(1, 0), 1), 1, position_sigma=1, velocity_sigma=.2, clock_sigma=.01)
        self.assertAlmostEqual(result.state.position.x, 1.5)
        self.assertLess(result.position_sigma, 1)
        self.assertEqual(result.samples, 2)

    def test_uncertainty_can_turn_nominal_clear_into_conservative_risk(self) -> None:
        own = KinematicState(1, Vec3(0, 0), Vec3(0, 0), 0)
        peer = TrackedState(KinematicState(2, Vec3(10, 3.4), Vec3(-2, 0), 0), 0, .5, .1, 0, 1)
        result = assess_uncertain(own, peer, now=0, safety_distance=3, horizon=6, max_age=1)
        self.assertFalse(result.nominal.risk)
        self.assertTrue(result.conservative.risk)

    def test_multi_threat_planner_improves_worst_separation(self) -> None:
        own = KinematicState(1, Vec3(0, 0), Vec3(0, 0), 0)
        threats = [
            KinematicState(2, Vec3(10, 0), Vec3(-2, 0), 0),
            KinematicState(3, Vec3(0, 10), Vec3(0, -2), 0),
        ]
        chosen, candidates = plan_maneuver(own, threats, PlannerConfig())
        continuing = next(item for item in candidates if item.name == "continue")
        self.assertGreater(chosen.worst_separation, continuing.worst_separation)
        self.assertGreaterEqual(chosen.worst_separation, 3)

    def test_routing_policies_are_deterministic(self) -> None:
        relay = RoutingPolicy(mode="relay", relay_nodes=(2, 4))
        self.assertTrue(relay.should_forward(2, 1, 9))
        self.assertFalse(relay.should_forward(3, 1, 9))
        policy = RoutingPolicy(mode="probabilistic", seed=42)
        self.assertEqual(policy.should_forward(3, 1, 9), policy.should_forward(3, 1, 9))


if __name__ == "__main__":
    unittest.main()
