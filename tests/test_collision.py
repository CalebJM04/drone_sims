from __future__ import annotations

import random
import unittest

from drone_sims.collision import KinematicState, Vec3, assess, brute_force_closest_approach


class CollisionAlgorithmTests(unittest.TestCase):
    def state(self, node: int, p: tuple[float, float, float], v: tuple[float, float, float], t: float = 0.0) -> KinematicState:
        return KinematicState(node, Vec3(*p), Vec3(*v), t)

    def test_head_on(self) -> None:
        result = assess(self.state(1, (0, 0, 0), (0, 0, 0)), self.state(2, (10, 0, 0), (-2, 0, 0)), now=0, safety_distance=3, horizon=6, max_age=1)
        self.assertTrue(result.risk)
        self.assertAlmostEqual(result.tcpa or -1, 5)
        self.assertAlmostEqual(result.dcpa, 0)

    def test_crossing(self) -> None:
        result = assess(self.state(1, (-5, 0, 0), (1, 0, 0)), self.state(2, (0, -5, 0), (0, 1, 0)), now=0, safety_distance=2, horizon=10, max_age=1)
        self.assertTrue(result.risk)
        self.assertAlmostEqual(result.tcpa or -1, 5)

    def test_vertical_separation_is_safe(self) -> None:
        result = assess(self.state(1, (-5, 0, 0), (1, 0, 0)), self.state(2, (0, -5, 8), (0, 1, 0)), now=0, safety_distance=3, horizon=10, max_age=1)
        self.assertFalse(result.risk)
        self.assertAlmostEqual(result.dcpa, 8)

    def test_overtaking(self) -> None:
        result = assess(self.state(1, (0, 0, 0), (3, 0, 0)), self.state(2, (8, 0, 0), (1, 0, 0)), now=0, safety_distance=2, horizon=5, max_age=1)
        self.assertTrue(result.risk)
        self.assertAlmostEqual(result.tcpa or -1, 4)

    def test_parallel_and_separating(self) -> None:
        parallel = assess(self.state(1, (0, 0, 0), (1, 0, 0)), self.state(2, (0, 5, 0), (1, 0, 0)), now=0, safety_distance=3, horizon=10, max_age=1)
        separating = assess(self.state(1, (0, 0, 0), (-1, 0, 0)), self.state(2, (4, 0, 0), (1, 0, 0)), now=0, safety_distance=3, horizon=10, max_age=1)
        self.assertEqual(parallel.reason, "parallel")
        self.assertEqual(separating.reason, "separating")

    def test_stale_and_future_states_are_rejected(self) -> None:
        own = self.state(1, (0, 0, 0), (0, 0, 0), 2)
        stale = assess(own, self.state(2, (1, 0, 0), (0, 0, 0), 0), now=2, safety_distance=3, horizon=5, max_age=1)
        future = assess(own, self.state(2, (1, 0, 0), (0, 0, 0), 3), now=2, safety_distance=3, horizon=5, max_age=1)
        self.assertEqual(stale.reason, "stale")
        self.assertEqual(future.reason, "future")

    def test_randomized_solution_matches_sampling_oracle(self) -> None:
        rng = random.Random(932)
        for _ in range(250):
            own = self.state(1, tuple(rng.uniform(-20, 20) for _ in range(3)), tuple(rng.uniform(-4, 4) for _ in range(3)))
            peer = self.state(2, tuple(rng.uniform(-20, 20) for _ in range(3)), tuple(rng.uniform(-4, 4) for _ in range(3)))
            result = assess(own, peer, now=0, safety_distance=3, horizon=8, max_age=1)
            _, sampled_distance = brute_force_closest_approach(own, peer, horizon=8, step=0.01)
            analytic_distance = min(
                (own.position - peer.position).norm(),
                result.dcpa if result.tcpa is not None and 0 <= result.tcpa <= 8 else float("inf"),
                (own.position + own.velocity * 8 - peer.position - peer.velocity * 8).norm(),
            )
            self.assertAlmostEqual(analytic_distance, sampled_distance, delta=0.08)


if __name__ == "__main__":
    unittest.main()

