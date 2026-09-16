"""The core must stay runnable while predictive routing remains unimplemented."""
import unittest

from drone_sims.collision import KinematicState, Vec3
from drone_sims.companion import CompanionConfig
from drone_sims.network_awareness import predict_link
from drone_sims.routing import RoutingPolicy


class PendingRoutingTests(unittest.TestCase):
    def test_proactive_mode_cannot_silently_use_basic_flooding(self):
        with self.assertRaises(NotImplementedError):
            RoutingPolicy(mode="proactive").should_forward(2, 1, 3)
        with self.assertRaises(ValueError):
            CompanionConfig(1, 10, routing_mode="proactive")

    def test_link_prediction_is_explicitly_pending(self):
        first = KinematicState(1, Vec3(0, 0), Vec3(1, 0), 0)
        second = KinematicState(2, Vec3(10, 0), Vec3(-1, 0), 0)
        with self.assertRaises(NotImplementedError):
            predict_link(first, second, now=0, range_m=30, lookahead_s=4, warning_margin_m=5)
