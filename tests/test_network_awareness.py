from __future__ import annotations

import tempfile
from pathlib import Path
import unittest

from drone_sims.collision import KinematicState, Vec3
from drone_sims.mesh_demo import run_mesh_demo
from drone_sims.mesh_visualization import build_mesh_dashboard
from drone_sims.network_awareness import (
    LinkQualityTable,
    planned_routes,
    predict_link,
)
from drone_sims.routing import RoutingPolicy


def state(node: int, x: float, y: float = 0.0, vx: float = 0.0) -> KinematicState:
    return KinematicState(node, Vec3(x, y), Vec3(vx, 0.0), 0.0)


class NetworkAwarenessTests(unittest.TestCase):
    def test_predicts_link_loss_before_range_is_crossed(self) -> None:
        link = predict_link(
            state(1, 0), state(2, 20, vx=2), now=0,
            range_m=30, lookahead_s=6, warning_margin_m=5,
        )
        self.assertTrue(link.connected_now)
        self.assertFalse(link.connected_projected)
        self.assertEqual(link.status, "critical")
        self.assertAlmostEqual(link.time_to_loss_s or 0, 5.0)

    def test_lookahead_route_uses_relay_before_direct_link_breaks(self) -> None:
        states = {
            1: state(1, 0),
            2: state(2, 20, vx=2),
            3: state(3, 12, 12),
            4: state(4, 28, 12),
        }
        routes = planned_routes(
            states, source=1, now=0, range_m=30, lookahead_s=4, route_margin=0.9
        )
        self.assertGreater(len(routes[2]), 2)
        policy = RoutingPolicy(
            mode="proactive", proactive_range_m=30, proactive_lookahead_s=4,
            proactive_route_margin=0.9,
        )
        self.assertTrue(policy.should_forward(3, 1, 1, states=states, now=0))
        self.assertTrue(policy.should_forward(4, 1, 5, states=states, now=0))

    def test_link_quality_tracks_delivery_and_signal(self) -> None:
        table = LinkQualityTable(1.0)
        table.observe(2, (2, 1, 1), 0.0, rssi_dbm=-90, snr_db=8)
        table.observe(2, (2, 1, 2), 2.0, rssi_dbm=-94, snr_db=4)
        snapshot = table.snapshot(2.0)[0]
        self.assertEqual(snapshot["received"], 2)
        self.assertEqual(snapshot["expected"], 3)
        self.assertAlmostEqual(snapshot["delivery_ratio"], 2 / 3)
        self.assertEqual(snapshot["rssi_dbm"], -91.0)

    def test_six_node_full_stack_demo_meets_acceptance(self) -> None:
        report = run_mesh_demo(nodes=6, duration_s=10, seed=31)
        self.assertEqual(report["status"], "PASS")
        self.assertTrue(all(report["checks"].values()))
        self.assertEqual(report["summary"]["control_commands"], 0)
        handoff = next(
            event for event in report["events"]
            if event["type"] == "preemptive_route_handoff"
        )
        self.assertTrue(handoff["observed_relay_transmitters"])
        with tempfile.TemporaryDirectory() as directory:
            destination = build_mesh_dashboard(report, Path(directory) / "mesh.html")
            rendered = destination.read_text(encoding="utf-8")
        self.assertIn("4+ node awareness mesh", rendered)
        self.assertIn("preemptive_route_handoff", rendered)


if __name__ == "__main__":
    unittest.main()
