from __future__ import annotations

import unittest

from drone_sims.campaign import run_campaign
from drone_sims.scenarios import build


class EndToEndTests(unittest.TestCase):
    def test_link_failure_uses_three_hop_alternate_path(self) -> None:
        simulation = build("head_on", 7)
        result = simulation.run()
        alternate = [
            event for event in simulation.events
            if event["type"] == "packet_delivered"
            and event["source"] == 1
            and event["receiver"] == 5
            and event["time"] >= 3.0
            and event["path"] == [1, 3, 4, 5]
        ]
        self.assertTrue(alternate)
        self.assertGreaterEqual(result["network"]["max_hops"], 3)

    def test_avoidance_changes_outcome(self) -> None:
        avoided = build("head_on", 3, event_logging=False).run()
        baseline = build("no_avoidance", 3, event_logging=False).run()
        self.assertGreaterEqual(avoided["avoidance"]["minimum_separation_m"], 4)
        self.assertLess(baseline["avoidance"]["minimum_separation_m"], 0.1)

    def test_command_loss_is_observable(self) -> None:
        result = build("command_loss", 4, event_logging=False).run()
        self.assertEqual(result["avoidance"]["maneuvers"], 0)
        self.assertEqual(result["avoidance"]["command_failures"], 1)
        self.assertLess(result["avoidance"]["minimum_separation_m"], 0.1)

    def test_acceleration_limits_change_avoidance_result(self) -> None:
        ideal = build("head_on", 4, event_logging=False).run()
        limited = build("limited_dynamics", 4, event_logging=False).run()
        self.assertLess(
            limited["avoidance"]["minimum_separation_m"],
            ideal["avoidance"]["minimum_separation_m"],
        )

    def test_vertical_separation_has_no_alarm(self) -> None:
        result = build("vertical_clear", 2, event_logging=False).run()
        self.assertEqual(result["prediction"]["alarms"], 0)
        self.assertAlmostEqual(result["avoidance"]["minimum_separation_m"], 8.0)

    def test_aloha_congestion_collapse_is_captured(self) -> None:
        result = build("congested", 1, event_logging=False).run()
        self.assertGreater(result["network"]["mac_collisions"], 100)
        self.assertLess(result["network"]["delivery_ratio"], 0.1)

    def test_small_campaign_is_reproducible(self) -> None:
        first = run_campaign(["head_on", "noisy"], 2)
        second = run_campaign(["head_on", "noisy"], 2)
        self.assertEqual(first, second)

    def test_event_logging_records_replayable_state_trace(self) -> None:
        simulation = build("crossing", 7)
        result = simulation.run()
        self.assertGreaterEqual(result["avoidance"]["minimum_separation_m"], 4)
        self.assertGreater(len(simulation.trace), 50)
        self.assertEqual(simulation.trace[0]["time"], 0.0)
        self.assertEqual(len(simulation.trace[0]["nodes"]), 3)
        self.assertIn("separation_m", simulation.trace[-1])


if __name__ == "__main__":
    unittest.main()
