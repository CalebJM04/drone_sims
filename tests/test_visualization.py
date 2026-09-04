from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from drone_sims.visualization import build_dashboard, dashboard_data


PROJECT = Path(__file__).resolve().parents[1]


class VisualizationTests(unittest.TestCase):
    def test_dashboard_data_contains_scenario_trace(self) -> None:
        data = dashboard_data(PROJECT / "results/full", scenarios=["head_on"], seed=7)
        self.assertEqual(data["traces"][0]["name"], "head_on")
        self.assertGreater(len(data["traces"][0]["trace"]), 100)

    def test_dashboard_is_self_contained_html(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "dashboard.html"
            build_dashboard(PROJECT / "results/full", output, scenarios=["crossing"])
            rendered = output.read_text(encoding="utf-8")
            self.assertIn("Pre-hardware flight-safety dashboard", rendered)
            self.assertIn('"name":"crossing"', rendered)
            self.assertNotIn("__DASHBOARD_DATA__", rendered)
            self.assertNotIn("https://", rendered)


if __name__ == "__main__":
    unittest.main()
