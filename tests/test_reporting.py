from __future__ import annotations

import json
from pathlib import Path
import unittest

from drone_sims.reporting import final_results_text, verification_text


PROJECT = Path(__file__).resolve().parents[1]
RESULTS = PROJECT / "results/full"


class ReportingTests(unittest.TestCase):
    def load(self, name: str):
        return json.loads((RESULTS / name).read_text(encoding="utf-8"))

    def test_verification_report_is_plain_text(self) -> None:
        text = verification_text(self.load("verification.json"))
        self.assertIn("Simulation checks: 13/13 passed", text)
        self.assertIn("Scenario results", text)
        self.assertNotIn("#", text)
        self.assertNotIn("|---", text)
        self.assertNotIn("**", text)

    def test_final_report_is_short_and_direct(self) -> None:
        readiness = self.load("pre_hardware_readiness.json")
        closed_loop = self.load("px4_closed_loop.json")
        text = final_results_text(
            readiness,
            self.load("verification.json"),
            self.load("network_matrix.json"),
            self.load("px4_sitl.json"),
            closed_loop,
        )
        self.assertIn(
            f"Overall: PASS ({readiness['passed']}/{readiness['total']} checks passed)",
            text,
        )
        self.assertIn(
            f"PX4 collision test: {closed_loop['passed']}/{closed_loop['total']} passed",
            text,
        )
        self.assertLess(len(text.splitlines()), 30)
        self.assertNotIn("**", text)


if __name__ == "__main__":
    unittest.main()
