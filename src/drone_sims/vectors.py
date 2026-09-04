from __future__ import annotations

import csv
from pathlib import Path
import random

from .collision import KinematicState, Vec3
from .fixedpoint import fixed_point_assess
from .protocol import TelemetryFrame, encode


def generate_golden_vectors(path: str | Path, count: int = 10_000, seed: int = 401) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)
    columns = [
        "case", "pa_x_cm", "pa_y_cm", "pa_z_cm", "va_x_cms", "va_y_cms", "va_z_cms",
        "pb_x_cm", "pb_y_cm", "pb_z_cm", "vb_x_cms", "vb_y_cms", "vb_z_cms",
        "safety_cm", "horizon_ms", "expected_risk", "expected_tcpa_ms", "expected_dcpa_cm",
        "telemetry_a_hex", "telemetry_b_hex",
    ]
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for case in range(count):
            pa = [rng.randint(-20_000, 20_000) for _ in range(3)]
            pb = [rng.randint(-20_000, 20_000) for _ in range(3)]
            va = [rng.randint(-2_000, 2_000) for _ in range(3)]
            vb = [rng.randint(-2_000, 2_000) for _ in range(3)]
            safety, horizon = rng.randint(100, 1_000), rng.randint(1_000, 15_000)
            own = KinematicState(1, Vec3(*(x / 100 for x in pa)), Vec3(*(x / 100 for x in va)), 0)
            peer = KinematicState(2, Vec3(*(x / 100 for x in pb)), Vec3(*(x / 100 for x in vb)), 0)
            expected = fixed_point_assess(own, peer, safety_distance_cm=safety, horizon_ms=horizon)
            frame_a = TelemetryFrame.from_state(own, sequence=case)
            frame_b = TelemetryFrame.from_state(peer, sequence=case)
            writer.writerow({
                "case": case,
                **{f"pa_{axis}_cm": value for axis, value in zip("xyz", pa)},
                **{f"va_{axis}_cms": value for axis, value in zip("xyz", va)},
                **{f"pb_{axis}_cm": value for axis, value in zip("xyz", pb)},
                **{f"vb_{axis}_cms": value for axis, value in zip("xyz", vb)},
                "safety_cm": safety,
                "horizon_ms": horizon,
                "expected_risk": int(expected.risk),
                "expected_tcpa_ms": "" if expected.tcpa_ms is None else expected.tcpa_ms,
                "expected_dcpa_cm": expected.dcpa_cm,
                "telemetry_a_hex": encode(frame_a).hex(),
                "telemetry_b_hex": encode(frame_b).hex(),
            })
