from __future__ import annotations

from dataclasses import dataclass
import math

from .collision import KinematicState


@dataclass(frozen=True, slots=True)
class FixedAssessment:
    risk: bool
    tcpa_ms: int | None
    dcpa_cm: int
    saturated: bool = False


def fixed_point_assess(
    own: KinematicState,
    peer: KinematicState,
    *,
    safety_distance_cm: int,
    horizon_ms: int,
) -> FixedAssessment:
    """Independent integer model used to catch embedded numeric regressions."""
    pa = [round(value * 100.0) for value in own.position.as_list()]
    pb = [round(value * 100.0) for value in peer.position.as_list()]
    va = [round(value * 100.0) for value in own.velocity.as_list()]
    vb = [round(value * 100.0) for value in peer.velocity.as_list()]
    pr = [a - b for a, b in zip(pa, pb)]
    vr = [a - b for a, b in zip(va, vb)]
    dot = sum(p * v for p, v in zip(pr, vr))
    speed2 = sum(v * v for v in vr)
    current_cm = math.isqrt(sum(p * p for p in pr))
    if speed2 == 0:
        return FixedAssessment(current_cm < safety_distance_cm, 0 if current_cm < safety_distance_cm else None, current_cm)
    if dot >= 0:
        return FixedAssessment(False, None, current_cm)
    tcpa_ms = (-dot * 1000 + speed2 // 2) // speed2
    closest_numerators = [p * 1000 + v * tcpa_ms for p, v in zip(pr, vr)]
    dcpa_cm = math.isqrt(sum(value * value for value in closest_numerators)) // 1000
    return FixedAssessment(tcpa_ms <= horizon_ms and dcpa_cm < safety_distance_cm, tcpa_ms, dcpa_cm)
