from __future__ import annotations

from dataclasses import dataclass
import math

from .collision import KinematicState, Vec3, assess


@dataclass(frozen=True, slots=True)
class PlannerConfig:
    maneuver_speed: float = 3.0
    vertical_speed: float = 1.5
    horizon: float = 5.0
    safety_distance: float = 3.0
    min_z: float = -20.0
    max_z: float = 20.0
    deviation_penalty: float = 0.05


@dataclass(frozen=True, slots=True)
class ManeuverCandidate:
    name: str
    velocity: Vec3
    worst_separation: float
    score: float
    feasible: bool


def _candidate_velocities(own: KinematicState, config: PlannerConfig) -> list[tuple[str, Vec3]]:
    horizontal = own.velocity.unit_xy()
    left = Vec3(-horizontal.y, horizontal.x)
    right = left * -1.0
    return [
        ("continue", own.velocity),
        ("hold", Vec3(0.0, 0.0, 0.0)),
        ("slow", own.velocity * 0.35),
        ("left", left * config.maneuver_speed),
        ("right", right * config.maneuver_speed),
        ("climb", Vec3(own.velocity.x, own.velocity.y, -config.vertical_speed)),
        ("descend", Vec3(own.velocity.x, own.velocity.y, config.vertical_speed)),
    ]


def plan_maneuver(
    own: KinematicState,
    threats: list[KinematicState],
    config: PlannerConfig,
) -> tuple[ManeuverCandidate, list[ManeuverCandidate]]:
    if not threats:
        candidate = ManeuverCandidate("continue", own.velocity, math.inf, 0.0, True)
        return candidate, [candidate]
    evaluated: list[ManeuverCandidate] = []
    for name, velocity in _candidate_velocities(own, config):
        future_z = own.position.z + velocity.z * config.horizon
        feasible = config.min_z <= future_z <= config.max_z
        worst = math.inf
        for threat in threats:
            result = assess(
                KinematicState(own.node_id, own.position, velocity, own.timestamp),
                threat,
                now=own.timestamp,
                safety_distance=config.safety_distance,
                horizon=config.horizon,
                max_age=math.inf,
            )
            if result.tcpa is None or result.tcpa < 0:
                separation = result.current_distance
            elif result.tcpa > config.horizon:
                separation = (
                    own.position + velocity * config.horizon
                    - threat.position - threat.velocity * config.horizon
                ).norm()
            else:
                separation = result.dcpa
            worst = min(worst, separation)
        deviation = (velocity - own.velocity).norm()
        score = worst - config.deviation_penalty * deviation if feasible else -math.inf
        evaluated.append(ManeuverCandidate(name, velocity, worst, score, feasible))
    best = max(evaluated, key=lambda candidate: candidate.score)
    return best, evaluated
