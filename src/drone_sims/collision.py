from __future__ import annotations

from dataclasses import dataclass
import math
import random


@dataclass(frozen=True, slots=True)
class Vec3:
    x: float
    y: float
    z: float = 0.0

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__

    def dot(self, other: "Vec3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def norm2(self) -> float:
        return self.dot(self)

    def norm(self) -> float:
        return math.sqrt(self.norm2())

    def unit_xy(self) -> "Vec3":
        length = math.hypot(self.x, self.y)
        return Vec3(1.0, 0.0) if length <= 1e-12 else Vec3(self.x / length, self.y / length)

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z]


@dataclass(frozen=True, slots=True)
class KinematicState:
    node_id: int
    position: Vec3
    velocity: Vec3
    timestamp: float

    def at(self, timestamp: float) -> "KinematicState":
        dt = max(0.0, timestamp - self.timestamp)
        return KinematicState(self.node_id, self.position + self.velocity * dt, self.velocity, timestamp)


@dataclass(frozen=True, slots=True)
class Assessment:
    risk: bool
    tcpa: float | None
    dcpa: float
    current_distance: float
    reason: str


def assess(
    own: KinematicState,
    peer: KinematicState,
    *,
    now: float,
    safety_distance: float,
    horizon: float,
    max_age: float,
) -> Assessment:
    age = now - peer.timestamp
    if age < -1e-9:
        return Assessment(False, None, math.inf, math.inf, "future")
    if age > max_age:
        return Assessment(False, None, math.inf, math.inf, "stale")
    own_now, peer_now = own.at(now), peer.at(now)
    relative_position = own_now.position - peer_now.position
    relative_velocity = own_now.velocity - peer_now.velocity
    distance = relative_position.norm()
    speed2 = relative_velocity.norm2()
    if speed2 <= 1e-12:
        risk = distance < safety_distance
        return Assessment(risk, 0.0 if risk else None, distance, distance, "inside" if risk else "parallel")
    tcpa = -relative_position.dot(relative_velocity) / speed2
    if tcpa < 0.0:
        return Assessment(False, tcpa, distance, distance, "separating")
    dcpa = (relative_position + relative_velocity * tcpa).norm()
    if tcpa > horizon:
        return Assessment(False, tcpa, dcpa, distance, "outside_horizon")
    risk = dcpa < safety_distance
    return Assessment(risk, tcpa, dcpa, distance, "risk" if risk else "clear")


def noisy_state(
    state: KinematicState,
    rng: random.Random,
    *,
    position_sigma: float = 0.0,
    velocity_sigma: float = 0.0,
    clock_sigma: float = 0.0,
) -> KinematicState:
    gaussian = rng.gauss
    return KinematicState(
        state.node_id,
        Vec3(
            state.position.x + gaussian(0.0, position_sigma),
            state.position.y + gaussian(0.0, position_sigma),
            state.position.z + gaussian(0.0, position_sigma),
        ),
        Vec3(
            state.velocity.x + gaussian(0.0, velocity_sigma),
            state.velocity.y + gaussian(0.0, velocity_sigma),
            state.velocity.z + gaussian(0.0, velocity_sigma),
        ),
        state.timestamp + gaussian(0.0, clock_sigma),
    )


def brute_force_closest_approach(
    own: KinematicState,
    peer: KinematicState,
    *,
    horizon: float,
    step: float = 0.002,
) -> tuple[float, float]:
    count = max(1, math.ceil(horizon / step))
    best_time, best_distance = 0.0, math.inf
    for index in range(count + 1):
        timestamp = min(horizon, index * step)
        distance = (own.position + own.velocity * timestamp - peer.position - peer.velocity * timestamp).norm()
        if distance < best_distance:
            best_time, best_distance = timestamp, distance
    return best_time, best_distance
