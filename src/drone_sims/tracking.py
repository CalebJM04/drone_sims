from __future__ import annotations

from dataclasses import dataclass
import math

from .collision import Assessment, KinematicState, Vec3, assess


@dataclass(frozen=True, slots=True)
class TrackedState:
    state: KinematicState
    received_at: float
    position_sigma: float
    velocity_sigma: float
    clock_sigma: float
    samples: int

    def uncertainty_at(self, now: float, horizon: float = 0.0) -> float:
        age = max(0.0, now - self.state.timestamp)
        propagation = age + max(0.0, horizon) + self.clock_sigma
        return math.sqrt(self.position_sigma**2 + (propagation * self.velocity_sigma) ** 2)


class AlphaBetaTracker:
    """Small deterministic tracker suitable for the embedded software reference."""

    def __init__(self, alpha: float = 0.65, beta: float = 0.12) -> None:
        if not 0 < alpha <= 1 or not 0 <= beta <= 1:
            raise ValueError("alpha and beta must be within [0, 1], with alpha nonzero")
        self.alpha = alpha
        self.beta = beta
        self.tracks: dict[int, TrackedState] = {}

    def update(
        self,
        measurement: KinematicState,
        received_at: float,
        *,
        position_sigma: float,
        velocity_sigma: float,
        clock_sigma: float,
    ) -> TrackedState:
        previous = self.tracks.get(measurement.node_id)
        if previous is None or measurement.timestamp <= previous.state.timestamp:
            if previous is not None:
                return previous
            tracked = TrackedState(measurement, received_at, position_sigma, velocity_sigma, clock_sigma, 1)
        else:
            dt = measurement.timestamp - previous.state.timestamp
            predicted = previous.state.at(measurement.timestamp)
            residual = measurement.position - predicted.position
            filtered_position = predicted.position + residual * self.alpha
            residual_velocity = residual * (self.beta / max(dt, 1e-6))
            measured_velocity = previous.state.velocity + residual_velocity
            filtered_velocity = measured_velocity * self.alpha + measurement.velocity * (1.0 - self.alpha)
            samples = previous.samples + 1
            tracked = TrackedState(
                KinematicState(measurement.node_id, filtered_position, filtered_velocity, measurement.timestamp),
                received_at,
                max(position_sigma * 0.6, previous.position_sigma * 0.8),
                max(velocity_sigma * 0.6, previous.velocity_sigma * 0.8),
                clock_sigma,
                samples,
            )
        self.tracks[measurement.node_id] = tracked
        return tracked

    def expire(self, now: float, max_age: float) -> list[int]:
        expired = [node for node, track in self.tracks.items() if now - track.received_at > max_age]
        for node in expired:
            del self.tracks[node]
        return expired


@dataclass(frozen=True, slots=True)
class UncertainAssessment:
    nominal: Assessment
    conservative: Assessment
    effective_safety_distance: float
    uncertainty_margin: float

    @property
    def risk(self) -> bool:
        return self.conservative.risk


def assess_uncertain(
    own: KinematicState,
    peer: TrackedState,
    *,
    now: float,
    safety_distance: float,
    horizon: float,
    max_age: float,
    sigma_multiplier: float = 2.5,
) -> UncertainAssessment:
    nominal = assess(
        own,
        peer.state,
        now=now,
        safety_distance=safety_distance,
        horizon=horizon,
        max_age=max_age,
    )
    prediction_time = max(0.0, min(horizon, nominal.tcpa or 0.0))
    margin = sigma_multiplier * peer.uncertainty_at(now, prediction_time)
    effective = safety_distance + margin
    conservative = assess(
        own,
        peer.state,
        now=now,
        safety_distance=effective,
        horizon=horizon,
        max_age=max_age,
    )
    return UncertainAssessment(nominal, conservative, effective, margin)

