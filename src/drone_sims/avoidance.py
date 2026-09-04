from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import random

from .collision import KinematicState, Vec3


class AvoidancePhase(str, Enum):
    NORMAL = "normal"
    COMMAND_PENDING = "command_pending"
    MANEUVERING = "maneuvering"
    COOLDOWN = "cooldown"
    FAILED = "failed"


@dataclass(slots=True)
class AvoidanceConfig:
    lateral_speed: float = 3.0
    maneuver_duration: float = 6.0
    cooldown: float = 3.0
    command_delay: float = 0.08
    acknowledgement_delay: float = 0.04
    command_loss: float = 0.0
    command_rejection: float = 0.0
    command_timeout: float = 0.5


@dataclass(slots=True)
class AvoidanceController:
    config: AvoidanceConfig
    phase: AvoidancePhase = AvoidancePhase.NORMAL
    threat: int | None = None
    command_sent_at: float | None = None
    execute_at: float | None = None
    finish_at: float | None = None
    cooldown_until: float | None = None
    original_velocity: Vec3 | None = None
    maneuvers: int = 0
    failures: int = 0
    planned_velocity: Vec3 | None = None
    planned_maneuver: str | None = None

    def request(
        self,
        own: KinematicState,
        threat: KinematicState,
        now: float,
        rng: random.Random,
        *,
        command_velocity: Vec3 | None = None,
        maneuver_name: str | None = None,
    ) -> bool:
        if self.phase is not AvoidancePhase.NORMAL:
            return False
        self.threat = threat.node_id
        self.command_sent_at = now
        self.original_velocity = own.velocity
        self.planned_velocity = command_velocity
        self.planned_maneuver = maneuver_name
        if rng.random() < self.config.command_loss:
            self.phase = AvoidancePhase.COMMAND_PENDING
            self.execute_at = None
            return True
        if rng.random() < self.config.command_rejection:
            self.phase = AvoidancePhase.FAILED
            self.failures += 1
            return True
        self.phase = AvoidancePhase.COMMAND_PENDING
        self.execute_at = now + self.config.command_delay + self.config.acknowledgement_delay
        return True

    def update(self, own: KinematicState, threat: KinematicState | None, now: float) -> Vec3 | None:
        if self.phase is AvoidancePhase.COMMAND_PENDING:
            assert self.command_sent_at is not None
            if self.execute_at is None and now - self.command_sent_at >= self.config.command_timeout:
                self.phase = AvoidancePhase.FAILED
                self.failures += 1
            elif self.execute_at is not None and now >= self.execute_at:
                direction = (threat.position - own.position).unit_xy() if threat else Vec3(1.0, 0.0)
                self.phase = AvoidancePhase.MANEUVERING
                self.finish_at = now + self.config.maneuver_duration
                self.maneuvers += 1
                return self.planned_velocity or Vec3(-direction.y, direction.x) * self.config.lateral_speed
        elif self.phase is AvoidancePhase.MANEUVERING and self.finish_at is not None and now >= self.finish_at:
            self.phase = AvoidancePhase.COOLDOWN
            self.cooldown_until = now + self.config.cooldown
            return self.original_velocity or Vec3(0.0, 0.0)
        elif self.phase is AvoidancePhase.COOLDOWN and self.cooldown_until is not None and now >= self.cooldown_until:
            self.phase = AvoidancePhase.NORMAL
            self.threat = None
            self.planned_velocity = None
            self.planned_maneuver = None
        return None
