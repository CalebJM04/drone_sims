from __future__ import annotations

from dataclasses import dataclass, field

from .collision import Vec3


@dataclass(frozen=True, slots=True)
class SensorFault:
    node_id: int
    start: float
    end: float
    position_bias: Vec3 = field(default_factory=lambda: Vec3(0, 0, 0))
    velocity_bias: Vec3 = field(default_factory=lambda: Vec3(0, 0, 0))
    clock_bias: float = 0.0
    drop_telemetry: bool = False

    def active(self, node_id: int, now: float) -> bool:
        return self.node_id == node_id and self.start <= now < self.end
