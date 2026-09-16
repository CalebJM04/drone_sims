from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Protocol

from .collision import KinematicState, Vec3
from .mavlink_codec import VelocityCommand
from .planner import PlannerConfig, plan_maneuver
from .protocol import CrcError, ProtocolError, TelemetryFrame, decode, encode
from .radio_types import RadioReception
from .routing import RoutingPolicy
from .state_table import NeighborTable
from .tracking import AlphaBetaTracker, assess_uncertain


class RadioPort(Protocol):
    def receive(self) -> list[bytes | RadioReception]: ...

    def send(self, payload: bytes) -> None: ...

    def close(self) -> None: ...


class FlightControllerPort(Protocol):
    def receive_state(self, timestamp_s: float) -> KinematicState | None: ...

    def send_velocity(self, command: VelocityCommand, time_boot_ms: int) -> None: ...

    def health_snapshot(self) -> dict[str, object]: ...

    def close(self) -> None: ...


@dataclass(frozen=True, slots=True)
class CompanionConfig:
    node_id: int
    boot_id: int
    mission_epoch_unix_s: float = 0.0
    telemetry_interval_s: float = 1.0
    risk_interval_s: float = 0.1
    command_interval_s: float = 0.1
    own_state_timeout_s: float = 0.5
    neighbor_expiry_s: float = 6.0
    safety_distance_m: float = 3.0
    horizon_s: float = 5.0
    uncertainty_sigma_multiplier: float = 2.5
    own_position_sigma_m: float = 0.7
    own_velocity_sigma_mps: float = 0.15
    measurement_position_sigma_m: float = 0.7
    measurement_velocity_sigma_mps: float = 0.15
    clock_sigma_s: float = 0.05
    maneuver_duration_s: float = 2.0
    maneuver_speed_mps: float = 3.0
    vertical_speed_mps: float = 1.5
    min_down_m: float = -20.0
    max_down_m: float = 20.0
    packet_ttl: int = 5
    table_capacity: int = 16
    control_enabled: bool = False
    collision_awareness_enabled: bool = True
    routing_mode: str = "flooding"
    relay_nodes: tuple[int, ...] = ()
    forwarding_probability: float = 0.65

    def __post_init__(self) -> None:
        if not 1 <= self.node_id <= 0xFFFF:
            raise ValueError("node_id must be between 1 and 65535")
        if not 1 <= self.boot_id <= 0xFFFF:
            raise ValueError("boot_id must be between 1 and 65535")
        if not math.isfinite(self.mission_epoch_unix_s) or self.mission_epoch_unix_s < 0:
            raise ValueError("mission_epoch_unix_s must be finite and non-negative")
        for name in (
            "telemetry_interval_s", "risk_interval_s", "command_interval_s",
            "own_state_timeout_s", "neighbor_expiry_s", "safety_distance_m",
            "horizon_s", "maneuver_duration_s", "maneuver_speed_mps",
            "uncertainty_sigma_multiplier", "vertical_speed_mps",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in (
            "own_position_sigma_m", "own_velocity_sigma_mps",
            "measurement_position_sigma_m", "measurement_velocity_sigma_mps",
            "clock_sigma_s",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if not math.isfinite(self.min_down_m) or not math.isfinite(self.max_down_m):
            raise ValueError("min_down_m and max_down_m must be finite")
        if self.min_down_m >= self.max_down_m:
            raise ValueError("min_down_m must be less than max_down_m")
        if not 0 <= self.packet_ttl <= 0xFF:
            raise ValueError("packet_ttl must fit in one byte")
        if self.table_capacity <= 0:
            raise ValueError("table_capacity must be positive")
        if self.routing_mode not in {"flooding", "relay", "probabilistic"}:
            raise ValueError("routing_mode must be flooding, relay, or probabilistic")
        if not 0.0 <= self.forwarding_probability <= 1.0:
            raise ValueError("forwarding_probability must be in [0, 1]")


@dataclass(slots=True)
class CompanionStats:
    telemetry_sent: int = 0
    received: int = 0
    forwarded: int = 0
    forwards_suppressed: int = 0
    duplicates: int = 0
    crc_rejections: int = 0
    protocol_rejections: int = 0
    stale_own_state_events: int = 0
    risk_events: int = 0
    commands_sent: int = 0


def global_to_local_ned(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    *,
    reference_latitude_deg: float,
    reference_longitude_deg: float,
    reference_altitude_m: float,
) -> Vec3:
    earth_radius_m = 6_378_137.0
    latitude_delta = math.radians(latitude_deg - reference_latitude_deg)
    longitude_delta_deg = (longitude_deg - reference_longitude_deg + 180.0) % 360.0 - 180.0
    longitude_delta = math.radians(longitude_delta_deg)
    north = earth_radius_m * latitude_delta
    east = earth_radius_m * longitude_delta * math.cos(math.radians(reference_latitude_deg))
    down = reference_altitude_m - altitude_m
    return Vec3(north, east, down)


class CompanionService:
    def __init__(
        self,
        config: CompanionConfig,
        flight_controller: FlightControllerPort,
        radio: RadioPort,
    ) -> None:
        self.config = config
        self.flight_controller = flight_controller
        self.radio = radio
        self.table = NeighborTable(config.table_capacity)
        self.tracker = AlphaBetaTracker()
        self.routing = RoutingPolicy(
            mode=config.routing_mode,
            relay_nodes=config.relay_nodes,
            forwarding_probability=config.forwarding_probability,
            seed=config.boot_id,
        )
        self.stats = CompanionStats()
        self.sequence = 0
        self.own_state: KinematicState | None = None
        self._own_received_at: float | None = None
        self._seen: dict[tuple[int, int, int], float] = {}
        self._next_telemetry = 0.0
        self._next_risk = 0.0
        self._next_command = 0.0
        self._active_command: VelocityCommand | None = None
        self._active_until = 0.0
        self._last_stale = True
        self._own_state_fresh = False
        self._collision_alerts: list[dict[str, object]] = []
        self._last_monotonic_s = 0.0
        self._last_mission_time_s = 0.0

    @property
    def active_command(self) -> VelocityCommand | None:
        return self._active_command

    def step(self, monotonic_s: float, mission_time_s: float) -> None:
        self._last_monotonic_s = monotonic_s
        self._last_mission_time_s = mission_time_s
        state = self.flight_controller.receive_state(mission_time_s)
        if state is not None:
            self.own_state = KinematicState(
                self.config.node_id, state.position, state.velocity, mission_time_s
            )
            self._own_received_at = monotonic_s

        for reception in self.radio.receive():
            self._ingest(reception, monotonic_s, mission_time_s)

        self.table.expire(monotonic_s, self.config.neighbor_expiry_s)
        self.tracker.expire(monotonic_s, self.config.neighbor_expiry_s)
        self._seen = {
            packet_id: seen_at for packet_id, seen_at in self._seen.items()
            if monotonic_s - seen_at <= self.config.neighbor_expiry_s
        }

        own_is_fresh = (
            self.own_state is not None
            and self._own_received_at is not None
            and monotonic_s - self._own_received_at <= self.config.own_state_timeout_s
        )
        self._own_state_fresh = own_is_fresh
        if not own_is_fresh:
            self._active_command = None
            self._active_until = 0.0
            self._collision_alerts = []
            if not self._last_stale:
                self.stats.stale_own_state_events += 1
            self._last_stale = True
            return
        self._last_stale = False
        assert self.own_state is not None
        own_now = self.own_state.at(mission_time_s)

        if monotonic_s >= self._next_telemetry:
            flags = 1 if self._active_command is not None else 0
            frame = TelemetryFrame.from_state(
                own_now,
                sequence=self.sequence,
                boot_id=self.config.boot_id,
                ttl=self.config.packet_ttl,
                flags=flags,
            )
            self.sequence = (self.sequence + 1) & 0xFFFF
            self._seen[frame.packet_id] = monotonic_s
            self.radio.send(encode(frame))
            self.stats.telemetry_sent += 1
            self._next_telemetry = monotonic_s + self.config.telemetry_interval_s

        if monotonic_s >= self._next_risk:
            self._evaluate_risk(own_now, monotonic_s, mission_time_s)
            self._next_risk = monotonic_s + self.config.risk_interval_s

        if self._active_command is not None and monotonic_s >= self._active_until:
            self._active_command = None

        if (
            self.config.control_enabled
            and monotonic_s >= self._next_command
        ):
            command = self._active_command or VelocityCommand(0.0, 0.0, 0.0)
            self.flight_controller.send_velocity(command, round(monotonic_s * 1000) & 0xFFFFFFFF)
            self.stats.commands_sent += 1
            self._next_command = monotonic_s + self.config.command_interval_s

    def _ingest(
        self,
        reception: bytes | RadioReception,
        received_at: float,
        mission_time_s: float,
    ) -> None:
        payload = reception.payload if isinstance(reception, RadioReception) else reception
        try:
            frame = decode(payload)
        except CrcError:
            self.stats.crc_rejections += 1
            return
        except ProtocolError:
            self.stats.protocol_rejections += 1
            return
        if frame.source == self.config.node_id:
            return
        if frame.packet_id in self._seen:
            self.stats.duplicates += 1
            return
        self._seen[frame.packet_id] = received_at
        previous = self.table.entries.get(frame.source)
        if not self.table.update(frame, received_at):
            return
        if previous is not None and previous.boot_id != frame.boot_id:
            self.tracker.tracks.pop(frame.source, None)
        self.tracker.update(
            frame.to_state(),
            received_at,
            position_sigma=self.config.measurement_position_sigma_m,
            velocity_sigma=self.config.measurement_velocity_sigma_mps,
            clock_sigma=self.config.clock_sigma_s,
        )
        self.stats.received += 1
        if frame.ttl > 0 and self.routing.should_forward(
            self.config.node_id,
            frame.source,
            frame.sequence,
            now=mission_time_s,
        ):
            self.radio.send(encode(frame.forwarded()))
            self.stats.forwarded += 1
        elif frame.ttl > 0:
            self.stats.forwards_suppressed += 1

    def _evaluate_risk(
        self,
        own: KinematicState,
        monotonic_s: float,
        mission_time_s: float,
    ) -> None:
        if not self.config.collision_awareness_enabled:
            self._collision_alerts = []
            return
        threats: list[KinematicState] = []
        risk_found = False
        alerts: list[dict[str, object]] = []
        for track in self.tracker.tracks.values():
            assessment = assess_uncertain(
                own,
                track,
                now=mission_time_s,
                safety_distance=self.config.safety_distance_m,
                horizon=self.config.horizon_s,
                max_age=self.config.neighbor_expiry_s,
                sigma_multiplier=self.config.uncertainty_sigma_multiplier,
                own_position_sigma=self.config.own_position_sigma_m,
                own_velocity_sigma=self.config.own_velocity_sigma_mps,
            )
            if assessment.nominal.reason in {"stale", "future"}:
                continue
            threats.append(track.state.at(mission_time_s))
            risk_found = risk_found or assessment.risk
            severity = "danger" if assessment.risk else (
                "watch" if assessment.nominal.tcpa is not None
                and 0.0 <= assessment.nominal.tcpa <= self.config.horizon_s
                and assessment.nominal.dcpa < self.config.safety_distance_m * 2.0
                else "clear"
            )
            alerts.append({
                "peer_id": track.state.node_id,
                "risk": assessment.risk,
                "severity": severity,
                "current_distance_m": assessment.nominal.current_distance,
                "closest_distance_m": assessment.nominal.dcpa,
                "time_to_closest_s": assessment.nominal.tcpa,
                "uncertainty_margin_m": assessment.uncertainty_margin,
                "reason": assessment.conservative.reason,
            })
        self._collision_alerts = sorted(
            alerts,
            key=lambda value: (
                not bool(value["risk"]),
                float(value["closest_distance_m"]),
            ),
        )
        if not risk_found:
            return
        planner = PlannerConfig(
            maneuver_speed=self.config.maneuver_speed_mps,
            vertical_speed=self.config.vertical_speed_mps,
            horizon=self.config.horizon_s,
            safety_distance=self.config.safety_distance_m,
            min_z=self.config.min_down_m,
            max_z=self.config.max_down_m,
        )
        chosen, _ = plan_maneuver(own, threats, planner)
        self._active_command = VelocityCommand(
            chosen.velocity.x, chosen.velocity.y, chosen.velocity.z
        )
        self._active_until = monotonic_s + self.config.maneuver_duration_s
        self.stats.risk_events += 1

    def snapshot(self) -> dict[str, object]:
        return {
            "node_id": self.config.node_id,
            "control_enabled": self.config.control_enabled,
            "own_state_available": self.own_state is not None,
            "own_state_fresh": self._own_state_fresh,
            "neighbors": len(self.table.entries),
            "routing_mode": self.config.routing_mode,
            "collision_alerts": self._collision_alerts,
            "active_command": asdict(self._active_command) if self._active_command else None,
            "flight_controller": self.flight_controller.health_snapshot(),
            "stats": asdict(self.stats),
        }

    def close(self) -> None:
        self.radio.close()
        self.flight_controller.close()
