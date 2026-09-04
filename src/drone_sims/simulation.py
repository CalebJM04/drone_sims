from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
import random
from statistics import fmean
from typing import Any

from .avoidance import AvoidanceConfig, AvoidanceController, AvoidancePhase
from .collision import KinematicState, Vec3, assess, noisy_state
from .faults import SensorFault
from .planner import PlannerConfig, plan_maneuver
from .protocol import CrcError, ProtocolError, TelemetryFrame, decode, encode
from .radio import LinkOutage, RadioConfig, RadioMedium
from .routing import RoutingPolicy
from .state_table import NeighborTable
from .tracking import AlphaBetaTracker, assess_uncertain


@dataclass(slots=True)
class Node:
    node_id: int
    position: Vec3
    velocity: Vec3
    autonomous: bool = False
    sequence: int = 0
    boot_id: int = 1
    table: NeighborTable = field(default_factory=lambda: NeighborTable(16))
    seen: dict[tuple[int, int, int], float] = field(default_factory=dict)
    tracker: AlphaBetaTracker = field(default_factory=AlphaBetaTracker)
    controller: AvoidanceController | None = None
    max_acceleration: float | None = None
    target_velocity: Vec3 | None = None

    def state(self, now: float) -> KinematicState:
        return KinematicState(self.node_id, self.position, self.velocity, now)


@dataclass(slots=True)
class SimulationConfig:
    duration: float = 15.0
    step: float = 0.02
    telemetry_interval: float = 1.0
    risk_interval: float = 0.1
    safety_distance: float = 3.0
    horizon: float = 5.0
    max_state_age: float = 1.5
    neighbor_expiry: float = 6.0
    packet_ttl: int = 5
    position_noise_sigma: float = 0.0
    velocity_noise_sigma: float = 0.0
    clock_noise_sigma: float = 0.0
    tracked_pair: tuple[int, int] = (1, 2)
    seed: int = 1
    avoidance: AvoidanceConfig = field(default_factory=AvoidanceConfig)
    uncertainty_aware: bool = True
    uncertainty_sigma_multiplier: float = 2.5
    measurement_position_sigma: float = 0.20
    measurement_velocity_sigma: float = 0.08
    measurement_clock_sigma: float = 0.02
    use_planner: bool = True
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    routing: RoutingPolicy = field(default_factory=RoutingPolicy)
    sensor_faults: tuple[SensorFault, ...] = ()
    node_reboots: tuple[tuple[int, float], ...] = ()
    trace_interval: float = 0.1


@dataclass(slots=True)
class Counters:
    packets: int = 0
    attempts: int = 0
    deliveries: int = 0
    collisions: int = 0
    link_losses: int = 0
    crc_rejections: int = 0
    protocol_rejections: int = 0
    duplicates: int = 0
    expired_states: int = 0
    alarms: int = 0
    true_positive: int = 0
    false_positive: int = 0
    true_negative: int = 0
    false_negative: int = 0
    nominal_safe_conservative_risk: int = 0
    forwards_suppressed: int = 0
    telemetry_suppressed: int = 0
    node_reboots: int = 0


class Simulation:
    def __init__(
        self,
        nodes: list[Node],
        *,
        config: SimulationConfig | None = None,
        radio: RadioConfig | None = None,
        outages: list[LinkOutage] | None = None,
        event_logging: bool = True,
    ) -> None:
        self.nodes = {node.node_id: node for node in nodes}
        if len(self.nodes) != len(nodes):
            raise ValueError("node IDs must be unique")
        self.config = config or SimulationConfig()
        self.radio_config = radio or RadioConfig()
        self.rng = random.Random(self.config.seed)
        self.medium = RadioMedium(self.radio_config, self.rng, lambda node_id: self.nodes[node_id].position, outages)
        for node in nodes:
            self.medium.register(node.node_id)
            if node.autonomous and node.controller is None:
                node.controller = AvoidanceController(self.config.avoidance)
        self.events: list[dict[str, Any]] = []
        self.trace: list[dict[str, Any]] = []
        self.event_logging = event_logging
        self.counters = Counters()
        self.latencies: list[float] = []
        self.hops: list[int] = []
        self.minimum_separation = math.inf
        self.first_alarm: float | None = None
        self.first_maneuver: float | None = None
        self._last_phases = {node.node_id: node.controller.phase if node.controller else None for node in nodes}
        self._applied_reboots: set[tuple[int, float]] = set()

    def _log(self, now: float, kind: str, **details: Any) -> None:
        if self.event_logging:
            self.events.append({"time": round(now, 6), "type": kind, **details})

    def run(self) -> dict[str, Any]:
        now, next_risk, next_trace = 0.0, 0.0, 0.0
        ordered_nodes = sorted(self.nodes)
        next_telemetry = {
            node_id: index * self.config.telemetry_interval / len(ordered_nodes)
            for index, node_id in enumerate(ordered_nodes)
        }
        while now <= self.config.duration + 1e-9:
            self._apply_reboots(now)
            self._receive(now)
            self._update_controllers(now)
            if now < self.config.duration - 1e-9:
                for node_id in ordered_nodes:
                    if now + 1e-9 >= next_telemetry[node_id]:
                        self._originate(self.nodes[node_id], now)
                        next_telemetry[node_id] += self.config.telemetry_interval
            if now + 1e-9 >= next_risk:
                self._risk_checks(now)
                next_risk += self.config.risk_interval
            self._maintenance(now)
            self._track_separation()
            if self.event_logging and now + 1e-9 >= next_trace:
                self._record_trace(now)
                next_trace += self.config.trace_interval
            advance = min(self.config.step, self.config.duration - now)
            if advance <= 1e-12:
                break
            for node in self.nodes.values():
                if node.target_velocity is not None and node.max_acceleration is not None:
                    delta = node.target_velocity - node.velocity
                    delta_speed = delta.norm()
                    max_delta = node.max_acceleration * advance
                    node.velocity = node.target_velocity if delta_speed <= max_delta else node.velocity + delta * (max_delta / delta_speed)
                node.position = node.position + node.velocity * advance
            now = round(now + advance, 10)
        self._receive(self.config.duration)
        self.counters.attempts = self.medium.transmission_attempts
        return self.summary()

    def _record_trace(self, now: float) -> None:
        first, second = (self.nodes[node_id] for node_id in self.config.tracked_pair)
        self.trace.append({
            "time": round(now, 6),
            "separation_m": (first.position - second.position).norm(),
            "nodes": [
                {
                    "id": node.node_id,
                    "position": [node.position.x, node.position.y, node.position.z],
                    "velocity": [node.velocity.x, node.velocity.y, node.velocity.z],
                    "autonomous": node.autonomous,
                    "phase": node.controller.phase.value if node.controller else None,
                }
                for node in sorted(self.nodes.values(), key=lambda item: item.node_id)
            ],
        })

    def _originate(self, node: Node, now: float) -> None:
        faults = [fault for fault in self.config.sensor_faults if fault.active(node.node_id, now)]
        if any(fault.drop_telemetry for fault in faults):
            self.counters.telemetry_suppressed += 1
            self._log(now, "telemetry_suppressed", source=node.node_id)
            return
        state = noisy_state(
            node.state(now),
            self.rng,
            position_sigma=self.config.position_noise_sigma,
            velocity_sigma=self.config.velocity_noise_sigma,
            clock_sigma=self.config.clock_noise_sigma,
        )
        for fault in faults:
            state = KinematicState(
                state.node_id,
                state.position + fault.position_bias,
                state.velocity + fault.velocity_bias,
                state.timestamp + fault.clock_bias,
            )
        frame = TelemetryFrame.from_state(
            state, sequence=node.sequence, boot_id=node.boot_id, ttl=self.config.packet_ttl
        )
        node.sequence = (node.sequence + 1) & 0xFFFF
        node.seen[frame.packet_id] = now
        self.counters.packets += 1
        self.medium.broadcast(node.node_id, encode(frame), (node.node_id,), now)
        self._log(
            now, "packet_created", source=node.node_id,
            boot_id=frame.boot_id, sequence=frame.sequence,
        )

    def _receive(self, now: float) -> None:
        for reception in self.medium.poll(now):
            if reception.collided:
                self.counters.collisions += 1
                self._log(reception.ends_at, "mac_collision", sender=reception.sender, receiver=reception.receiver)
                continue
            if reception.lost:
                self.counters.link_losses += 1
                continue
            try:
                frame = decode(reception.data)
            except CrcError:
                self.counters.crc_rejections += 1
                continue
            except ProtocolError:
                self.counters.protocol_rejections += 1
                continue
            receiver = self.nodes[reception.receiver]
            if frame.packet_id in receiver.seen:
                self.counters.duplicates += 1
                continue
            receiver.seen[frame.packet_id] = reception.ends_at
            previous = receiver.table.entries.get(frame.source)
            accepted = receiver.table.update(frame, reception.ends_at)
            if not accepted:
                continue
            if previous is not None and previous.boot_id != frame.boot_id:
                receiver.tracker.tracks.pop(frame.source, None)
            receiver.tracker.update(
                frame.to_state(),
                reception.ends_at,
                position_sigma=max(self.config.measurement_position_sigma, self.config.position_noise_sigma),
                velocity_sigma=max(self.config.measurement_velocity_sigma, self.config.velocity_noise_sigma),
                clock_sigma=max(self.config.measurement_clock_sigma, self.config.clock_noise_sigma),
            )
            self.counters.deliveries += 1
            latency = reception.ends_at - frame.timestamp_ms / 1000.0
            self.latencies.append(latency)
            self.hops.append(frame.hops + 1)
            self._log(
                reception.ends_at,
                "packet_delivered",
                source=frame.source,
                receiver=receiver.node_id,
                boot_id=frame.boot_id,
                sequence=frame.sequence,
                path=list(reception.path),
                hops=frame.hops + 1,
                latency=round(latency, 6),
            )
            if frame.ttl > 0 and self.config.routing.should_forward(
                receiver.node_id, frame.source, frame.sequence
            ):
                forwarded = frame.forwarded()
                self.medium.broadcast(receiver.node_id, encode(forwarded), reception.path, reception.ends_at)
            elif frame.ttl > 0:
                self.counters.forwards_suppressed += 1

    def _apply_reboots(self, now: float) -> None:
        for node_id, reboot_at in self.config.node_reboots:
            marker = (node_id, reboot_at)
            if marker in self._applied_reboots or now + 1e-9 < reboot_at:
                continue
            node = self.nodes[node_id]
            node.sequence = 0
            node.boot_id = (node.boot_id + 1) & 0xFFFF or 1
            self._applied_reboots.add(marker)
            self.counters.node_reboots += 1
            self._log(now, "node_reboot", node=node_id, boot_id=node.boot_id)

    def _risk_checks(self, now: float) -> None:
        for node in self.nodes.values():
            if not node.autonomous or node.controller is None:
                continue
            best: tuple[float, KinematicState] | None = None
            perceived_threats: list[KinematicState] = []
            for peer_id, entry in node.table.entries.items():
                track = node.tracker.tracks.get(peer_id)
                if self.config.uncertainty_aware and track is not None:
                    uncertain = assess_uncertain(
                        node.state(now), track, now=now,
                        safety_distance=self.config.safety_distance,
                        horizon=self.config.horizon, max_age=self.config.neighbor_expiry,
                        sigma_multiplier=self.config.uncertainty_sigma_multiplier,
                    )
                    perceived = uncertain.conservative
                    perceived_state = track.state.at(now)
                    if not uncertain.nominal.risk and perceived.risk:
                        self.counters.nominal_safe_conservative_risk += 1
                else:
                    perceived = assess(
                        node.state(now), entry.state, now=now,
                        safety_distance=self.config.safety_distance,
                        horizon=self.config.horizon, max_age=self.config.max_state_age,
                    )
                    perceived_state = entry.state.at(now)
                perceived_threats.append(perceived_state)
                true = assess(
                    node.state(now), self.nodes[peer_id].state(now), now=now,
                    safety_distance=self.config.safety_distance,
                    horizon=self.config.horizon, max_age=0.0,
                )
                if perceived.risk and true.risk:
                    self.counters.true_positive += 1
                elif perceived.risk:
                    self.counters.false_positive += 1
                elif true.risk:
                    self.counters.false_negative += 1
                else:
                    self.counters.true_negative += 1
                if perceived.risk and (best is None or (perceived.tcpa or 0.0) < best[0]):
                    best = (perceived.tcpa or 0.0, perceived_state)
            if best and node.controller.phase is AvoidancePhase.NORMAL:
                threat = best[1]
                maneuver_velocity = None
                maneuver_name = None
                candidate_details: list[dict[str, Any]] = []
                if self.config.use_planner:
                    planner_config = PlannerConfig(
                        maneuver_speed=self.config.avoidance.lateral_speed,
                        vertical_speed=self.config.planner.vertical_speed,
                        horizon=self.config.horizon,
                        safety_distance=self.config.safety_distance,
                        min_z=self.config.planner.min_z,
                        max_z=self.config.planner.max_z,
                        deviation_penalty=self.config.planner.deviation_penalty,
                    )
                    chosen, candidates = plan_maneuver(node.state(now), perceived_threats, planner_config)
                    maneuver_velocity = chosen.velocity
                    maneuver_name = chosen.name
                    candidate_details = [
                        {"name": item.name, "separation": round(item.worst_separation, 3), "feasible": item.feasible}
                        for item in candidates
                    ]
                if node.controller.request(
                    node.state(now), threat, now, self.rng,
                    command_velocity=maneuver_velocity, maneuver_name=maneuver_name,
                ):
                    self.counters.alarms += 1
                    self.first_alarm = now if self.first_alarm is None else self.first_alarm
                    self._log(
                        now, "collision_alarm", node=node.node_id, threat=threat.node_id,
                        tcpa=best[0], maneuver=maneuver_name, candidates=candidate_details,
                    )

    def _update_controllers(self, now: float) -> None:
        for node in self.nodes.values():
            controller = node.controller
            if controller is None:
                continue
            threat_entry = node.table.entries.get(controller.threat) if controller.threat is not None else None
            threat = threat_entry.state.at(now) if threat_entry else None
            velocity = controller.update(node.state(now), threat, now)
            if velocity is not None:
                node.target_velocity = velocity
                if node.max_acceleration is None:
                    node.velocity = velocity
            old_phase = self._last_phases[node.node_id]
            if controller.phase is not old_phase:
                self._log(now, "avoidance_phase", node=node.node_id, before=old_phase.value, after=controller.phase.value)
                if controller.phase is AvoidancePhase.MANEUVERING and self.first_maneuver is None:
                    self.first_maneuver = now
                self._last_phases[node.node_id] = controller.phase

    def _maintenance(self, now: float) -> None:
        retention = max(self.config.neighbor_expiry, self.config.duration)
        for node in self.nodes.values():
            self.counters.expired_states += len(node.table.expire(now, self.config.neighbor_expiry))
            node.tracker.expire(now, self.config.neighbor_expiry)
            node.seen = {packet: seen_at for packet, seen_at in node.seen.items() if now - seen_at <= retention}

    def _track_separation(self) -> None:
        first, second = (self.nodes[node_id] for node_id in self.config.tracked_pair)
        self.minimum_separation = min(self.minimum_separation, (first.position - second.position).norm())

    def summary(self) -> dict[str, Any]:
        possible = self.counters.packets * max(0, len(self.nodes) - 1)
        maneuvers = sum(node.controller.maneuvers for node in self.nodes.values() if node.controller)
        failures = sum(node.controller.failures for node in self.nodes.values() if node.controller)
        classified = self.counters.true_positive + self.counters.false_positive + self.counters.true_negative + self.counters.false_negative
        return {
            "seed": self.config.seed,
            "nodes": len(self.nodes),
            "network": {
                "packets": self.counters.packets,
                "transmission_attempts": self.medium.transmission_attempts,
                "deliveries": self.counters.deliveries,
                "delivery_ratio": self.counters.deliveries / possible if possible else 0.0,
                "mean_latency_ms": fmean(self.latencies) * 1000.0 if self.latencies else None,
                "max_latency_ms": max(self.latencies) * 1000.0 if self.latencies else None,
                "max_hops": max(self.hops, default=0),
                "mac_collisions": self.counters.collisions,
                "link_losses": self.counters.link_losses,
                "crc_rejections": self.counters.crc_rejections,
                "duplicates": self.counters.duplicates,
                "expired_states": self.counters.expired_states,
                "duty_cycle_queue_ms": self.medium.queue_delay_seconds * 1000.0,
                "half_duplex_losses": self.medium.half_duplex_losses,
                "forwards_suppressed": self.counters.forwards_suppressed,
                "telemetry_suppressed": self.counters.telemetry_suppressed,
                "node_reboots": self.counters.node_reboots,
            },
            "prediction": {
                "true_positive_checks": self.counters.true_positive,
                "false_positive_checks": self.counters.false_positive,
                "true_negative_checks": self.counters.true_negative,
                "false_negative_checks": self.counters.false_negative,
                "uncertainty_only_risk_checks": self.counters.nominal_safe_conservative_risk,
                "check_accuracy": (self.counters.true_positive + self.counters.true_negative) / classified if classified else None,
                "alarms": self.counters.alarms,
            },
            "avoidance": {
                "maneuvers": maneuvers,
                "command_failures": failures,
                "first_alarm_s": self.first_alarm,
                "first_maneuver_s": self.first_maneuver,
                "reaction_time_s": self.first_maneuver - self.first_alarm if self.first_alarm is not None and self.first_maneuver is not None else None,
                "minimum_separation_m": self.minimum_separation,
            },
        }
