from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import multiprocessing
from multiprocessing.connection import Connection
import os
from pathlib import Path
import random
import time
from typing import Callable

from .collision import KinematicState, Vec3
from .companion import CompanionConfig, CompanionService
from .mavlink_codec import VelocityCommand
from .network_awareness import planned_routes, serialize_links, topology_links
from .protocol import ProtocolError, decode
from .provenance import collect_metadata
from .radio import RadioConfig, RadioMedium
from .radio_types import RadioReception


MAX_DEMO_NODES = 16


@dataclass(frozen=True, slots=True)
class DemoRequirements:
    minimum_nodes: int = 4
    minimum_delivery_ratio: float = 0.90
    maximum_p95_latency_ms: float = 1500.0
    maximum_state_age_s: float = 3.0
    require_preemptive_route: bool = True
    require_collision_alert: bool = True
    maximum_control_commands: int = 0


class TrajectoryFlightController:
    def __init__(self, initial: KinematicState) -> None:
        self.initial = initial
        self.commands: list[tuple[VelocityCommand, int]] = []

    def receive_state(self, timestamp_s: float) -> KinematicState:
        return self.initial.at(timestamp_s)

    def send_velocity(self, command: VelocityCommand, time_boot_ms: int) -> None:
        self.commands.append((command, time_boot_ms))

    def health_snapshot(self) -> dict[str, object]:
        return {"adapter": "simulated", "gps_fix_type": 6, "gps_healthy": True}

    def close(self) -> None:
        pass


class MeshRadioHub:
    def __init__(
        self,
        node_ids: list[int],
        position: Callable[[int], Vec3],
        *,
        seed: int,
        config: RadioConfig,
    ) -> None:
        self.now = 0.0
        self.config = config
        self.medium = RadioMedium(config, random.Random(seed), position)
        self.inboxes: dict[int, list[RadioReception]] = {node: [] for node in node_ids}
        for node in node_ids:
            self.medium.register(node)
        self.originated: set[tuple[int, int, int]] = set()
        self.delivered: set[tuple[int, tuple[int, int, int]]] = set()
        self.last_delivery: dict[tuple[int, int], float] = {}
        self.latencies: list[float] = []
        self.hops: list[int] = []
        self.transmissions: list[dict[str, int | float]] = []
        self.corrupt_frames_rejected = 0
        self.undetected_corruptions = 0

    def transmit(self, sender: int, payload: bytes) -> None:
        try:
            frame = decode(payload)
        except ProtocolError:
            return
        if frame.hops == 0 and frame.source == sender:
            self.originated.add(frame.packet_id)
        self.transmissions.append({
            "time_s": self.now,
            "sender": sender,
            "source": frame.source,
            "sequence": frame.sequence,
            "hops": frame.hops,
        })
        self.medium.broadcast(sender, payload, (sender,), self.now)

    def deliver(self, now: float) -> None:
        self.now = now
        for reception in self.medium.poll(now):
            if reception.collided or reception.lost:
                continue
            try:
                frame = decode(reception.data)
            except ProtocolError:
                self.corrupt_frames_rejected += 1
                self.inboxes[reception.receiver].append(
                    RadioReception(
                        reception.data,
                        sender_id=reception.sender,
                        rssi_dbm=reception.received_power_dbm,
                    )
                )
                continue
            if reception.corrupted:
                self.undetected_corruptions += 1
            if reception.receiver == frame.source:
                continue
            key = (reception.receiver, frame.packet_id)
            if key not in self.delivered:
                self.delivered.add(key)
                self.last_delivery[(reception.receiver, frame.source)] = reception.ends_at
                self.latencies.append(reception.ends_at - frame.timestamp_ms / 1000.0)
                self.hops.append(frame.hops + 1)
            snr = reception.received_power_dbm - self.config.receiver_sensitivity_dbm - 8.0
            self.inboxes[reception.receiver].append(
                RadioReception(
                    reception.data,
                    sender_id=reception.sender,
                    rssi_dbm=reception.received_power_dbm,
                    snr_db=snr,
                )
            )

    def radio(self, node_id: int) -> "HubRadio":
        return HubRadio(self, node_id)


class HubRadio:
    def __init__(self, hub: MeshRadioHub, node_id: int) -> None:
        self.hub = hub
        self.node_id = node_id

    def receive(self) -> list[RadioReception]:
        packets, self.hub.inboxes[self.node_id] = self.hub.inboxes[self.node_id], []
        return packets

    def send(self, payload: bytes) -> None:
        self.hub.transmit(self.node_id, payload)

    def close(self) -> None:
        pass


class BufferedRadio:
    def __init__(self) -> None:
        self.inbox: list[RadioReception] = []
        self.outbox: list[bytes] = []

    def receive(self) -> list[RadioReception]:
        packets, self.inbox = self.inbox, []
        return packets

    def send(self, payload: bytes) -> None:
        self.outbox.append(payload)

    def close(self) -> None:
        pass


def _node_process(
    connection: Connection,
    initial: KinematicState,
    config: CompanionConfig,
) -> None:
    controller = TrajectoryFlightController(initial)
    radio = BufferedRadio()
    service = CompanionService(config, controller, radio)
    try:
        connection.send(("ready", os.getpid()))
        while True:
            message = connection.recv()
            operation = message[0]
            if operation == "step":
                _, now, packets, capture = message
                radio.inbox.extend(packets)
                service.step(now, now)
                outgoing, radio.outbox = radio.outbox, []
                snapshot = service.snapshot() if capture else None
                connection.send(("step", outgoing, snapshot, len(controller.commands)))
            elif operation == "suspend":
                service._next_telemetry = math.inf
                connection.send(("suspended",))
            elif operation == "stop":
                connection.send(("stopped", service.snapshot(), len(controller.commands)))
                break
            else:
                raise ValueError(f"unknown node operation: {operation}")
    except (EOFError, BrokenPipeError):
        pass
    finally:
        service.close()
        connection.close()


class InlineNodeRuntime:
    def __init__(
        self,
        trajectories: dict[int, KinematicState],
        configs: dict[int, CompanionConfig],
        hub: MeshRadioHub,
    ) -> None:
        self.controllers = {
            node: TrajectoryFlightController(state) for node, state in trajectories.items()
        }
        self.services = {
            node: CompanionService(configs[node], self.controllers[node], hub.radio(node))
            for node in trajectories
        }

    @property
    def process_ids(self) -> list[int]:
        return [os.getpid()]

    def step(
        self,
        hub: MeshRadioHub,
        now: float,
        *,
        capture: bool,
    ) -> dict[str, dict[str, object]] | None:
        for service in self.services.values():
            service.step(now, now)
        if not capture:
            return None
        return {str(node): service.snapshot() for node, service in self.services.items()}

    def suspend_telemetry(self) -> None:
        for service in self.services.values():
            service._next_telemetry = math.inf

    def close(self) -> tuple[dict[str, dict[str, object]], int]:
        snapshots = {str(node): service.snapshot() for node, service in self.services.items()}
        commands = sum(len(controller.commands) for controller in self.controllers.values())
        for service in self.services.values():
            service.close()
        return snapshots, commands


class ProcessNodeRuntime:
    def __init__(
        self,
        trajectories: dict[int, KinematicState],
        configs: dict[int, CompanionConfig],
    ) -> None:
        context = multiprocessing.get_context("spawn")
        self.connections: dict[int, Connection] = {}
        self.processes: dict[int, multiprocessing.Process] = {}
        self.command_counts: dict[int, int] = {}
        self._process_ids: list[int] = []
        for node, initial in trajectories.items():
            parent, child = context.Pipe()
            process = context.Process(
                target=_node_process,
                args=(child, initial, configs[node]),
                name=f"drone-node-{node}",
            )
            process.daemon = True
            process.start()
            child.close()
            self.connections[node] = parent
            self.processes[node] = process
        for node in trajectories:
            message = self._receive(node)
            if message[0] != "ready":
                raise RuntimeError(f"node {node} did not start correctly")
            self._process_ids.append(int(message[1]))

    @property
    def process_ids(self) -> list[int]:
        return self._process_ids

    def _receive(self, node: int) -> tuple[object, ...]:
        connection = self.connections[node]
        if not connection.poll(30.0):
            process = self.processes[node]
            raise TimeoutError(
                f"node {node} did not respond; exit code is {process.exitcode}"
            )
        try:
            return connection.recv()
        except EOFError as error:
            raise RuntimeError(f"node {node} exited unexpectedly") from error

    def step(
        self,
        hub: MeshRadioHub,
        now: float,
        *,
        capture: bool,
    ) -> dict[str, dict[str, object]] | None:
        for node, connection in self.connections.items():
            packets, hub.inboxes[node] = hub.inboxes[node], []
            connection.send(("step", now, packets, capture))
        snapshots: dict[str, dict[str, object]] = {}
        outgoing: list[tuple[int, bytes]] = []
        for node in self.connections:
            message = self._receive(node)
            if message[0] != "step":
                raise RuntimeError(f"unexpected response from node {node}: {message[0]}")
            outgoing.extend((node, payload) for payload in message[1])
            if message[2] is not None:
                snapshots[str(node)] = message[2]
            self.command_counts[node] = int(message[3])
        for node, payload in outgoing:
            hub.transmit(node, payload)
        return snapshots if capture else None

    def suspend_telemetry(self) -> None:
        for connection in self.connections.values():
            connection.send(("suspend",))
        for node in self.connections:
            if self._receive(node)[0] != "suspended":
                raise RuntimeError(f"node {node} did not suspend telemetry")

    def close(self) -> tuple[dict[str, dict[str, object]], int]:
        for node, connection in self.connections.items():
            if self.processes[node].is_alive():
                connection.send(("stop",))
        snapshots: dict[str, dict[str, object]] = {}
        for node in self.connections:
            if self.processes[node].is_alive():
                message = self._receive(node)
                if message[0] == "stopped":
                    snapshots[str(node)] = message[1]
                    self.command_counts[node] = int(message[2])
        deadline = time.monotonic() + 5.0
        for process in self.processes.values():
            process.join(timeout=max(0.0, deadline - time.monotonic()))
        for process in self.processes.values():
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
        for connection in self.connections.values():
            connection.close()
        return snapshots, sum(self.command_counts.values())


def demo_trajectories(nodes: int) -> dict[int, KinematicState]:
    if not 4 <= nodes <= MAX_DEMO_NODES:
        raise ValueError(f"mesh demo requires 4 to {MAX_DEMO_NODES} nodes")
    base = {
        1: KinematicState(1, Vec3(0.0, 0.0), Vec3(0.0, 0.0), 0.0),
        2: KinematicState(2, Vec3(20.0, 0.0), Vec3(1.5, 0.0), 0.0),
        3: KinematicState(3, Vec3(11.0, 12.0), Vec3(0.0, 0.0), 0.0),
        4: KinematicState(4, Vec3(28.0, 12.0), Vec3(0.0, 0.0), 0.0),
        5: KinematicState(5, Vec3(6.0, -8.0), Vec3(0.0, 1.0), 0.0),
        6: KinematicState(6, Vec3(6.0, 8.0), Vec3(0.0, -1.0), 0.0),
        7: KinematicState(7, Vec3(18.0, 18.0), Vec3(0.0, -0.2), 0.0),
        8: KinematicState(8, Vec3(35.0, 5.0), Vec3(0.2, 0.1), 0.0),
    }
    for node in range(9, nodes + 1):
        index = node - 9
        row, column = divmod(index, 8)
        base[node] = KinematicState(
            node,
            Vec3(float(column * 8), float(24 + row * 8)),
            Vec3(0.0, 0.0),
            0.0,
        )
    return {node: base[node] for node in range(1, nodes + 1)}


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(percentile * len(ordered)) - 1)
    return ordered[index]


def load_requirements(path: str | Path | None = None) -> DemoRequirements:
    if path is None:
        path = Path("requirements/network_demo_acceptance.json")
    source = Path(path)
    if not source.exists():
        return DemoRequirements()
    return DemoRequirements(**json.loads(source.read_text(encoding="utf-8")))


def run_mesh_demo(
    *,
    nodes: int = 6,
    duration_s: float = 15.0,
    seed: int = 31,
    realtime: bool = False,
    backend: str = "inline",
    telemetry_interval_s: float = 1.0,
    requirements: DemoRequirements | None = None,
    update: Callable[[dict[str, object]], None] | None = None,
) -> dict[str, object]:
    if not 4 <= nodes <= MAX_DEMO_NODES:
        raise ValueError(f"nodes must be between 4 and {MAX_DEMO_NODES}")
    if duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if telemetry_interval_s <= 0:
        raise ValueError("telemetry_interval_s must be positive")
    if backend not in {"inline", "process"}:
        raise ValueError("backend must be inline or process")
    requirements = requirements or load_requirements()
    trajectories = demo_trajectories(nodes)
    radio_config = RadioConfig(
        range_m=30.0,
        random_loss=0.002,
        bit_corruption=0.0005,
        burst_enter=0.001,
        max_backoff_s=0.08,
        channel_access="csma",
    )
    hub = MeshRadioHub(
        list(trajectories),
        lambda node: trajectories[node].at(hub.now).position,
        seed=seed,
        config=radio_config,
    )
    configs = {
        node: CompanionConfig(
            node,
            1000 + node,
            telemetry_interval_s=telemetry_interval_s,
            risk_interval_s=0.1,
            safety_distance_m=3.0,
            horizon_s=6.0,
            packet_ttl=max(5, math.ceil(math.sqrt(nodes))),
            table_capacity=max(16, nodes),
            control_enabled=False,
            collision_awareness_enabled=True,
            routing_mode="proactive",
            nominal_radio_range_m=radio_config.range_m,
            route_lookahead_s=4.0,
            route_margin=0.90,
            link_warning_margin_m=6.0,
            routing_discovery_interval_packets=max(2, nodes // 8),
        )
        for node in trajectories
    }
    runtime: InlineNodeRuntime | ProcessNodeRuntime
    if backend == "process":
        runtime = ProcessNodeRuntime(trajectories, configs)
    else:
        runtime = InlineNodeRuntime(trajectories, configs, hub)
    snapshots: list[dict[str, object]] = []
    events: list[dict[str, object]] = []
    preemptive_seen = False
    collision_seen = False
    maximum_state_age = 0.0
    started = time.monotonic()
    step = 0.05
    next_snapshot = 0.0
    now = 0.0
    while now < duration_s - 1e-9:
        if realtime:
            target = started + now
            remaining = target - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        hub.deliver(now)
        capture = now + 1e-9 >= next_snapshot
        node_snapshots = runtime.step(hub, now, capture=capture)
        if capture:
            assert node_snapshots is not None
            states = {node: state.at(now) for node, state in trajectories.items()}
            links = topology_links(
                states,
                now=now,
                range_m=radio_config.range_m,
                lookahead_s=4.0,
                warning_margin_m=6.0,
            )
            current_routes = planned_routes(
                states,
                source=1,
                now=now,
                range_m=radio_config.range_m,
                lookahead_s=0.0,
                route_margin=1.0,
            )
            future_routes = planned_routes(
                states,
                source=1,
                now=now,
                range_m=radio_config.range_m,
                lookahead_s=4.0,
                route_margin=0.90,
            )
            current_to_two = current_routes.get(2)
            future_to_two = future_routes.get(2)
            actual_relays = sorted({
                int(item["sender"])
                for item in hub.transmissions
                if item["source"] == 1
                and item["sender"] != 1
                and int(item["sequence"]) % 2 != 0
                and now - 1.5 <= float(item["time_s"]) <= now
            })
            if (
                not preemptive_seen
                and current_to_two == [1, 2]
                and future_to_two is not None
                and len(future_to_two) > 2
                and any(relay in future_to_two[1:-1] for relay in actual_relays)
            ):
                preemptive_seen = True
                events.append({
                    "time_s": now,
                    "type": "preemptive_route_handoff",
                    "source": 1,
                    "target": 2,
                    "current_path": current_to_two,
                    "planned_path": future_to_two,
                    "observed_relay_transmitters": actual_relays,
                })
            active_alerts = [
                {"node_id": int(node), **alert}
                for node, snapshot in node_snapshots.items()
                for alert in snapshot["collision_alerts"]
                if alert["risk"]
            ]
            if active_alerts and not collision_seen:
                collision_seen = True
                events.append({
                    "time_s": now,
                    "type": "collision_awareness_alert",
                    "alerts": active_alerts,
                })
            for receiver in trajectories:
                for source in trajectories:
                    if receiver == source:
                        continue
                    last = hub.last_delivery.get((receiver, source))
                    if last is not None:
                        maximum_state_age = max(maximum_state_age, now - last)
            possible_so_far = len(hub.originated) * (nodes - 1)
            state = {
                "time_s": round(now, 3),
                "backend": backend,
                "node_processes": len(runtime.process_ids),
                "source_channel_load": (
                    nodes * radio_config.modem.airtime_seconds(36)
                    / telemetry_interval_s
                ),
                "nodes": [
                    {
                        "id": node,
                        "position": value.position.as_list(),
                        "velocity": value.velocity.as_list(),
                    }
                    for node, value in sorted(states.items())
                ],
                "links": serialize_links(links),
                "route_1_to_2": future_to_two,
                "delivery_ratio": len(hub.delivered) / possible_so_far if possible_so_far else 0.0,
                "alerts": active_alerts,
                "node_health": {
                    node: {
                        "neighbors": snapshot["neighbors"],
                        "link_quality": snapshot["link_quality"],
                        "stats": snapshot["stats"],
                    }
                    for node, snapshot in node_snapshots.items()
                },
            }
            snapshots.append(state)
            if update is not None:
                update(state)
            next_snapshot += 0.5
        now = round(now + step, 10)

    runtime.suspend_telemetry()
    drain_until = duration_s + 2.0
    while now <= drain_until + 1e-9:
        hub.deliver(now)
        runtime.step(hub, now, capture=False)
        now = round(now + step, 10)

    final_nodes, control_commands = runtime.close()
    possible = len(hub.originated) * (nodes - 1)
    delivery_ratio = len(hub.delivered) / possible if possible else 0.0
    p95_latency_ms = (_percentile(hub.latencies, 0.95) or 0.0) * 1000.0
    channel_capacity_pps = 1.0 / radio_config.modem.airtime_seconds(36)
    offered_load_pps = nodes / telemetry_interval_s
    checks = {
        "minimum_nodes": nodes >= requirements.minimum_nodes,
        "delivery_ratio": delivery_ratio >= requirements.minimum_delivery_ratio,
        "p95_latency": p95_latency_ms <= requirements.maximum_p95_latency_ms,
        "state_age": maximum_state_age <= requirements.maximum_state_age_s,
        "preemptive_route": preemptive_seen or not requirements.require_preemptive_route,
        "collision_awareness": collision_seen or not requirements.require_collision_alert,
        "observation_only": control_commands <= requirements.maximum_control_commands,
        "protocol_integrity": hub.undetected_corruptions == 0,
    }
    report: dict[str, object] = {
        "metadata": collect_metadata(
            "drone-sims mesh-demo",
            seeds=[seed],
            parameters={
                "nodes": nodes,
                "duration_s": duration_s,
                "realtime": realtime,
                "backend": backend,
                "telemetry_interval_s": telemetry_interval_s,
            },
        ),
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "requirements": asdict(requirements),
        "summary": {
            "nodes": nodes,
            "backend": backend,
            "node_processes": len(runtime.process_ids),
            "process_ids": runtime.process_ids,
            "duration_s": duration_s,
            "telemetry_interval_s": telemetry_interval_s,
            "offered_load_pps": offered_load_pps,
            "single_channel_capacity_pps": channel_capacity_pps,
            "source_channel_load": offered_load_pps / channel_capacity_pps,
            "originated_packets": len(hub.originated),
            "unique_node_deliveries": len(hub.delivered),
            "delivery_ratio": delivery_ratio,
            "p95_latency_ms": p95_latency_ms,
            "maximum_state_age_s": maximum_state_age,
            "transmission_attempts": hub.medium.transmission_attempts,
            "half_duplex_losses": hub.medium.half_duplex_losses,
            "maximum_hops": max(hub.hops, default=0),
            "preemptive_route_handoffs": sum(
                event["type"] == "preemptive_route_handoff" for event in events
            ),
            "collision_awareness_alerts": sum(
                event["type"] == "collision_awareness_alert" for event in events
            ),
            "control_commands": control_commands,
            "corrupt_frames_rejected": hub.corrupt_frames_rejected,
            "undetected_corruptions": hub.undetected_corruptions,
        },
        "events": events,
        "snapshots": snapshots,
        "final_nodes": final_nodes,
    }
    return report
