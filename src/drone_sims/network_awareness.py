from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import math
from typing import Iterable

from .collision import KinematicState


@dataclass(frozen=True, slots=True)
class LinkPrediction:
    a: int
    b: int
    distance_m: float
    projected_distance_m: float
    margin_m: float
    time_to_loss_s: float | None
    connected_now: bool
    connected_projected: bool
    status: str


def _time_to_range_boundary(
    first: KinematicState,
    second: KinematicState,
    range_m: float,
) -> float | None:
    relative_position = second.position - first.position
    relative_velocity = second.velocity - first.velocity
    a = relative_velocity.dot(relative_velocity)
    b = 2.0 * relative_position.dot(relative_velocity)
    c = relative_position.dot(relative_position) - range_m * range_m
    if a <= 1e-12:
        return None
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0:
        return None
    roots = [
        value
        for value in (
            (-b - math.sqrt(discriminant)) / (2.0 * a),
            (-b + math.sqrt(discriminant)) / (2.0 * a),
        )
        if value >= 0.0
    ]
    if not roots:
        return None
    return max(roots) if c <= 0.0 else None


def predict_link(
    first: KinematicState,
    second: KinematicState,
    *,
    now: float,
    range_m: float,
    lookahead_s: float,
    warning_margin_m: float,
) -> LinkPrediction:
    first_now = first.at(now)
    second_now = second.at(now)
    distance = (first_now.position - second_now.position).norm()
    projected = (
        first_now.at(now + lookahead_s).position
        - second_now.at(now + lookahead_s).position
    ).norm()
    connected_now = distance <= range_m
    connected_projected = projected <= range_m
    margin = range_m - distance
    time_to_loss = _time_to_range_boundary(first_now, second_now, range_m)
    if not connected_now:
        status = "out_of_range"
    elif not connected_projected:
        status = "critical"
    elif margin <= warning_margin_m or projected > distance + warning_margin_m:
        status = "degrading"
    else:
        status = "healthy"
    return LinkPrediction(
        min(first.node_id, second.node_id),
        max(first.node_id, second.node_id),
        distance,
        projected,
        margin,
        time_to_loss,
        connected_now,
        connected_projected,
        status,
    )


def topology_links(
    states: dict[int, KinematicState],
    *,
    now: float,
    range_m: float,
    lookahead_s: float,
    warning_margin_m: float,
) -> list[LinkPrediction]:
    ordered = sorted(states)
    return [
        predict_link(
            states[first], states[second], now=now, range_m=range_m,
            lookahead_s=lookahead_s, warning_margin_m=warning_margin_m,
        )
        for index, first in enumerate(ordered)
        for second in ordered[index + 1 :]
    ]


def adjacency(
    states: dict[int, KinematicState],
    *,
    now: float,
    range_m: float,
    lookahead_s: float = 0.0,
) -> dict[int, set[int]]:
    graph = {node_id: set() for node_id in states}
    projected = {node_id: state.at(now + lookahead_s) for node_id, state in states.items()}
    ordered = sorted(projected)
    for index, first in enumerate(ordered):
        for second in ordered[index + 1 :]:
            if (projected[first].position - projected[second].position).norm() <= range_m:
                graph[first].add(second)
                graph[second].add(first)
    return graph


def shortest_path(graph: dict[int, set[int]], source: int, target: int) -> list[int] | None:
    if source == target:
        return [source]
    if source not in graph or target not in graph:
        return None
    pending: deque[list[int]] = deque([[source]])
    visited = {source}
    while pending:
        path = pending.popleft()
        for neighbor in sorted(graph[path[-1]]):
            if neighbor in visited:
                continue
            candidate = path + [neighbor]
            if neighbor == target:
                return candidate
            visited.add(neighbor)
            pending.append(candidate)
    return None


def planned_routes(
    states: dict[int, KinematicState],
    *,
    source: int,
    now: float,
    range_m: float,
    lookahead_s: float,
    route_margin: float,
) -> dict[int, list[int]]:
    trusted_range = range_m * route_margin
    graph = adjacency(
        states, now=now, range_m=trusted_range, lookahead_s=lookahead_s
    )
    return {
        target: path
        for target in sorted(states)
        if target != source
        and (path := shortest_path(graph, source, target)) is not None
    }


def proactive_forwarders(
    states: dict[int, KinematicState],
    *,
    source: int,
    now: float,
    range_m: float,
    lookahead_s: float,
    route_margin: float,
) -> set[int]:
    routes = planned_routes(
        states,
        source=source,
        now=now,
        range_m=range_m,
        lookahead_s=lookahead_s,
        route_margin=route_margin,
    )
    return {
        relay
        for path in routes.values()
        for relay in path[1:-1]
    }


@dataclass(slots=True)
class LinkQuality:
    sender_id: int
    first_seen_s: float
    last_seen_s: float
    received: int = 0
    expected: int = 0
    rssi_dbm: float | None = None
    snr_db: float | None = None

    @property
    def delivery_ratio(self) -> float:
        return self.received / self.expected if self.expected else 0.0


class LinkQualityTable:
    def __init__(self, telemetry_interval_s: float, alpha: float = 0.25) -> None:
        self.telemetry_interval_s = telemetry_interval_s
        self.alpha = alpha
        self.entries: dict[int, LinkQuality] = {}
        self._seen: dict[tuple[int, tuple[int, int, int]], float] = {}

    def observe(
        self,
        sender_id: int,
        packet_id: tuple[int, int, int],
        now: float,
        *,
        rssi_dbm: float | None = None,
        snr_db: float | None = None,
    ) -> None:
        marker = (sender_id, packet_id)
        if marker in self._seen:
            return
        self._seen[marker] = now
        entry = self.entries.get(sender_id)
        if entry is None:
            entry = LinkQuality(sender_id, now, now)
            self.entries[sender_id] = entry
            intervals = 1
        else:
            intervals = max(1, round((now - entry.last_seen_s) / self.telemetry_interval_s))
        entry.received += 1
        entry.expected += intervals
        entry.last_seen_s = now
        if rssi_dbm is not None:
            entry.rssi_dbm = rssi_dbm if entry.rssi_dbm is None else (
                self.alpha * rssi_dbm + (1.0 - self.alpha) * entry.rssi_dbm
            )
        if snr_db is not None:
            entry.snr_db = snr_db if entry.snr_db is None else (
                self.alpha * snr_db + (1.0 - self.alpha) * entry.snr_db
            )

    def expire(self, now: float, max_age_s: float) -> None:
        expired = [node for node, value in self.entries.items() if now - value.last_seen_s > max_age_s]
        for node in expired:
            del self.entries[node]
        self._seen = {
            marker: seen_at
            for marker, seen_at in self._seen.items()
            if marker[0] in self.entries and now - seen_at <= max_age_s
        }

    def snapshot(self, now: float) -> list[dict[str, object]]:
        return [
            {
                **asdict(value),
                "age_s": now - value.last_seen_s,
                "delivery_ratio": value.delivery_ratio,
            }
            for value in sorted(self.entries.values(), key=lambda item: item.sender_id)
        ]


def serialize_links(links: Iterable[LinkPrediction]) -> list[dict[str, object]]:
    return [asdict(link) for link in links]
