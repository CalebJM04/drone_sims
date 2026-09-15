from __future__ import annotations

from dataclasses import dataclass

from .collision import KinematicState
from .network_awareness import proactive_forwarders


@dataclass(frozen=True, slots=True)
class RoutingPolicy:
    mode: str = "flooding"
    relay_nodes: tuple[int, ...] = ()
    forwarding_probability: float = 0.65
    seed: int = 1
    proactive_range_m: float = 30.0
    proactive_lookahead_s: float = 4.0
    proactive_route_margin: float = 0.90
    discovery_interval_packets: int = 5

    def should_forward(
        self,
        node_id: int,
        source: int,
        sequence: int,
        *,
        states: dict[int, KinematicState] | None = None,
        now: float = 0.0,
    ) -> bool:
        if self.mode == "flooding":
            return True
        if self.mode == "relay":
            return node_id in self.relay_nodes
        if self.mode == "probabilistic":
            if not 0.0 <= self.forwarding_probability <= 1.0:
                raise ValueError("forwarding_probability must be in [0, 1]")
            value = (
                source * 0x9E3779B1
                ^ sequence * 0x85EBCA77
                ^ node_id * 0xC2B2AE3D
                ^ self.seed
            ) & 0xFFFFFFFF
            value ^= value >> 16
            return value / 0xFFFFFFFF < self.forwarding_probability
        if self.mode == "proactive":
            # Send an occasional flood so nodes can find new routes.
            if (
                states is None
                or source not in states
                or len(states) < 3
                or self.discovery_interval_packets <= 1
                or sequence % self.discovery_interval_packets == 0
            ):
                return True
            return node_id in proactive_forwarders(
                states,
                source=source,
                now=now,
                range_m=self.proactive_range_m,
                lookahead_s=self.proactive_lookahead_s,
                route_margin=self.proactive_route_margin,
            )
        raise ValueError(
            "routing mode must be 'flooding', 'relay', 'probabilistic', or 'proactive'"
        )
