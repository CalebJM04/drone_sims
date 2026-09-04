from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class RoutingPolicy:
    """Deterministic broadcast-forwarding policy for repeatable experiments."""

    mode: str = "flooding"
    relay_nodes: tuple[int, ...] = ()
    forwarding_probability: float = 0.65
    seed: int = 1

    def should_forward(self, node_id: int, source: int, sequence: int) -> bool:
        if self.mode == "flooding":
            return True
        if self.mode == "relay":
            return node_id in self.relay_nodes
        if self.mode == "probabilistic":
            if not 0.0 <= self.forwarding_probability <= 1.0:
                raise ValueError("forwarding_probability must be in [0, 1]")
            # Stable integer mixing keeps campaign runs reproducible and avoids
            # changing radio/error random streams when a packet is suppressed.
            value = (
                source * 0x9E3779B1
                ^ sequence * 0x85EBCA77
                ^ node_id * 0xC2B2AE3D
                ^ self.seed
            ) & 0xFFFFFFFF
            value ^= value >> 16
            return value / 0xFFFFFFFF < self.forwarding_probability
        raise ValueError("routing mode must be 'flooding', 'relay', or 'probabilistic'")
