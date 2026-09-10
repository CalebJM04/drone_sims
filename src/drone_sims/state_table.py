from __future__ import annotations

from dataclasses import dataclass

from .collision import KinematicState
from .protocol import TelemetryFrame, sequence_is_newer


@dataclass(frozen=True, slots=True)
class StateEntry:
    state: KinematicState
    sequence: int
    boot_id: int
    received_at: float
    hops: int


class NeighborTable:
    def __init__(self, capacity: int = 16) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self.capacity = capacity
        self.entries: dict[int, StateEntry] = {}
        self.rejected_old = 0
        self.evictions = 0
        self._retired_boots: dict[int, list[int]] = {}
        # Replay metadata deliberately survives state expiry. A radio outage
        # must not make an old packet look new when the neighbor reappears.
        self._sessions: dict[int, tuple[int, int, int]] = {}

    def update(self, frame: TelemetryFrame, received_at: float) -> bool:
        session = self._sessions.get(frame.source)
        retired = self._retired_boots.get(frame.source, [])
        if session is not None:
            active_boot, last_sequence, last_timestamp_ms = session
            if frame.boot_id == active_boot:
                # Sequence comparison is unambiguous for gaps shorter than half
                # the uint16 range. A synchronized timestamp also permits clean
                # recovery after a longer outage without accepting old packets.
                if not sequence_is_newer(frame.sequence, last_sequence) and (
                    frame.timestamp_ms <= last_timestamp_ms
                ):
                    self.rejected_old += 1
                    return False
            elif frame.boot_id in retired:
                self.rejected_old += 1
                return False
            else:
                retired = self._retired_boots.setdefault(frame.source, [])
                retired.append(active_boot)
                del retired[:-8]
        elif frame.boot_id in retired:
            self.rejected_old += 1
            return False
        old = self.entries.get(frame.source)
        if old is None and len(self.entries) >= self.capacity:
            victim = min(self.entries, key=lambda node: self.entries[node].received_at)
            del self.entries[victim]
            self.evictions += 1
        self.entries[frame.source] = StateEntry(
            frame.to_state(), frame.sequence, frame.boot_id, received_at, frame.hops
        )
        self._sessions[frame.source] = (
            frame.boot_id,
            frame.sequence,
            frame.timestamp_ms,
        )
        return True

    def expire(self, now: float, max_age: float) -> list[int]:
        expired = [node for node, entry in self.entries.items() if now - entry.received_at > max_age]
        for node in expired:
            del self.entries[node]
        return expired
