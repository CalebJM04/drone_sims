from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
import random
from typing import Callable

from .collision import Vec3
from .lora_phy import LoRaModem


@dataclass(frozen=True, slots=True)
class LinkOutage:
    a: int
    b: int
    start: float
    end: float | None = None

    def active(self, sender: int, receiver: int, now: float) -> bool:
        return {self.a, self.b} == {sender, receiver} and now >= self.start and (self.end is None or now < self.end)


@dataclass(slots=True)
class RadioConfig:
    range_m: float = 30.0
    bit_rate: float = 5_000.0
    preamble_s: float = 0.025
    processing_s: float = 0.010
    max_backoff_s: float = 0.350
    random_loss: float = 0.01
    bit_corruption: float = 0.001
    burst_enter: float = 0.01
    burst_exit: float = 0.30
    burst_loss: float = 0.80
    propagation_s: float = 0.001
    channel_access: str = "csma"
    modem: LoRaModem | None = field(
        default_factory=lambda: LoRaModem(spreading_factor=7, bandwidth_hz=250_000)
    )
    duty_cycle: float = 1.0
    tx_power_dbm: float = 20.0
    path_loss_at_1m_db: float = 40.0
    path_loss_exponent: float = 2.2
    receiver_sensitivity_dbm: float = -130.0
    capture_threshold_db: float = 6.0
    asymmetric_loss: dict[tuple[int, int], float] = field(default_factory=dict)

    def airtime(self, byte_count: int) -> float:
        return self.modem.airtime_seconds(byte_count) if self.modem else self.preamble_s + byte_count * 8.0 / self.bit_rate

    def received_power_dbm(self, distance_m: float) -> float:
        distance = max(1.0, distance_m)
        return self.tx_power_dbm - self.path_loss_at_1m_db - 10.0 * self.path_loss_exponent * math.log10(distance)


@dataclass(slots=True)
class Reception:
    sender: int
    receiver: int
    data: bytes
    path: tuple[int, ...]
    started_at: float
    ends_at: float
    collided: bool = False
    lost: bool = False
    corrupted: bool = False
    received_power_dbm: float = 0.0
    half_duplex_blocked: bool = False


class RadioMedium:
    def __init__(
        self,
        config: RadioConfig,
        rng: random.Random,
        position: Callable[[int], Vec3],
        outages: list[LinkOutage] | None = None,
    ) -> None:
        self.config = config
        self.rng = rng
        self.position = position
        self.outages = outages or []
        self.nodes: set[int] = set()
        self._queue: list[tuple[float, int, Reception]] = []
        self._active: dict[int, list[Reception]] = {}
        self._counter = 0
        self._burst_bad: dict[tuple[int, int], bool] = {}
        self._next_tx_allowed: dict[int, float] = {}
        self._tx_windows: dict[int, list[tuple[float, float]]] = {}
        self.transmission_attempts = 0
        self.queue_delay_seconds = 0.0
        self.half_duplex_losses = 0

    def register(self, node_id: int) -> None:
        self.nodes.add(node_id)

    def _reachable(self, sender: int, receiver: int, now: float) -> bool:
        if any(outage.active(sender, receiver, now) for outage in self.outages):
            return False
        distance = (self.position(sender) - self.position(receiver)).norm()
        return distance <= self.config.range_m and self.config.received_power_dbm(distance) >= self.config.receiver_sensitivity_dbm

    def _link_lost(self, sender: int, receiver: int) -> bool:
        key = (sender, receiver)
        bad = self._burst_bad.get(key, False)
        if bad:
            bad = not (self.rng.random() < self.config.burst_exit)
        else:
            bad = self.rng.random() < self.config.burst_enter
        self._burst_bad[key] = bad
        loss = self.config.burst_loss if bad else self.config.asymmetric_loss.get(key, self.config.random_loss)
        return self.rng.random() < loss

    def broadcast(self, sender: int, data: bytes, path: tuple[int, ...], now: float) -> None:
        if not 0 < self.config.duty_cycle <= 1.0:
            raise ValueError("duty_cycle must be in (0, 1]")
        requested_start = now + self.rng.uniform(0.0, self.config.max_backoff_s)
        start = max(requested_start, self._next_tx_allowed.get(sender, requested_start))
        duration = self.config.airtime(len(data)) + self.config.propagation_s + self.config.processing_s
        receivers = [
            receiver
            for receiver in self.nodes
            if receiver != sender and receiver not in path and self._reachable(sender, receiver, start)
        ]
        if self.config.channel_access not in {"csma", "aloha"}:
            raise ValueError("channel_access must be 'csma' or 'aloha'")
        if self.config.channel_access == "csma":
            while True:
                conflicts = [
                    prior
                    for receiver in receivers
                    for prior in self._active.get(receiver, [])
                    if prior.ends_at > start and prior.started_at < start + duration
                ]
                if not conflicts:
                    break
                start = max(prior.ends_at for prior in conflicts) + self.rng.uniform(0.0, self.config.max_backoff_s)
        end = start + duration
        self.queue_delay_seconds += max(0.0, start - requested_start)
        off_time = self.config.airtime(len(data)) * (1.0 / self.config.duty_cycle - 1.0)
        self._next_tx_allowed[sender] = end + off_time
        self._tx_windows.setdefault(sender, []).append((start, end))
        # A LoRa transceiver cannot receive while its own transmitter is on.
        for prior in self._active.get(sender, []):
            if prior.started_at < end and start < prior.ends_at and not prior.lost:
                prior.lost = True
                prior.half_duplex_blocked = True
                self.half_duplex_losses += 1
        for receiver in receivers:
            self.transmission_attempts += 1
            distance = (self.position(sender) - self.position(receiver)).norm()
            reception = Reception(
                sender, receiver, data, path + (receiver,), start, end,
                received_power_dbm=self.config.received_power_dbm(distance),
            )
            reception.lost = self._link_lost(sender, receiver)
            receiver_tx = self._tx_windows.get(receiver, [])
            if any(tx_start < end and start < tx_end for tx_start, tx_end in receiver_tx):
                reception.lost = True
                reception.half_duplex_blocked = True
                self.half_duplex_losses += 1
            active = self._active.setdefault(receiver, [])
            active[:] = [prior for prior in active if prior.ends_at > start]
            if self.config.channel_access == "aloha":
                for prior in active:
                    if prior.started_at < end and start < prior.ends_at:
                        power_delta = reception.received_power_dbm - prior.received_power_dbm
                        if power_delta >= self.config.capture_threshold_db:
                            prior.collided = True
                        elif power_delta <= -self.config.capture_threshold_db:
                            reception.collided = True
                        else:
                            prior.collided = True
                            reception.collided = True
            active.append(reception)
            self._counter += 1
            heapq.heappush(self._queue, (end, self._counter, reception))

    def poll(self, until: float) -> list[Reception]:
        ready: list[Reception] = []
        while self._queue and self._queue[0][0] <= until + 1e-12:
            _, _, reception = heapq.heappop(self._queue)
            if not reception.collided and not reception.lost and self.rng.random() < self.config.bit_corruption:
                mutable = bytearray(reception.data)
                index = self.rng.randrange(len(mutable))
                mutable[index] ^= 1 << self.rng.randrange(8)
                reception.data = bytes(mutable)
                reception.corrupted = True
            ready.append(reception)
        return ready
