from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class LoRaModem:
    spreading_factor: int = 7
    bandwidth_hz: int = 125_000
    coding_rate: int = 1
    preamble_symbols: int = 8
    explicit_header: bool = True
    crc_enabled: bool = True
    low_data_rate_optimization: bool | None = None

    def __post_init__(self) -> None:
        if not 5 <= self.spreading_factor <= 12:
            raise ValueError("spreading_factor must be between 5 and 12")
        if self.bandwidth_hz not in {7_800, 10_400, 15_600, 20_800, 31_250, 41_700, 62_500, 125_000, 250_000, 500_000}:
            raise ValueError("unsupported LoRa bandwidth")
        if not 1 <= self.coding_rate <= 4:
            raise ValueError("coding_rate must be 1 (4/5) through 4 (4/8)")

    @property
    def symbol_seconds(self) -> float:
        return (2**self.spreading_factor) / self.bandwidth_hz

    @property
    def low_rate_enabled(self) -> bool:
        if self.low_data_rate_optimization is not None:
            return self.low_data_rate_optimization
        return self.symbol_seconds >= 0.016

    def payload_symbols(self, payload_bytes: int) -> int:
        if payload_bytes < 0 or payload_bytes > 255:
            raise ValueError("LoRa payload must contain 0 through 255 bytes")
        sf = self.spreading_factor
        numerator = (
            8 * payload_bytes
            - 4 * sf
            + 28
            + (16 if self.crc_enabled else 0)
            - (0 if self.explicit_header else 20)
        )
        denominator = 4 * (sf - (2 if self.low_rate_enabled else 0))
        encoded = max(math.ceil(numerator / denominator) * (self.coding_rate + 4), 0)
        return 8 + encoded

    def airtime_seconds(self, payload_bytes: int) -> float:
        preamble = (self.preamble_symbols + 4.25) * self.symbol_seconds
        payload = self.payload_symbols(payload_bytes) * self.symbol_seconds
        return preamble + payload

    def capacity_packets_per_second(self, payload_bytes: int, duty_cycle: float = 1.0) -> float:
        if not 0 < duty_cycle <= 1:
            raise ValueError("duty_cycle must be in (0, 1]")
        return duty_cycle / self.airtime_seconds(payload_bytes)


COMMON_PROFILES: dict[str, LoRaModem] = {
    "fast": LoRaModem(spreading_factor=7, bandwidth_hz=250_000, coding_rate=1),
    "balanced": LoRaModem(spreading_factor=9, bandwidth_hz=125_000, coding_rate=1),
    "long_range": LoRaModem(spreading_factor=12, bandwidth_hz=125_000, coding_rate=1),
}
