from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class RadioReception:
    payload: bytes
    sender_id: int | None = None
    rssi_dbm: float | None = None
    snr_db: float | None = None
