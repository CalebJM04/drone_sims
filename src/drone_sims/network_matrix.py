from __future__ import annotations

from dataclasses import asdict
from statistics import fmean
from typing import Any

from .lora_phy import COMMON_PROFILES
from .campaign import DEFAULT_OPERATIONAL_MINIMUM_SEPARATION_M
from .protocol import FRAME_SIZE
from .provenance import collect_metadata
from .routing import RoutingPolicy
from .scenarios import build


def run_network_matrix(seeds: int = 3) -> dict[str, Any]:
    """Compare PHY, MAC, routing and offered load on the same topology."""

    rows: list[dict[str, Any]] = []
    for profile_name, modem in COMMON_PROFILES.items():
        for mac in ("csma", "aloha"):
            for routing in ("flooding", "relay", "probabilistic"):
                for interval in (0.5, 1.0):
                    runs = []
                    for seed in range(1, seeds + 1):
                        simulation = build("head_on", seed, event_logging=False)
                        simulation.config.telemetry_interval = interval
                        simulation.config.routing = RoutingPolicy(
                            mode=routing,
                            relay_nodes=(3, 4),
                            forwarding_probability=0.65,
                            seed=seed,
                        )
                        simulation.radio_config.modem = modem
                        simulation.radio_config.channel_access = mac
                        runs.append(simulation.run())
                    latencies = [
                        run["network"]["mean_latency_ms"] for run in runs
                        if run["network"]["mean_latency_ms"] is not None
                    ]
                    rows.append({
                        "phy": profile_name,
                        "mac": mac,
                        "routing": routing,
                        "telemetry_interval_s": interval,
                        "seeds": seeds,
                        "time_on_air_ms": modem.airtime_seconds(FRAME_SIZE) * 1000,
                        "single_channel_capacity_pps": modem.capacity_packets_per_second(FRAME_SIZE),
                        "delivery_ratio_mean": fmean(run["network"]["delivery_ratio"] for run in runs),
                        "latency_ms_mean": fmean(latencies) if latencies else None,
                        "transmission_attempts_mean": fmean(run["network"]["transmission_attempts"] for run in runs),
                        "half_duplex_losses_mean": fmean(run["network"]["half_duplex_losses"] for run in runs),
                        "minimum_separation_worst_m": min(run["avoidance"]["minimum_separation_m"] for run in runs),
                    })
    viable = [
        row for row in rows
        if row["delivery_ratio_mean"] >= 0.70
        and row["latency_ms_mean"] is not None
        and row["latency_ms_mean"] <= 1500
        and row["minimum_separation_worst_m"] >= DEFAULT_OPERATIONAL_MINIMUM_SEPARATION_M
    ]
    recommended = max(
        viable or rows,
        key=lambda row: (
            row["delivery_ratio_mean"],
            -float(row["latency_ms_mean"] or 1e12),
            -row["transmission_attempts_mean"],
        ),
    )
    return {
        "metadata": collect_metadata(
            "drone-sims network-matrix",
            seeds=range(1, seeds + 1),
            parameters={"seeds": seeds, "rows": len(rows)},
        ),
        "frame_size_bytes": FRAME_SIZE,
        "minimum_separation_requirement_m": DEFAULT_OPERATIONAL_MINIMUM_SEPARATION_M,
        "profiles": {name: asdict(modem) for name, modem in COMMON_PROFILES.items()},
        "rows": rows,
        "viable_rows": len(viable),
        "recommendation": recommended,
    }
