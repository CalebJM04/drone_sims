from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
import math
from statistics import fmean
from typing import Any

from .scenarios import build


DEFAULT_OPERATIONAL_MINIMUM_SEPARATION_M = 4.0


def _one_run(task: tuple[str, int]) -> tuple[str, dict[str, Any]]:
    name, seed = task
    return name, build(name, seed, event_logging=False).run()


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def run_campaign(
    names: list[str],
    seeds: int,
    workers: int = 1,
    minimum_separation_m: float = DEFAULT_OPERATIONAL_MINIMUM_SEPARATION_M,
) -> dict[str, Any]:
    tasks = [(name, seed) for name in names for seed in range(1, seeds + 1)]
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            results = list(pool.map(_one_run, tasks))
    else:
        results = [_one_run(task) for task in tasks]
    grouped: dict[str, list[dict[str, Any]]] = {name: [] for name in names}
    for name, result in results:
        grouped[name].append(result)
    report: dict[str, Any] = {
        "runs": len(results),
        "minimum_separation_requirement_m": minimum_separation_m,
        "scenarios": {},
    }
    for name, runs in grouped.items():
        delivery = [run["network"]["delivery_ratio"] for run in runs]
        latency = [run["network"]["mean_latency_ms"] for run in runs if run["network"]["mean_latency_ms"] is not None]
        separation = [run["avoidance"]["minimum_separation_m"] for run in runs]
        accuracy = [run["prediction"]["check_accuracy"] for run in runs if run["prediction"]["check_accuracy"] is not None]
        report["scenarios"][name] = {
            "runs": len(runs),
            "delivery_ratio_mean": fmean(delivery),
            "delivery_ratio_p05": _percentile(delivery, 0.05),
            "latency_ms_mean": fmean(latency) if latency else None,
            "latency_ms_p95": _percentile(latency, 0.95),
            "minimum_separation_mean_m": fmean(separation),
            "minimum_separation_worst_m": min(separation),
            "prediction_accuracy_mean": fmean(accuracy) if accuracy else None,
            "safety_distance_success_rate": sum(
                run["avoidance"]["minimum_separation_m"] >= minimum_separation_m
                for run in runs
            ) / len(runs),
            "command_failure_runs": sum(run["avoidance"]["command_failures"] > 0 for run in runs),
        }
    return report
