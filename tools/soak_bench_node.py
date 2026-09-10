#!/usr/bin/env python3
"""Run one complete fake-PX4/companion/Heltec node for a timed soak."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import Any, TextIO

from drone_sims.provenance import collect_metadata


def consume_json(
    stream: TextIO,
    state: dict[str, Any],
    key: str,
) -> None:
    for line in stream:
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            output = state.setdefault(f"{key}_non_json_output", [])
            output.append(line.rstrip())
            del output[:-20]
            continue
        state[key] = value
        state[f"{key}_lines"] = int(state.get(f"{key}_lines", 0)) + 1
        if isinstance(value, dict) and "stats" in value:
            state["companion_snapshot"] = value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--fake-pixhawk", type=Path, required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--node-id", type=int, required=True)
    parser.add_argument("--north", type=float, required=True)
    parser.add_argument("--duration", type=float, default=3600.0)
    parser.add_argument("--progress-interval", type=float, default=60.0)
    parser.add_argument("--minimum-delivery-ratio", type=float, default=0.95)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.progress_interval <= 0:
        parser.error("--progress-interval must be positive")
    if not 0.0 <= args.minimum_delivery_ratio <= 1.0:
        parser.error("--minimum-delivery-ratio must be between 0 and 1")

    state: dict[str, Any] = {}
    fake = subprocess.Popen(
        [
            sys.executable,
            str(args.fake_pixhawk),
            "--target",
            args.target,
            "--node-id",
            str(args.node_id),
            "--north",
            str(args.north),
            "--vn",
            "0",
            "--duration",
            str(args.duration + 15.0),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    companion = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "drone_sims.cli",
            "companion",
            "--config",
            str(args.config),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    assert fake.stdout is not None
    assert companion.stdout is not None
    readers = [
        threading.Thread(
            target=consume_json,
            args=(fake.stdout, state, "fake"),
            daemon=True,
        ),
        threading.Thread(
            target=consume_json,
            args=(companion.stdout, state, "companion"),
            daemon=True,
        ),
    ]
    for reader in readers:
        reader.start()

    started = time.monotonic()
    next_progress = started + args.progress_interval
    interrupted = False
    try:
        while time.monotonic() - started < args.duration:
            if fake.poll() is not None or companion.poll() is not None:
                break
            now = time.monotonic()
            if now >= next_progress:
                snapshot = state.get("companion_snapshot", {})
                print(
                    json.dumps(
                        {
                            "event": "soak_progress",
                            "node": args.node_id,
                            "elapsed_s": round(now - started, 1),
                            "stats": snapshot.get("stats", {}),
                            "neighbors": snapshot.get("neighbors"),
                            "px4": snapshot.get("px4", {}),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                next_progress += args.progress_interval
            time.sleep(0.2)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        for process in (companion, fake):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in (companion, fake):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        for reader in readers:
            reader.join(timeout=1)

    elapsed = time.monotonic() - started
    snapshot = state.get("companion_snapshot", {})
    stats = snapshot.get("stats", {})
    telemetry_sent = int(stats.get("telemetry_sent", 0))
    received = int(stats.get("received", 0))
    estimated_delivery_ratio = (
        min(1.0, received / telemetry_sent) if telemetry_sent > 0 else 0.0
    )
    passed = (
        not interrupted
        and elapsed >= args.duration
        and companion.returncode == 0
        and fake.returncode == 0
        and snapshot.get("own_state_fresh") is True
        and snapshot.get("control_enabled") is False
        and int(snapshot.get("neighbors", 0)) >= 1
        and telemetry_sent > 0
        and received > 0
        and estimated_delivery_ratio >= args.minimum_delivery_ratio
        and stats.get("crc_rejections", 1) == 0
        and stats.get("protocol_rejections", 1) == 0
        and stats.get("stale_own_state_events", 1) == 0
        and stats.get("commands_sent", 1) == 0
    )
    result = {
        "metadata": collect_metadata(
            "tools/soak_bench_node.py",
            parameters={
                "config": str(args.config),
                "duration_s": args.duration,
                "minimum_delivery_ratio": args.minimum_delivery_ratio,
                "node_id": args.node_id,
            },
        ),
        "event": "soak_summary",
        "passed": passed,
        "node": args.node_id,
        "elapsed_s": round(elapsed, 1),
        "companion_returncode": companion.returncode,
        "fake_px4_returncode": fake.returncode,
        "companion_non_json_output": state.get("companion_non_json_output", []),
        "fake_px4_non_json_output": state.get("fake_non_json_output", []),
        "companion_snapshots": state.get("companion_lines", 0),
        "minimum_delivery_ratio": args.minimum_delivery_ratio,
        "estimated_delivery_ratio": round(estimated_delivery_ratio, 6),
        "stats": stats,
        "neighbors": snapshot.get("neighbors"),
        "px4": snapshot.get("px4", {}),
    }
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
