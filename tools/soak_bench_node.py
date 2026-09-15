#!/usr/bin/env python3
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
    parser.add_argument("--expected-peers", type=int, default=1)
    parser.add_argument("--minimum-neighbors", type=int, default=1)
    parser.add_argument("--require-link-metadata", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.progress_interval <= 0:
        parser.error("--progress-interval must be positive")
    if not 0.0 <= args.minimum_delivery_ratio <= 1.0:
        parser.error("--minimum-delivery-ratio must be between 0 and 1")
    if args.expected_peers <= 0 or args.minimum_neighbors <= 0:
        parser.error("--expected-peers and --minimum-neighbors must be positive")

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

    interrupted = False
    interruption_reason: str | None = None

    def request_stop(signum: int, _frame: Any) -> None:
        nonlocal interrupted, interruption_reason
        interrupted = True
        interruption_reason = signal.Signals(signum).name

    signal.signal(signal.SIGTERM, request_stop)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, request_stop)

    started = time.monotonic()
    next_progress = started + args.progress_interval
    try:
        while not interrupted and time.monotonic() - started < args.duration:
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
        interruption_reason = "KeyboardInterrupt"
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
    expected_deliveries = telemetry_sent * args.expected_peers
    estimated_delivery_ratio = (
        min(1.0, received / expected_deliveries) if expected_deliveries > 0 else 0.0
    )
    link_quality = snapshot.get("link_quality", [])
    metadata_available = bool(link_quality) and all(
        item.get("rssi_dbm") is not None and item.get("snr_db") is not None
        for item in link_quality
    )
    passed = (
        not interrupted
        and elapsed >= args.duration
        and companion.returncode == 0
        and fake.returncode == 0
        and snapshot.get("own_state_fresh") is True
        and snapshot.get("control_enabled") is False
        and int(snapshot.get("neighbors", 0)) >= args.minimum_neighbors
        and telemetry_sent > 0
        and received > 0
        and estimated_delivery_ratio >= args.minimum_delivery_ratio
        and stats.get("crc_rejections", 1) == 0
        and stats.get("protocol_rejections", 1) == 0
        and stats.get("stale_own_state_events", 1) == 0
        and stats.get("commands_sent", 1) == 0
        and (metadata_available or not args.require_link_metadata)
    )
    result = {
        "metadata": collect_metadata(
            "tools/soak_bench_node.py",
            parameters={
                "config": str(args.config),
                "duration_s": args.duration,
                "minimum_delivery_ratio": args.minimum_delivery_ratio,
                "node_id": args.node_id,
                "expected_peers": args.expected_peers,
                "minimum_neighbors": args.minimum_neighbors,
                "require_link_metadata": args.require_link_metadata,
            },
        ),
        "event": "soak_summary",
        "passed": passed,
        "interrupted": interrupted,
        "interruption_reason": interruption_reason,
        "node": args.node_id,
        "elapsed_s": round(elapsed, 1),
        "companion_returncode": companion.returncode,
        "fake_px4_returncode": fake.returncode,
        "companion_non_json_output": state.get("companion_non_json_output", []),
        "fake_px4_non_json_output": state.get("fake_non_json_output", []),
        "companion_snapshots": state.get("companion_lines", 0),
        "minimum_delivery_ratio": args.minimum_delivery_ratio,
        "expected_peers": args.expected_peers,
        "minimum_neighbors": args.minimum_neighbors,
        "estimated_delivery_ratio": round(estimated_delivery_ratio, 6),
        "link_metadata_available": metadata_available,
        "link_quality": link_quality,
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
    try:
        print(json.dumps(result, sort_keys=True), flush=True)
    except BrokenPipeError:
        pass
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
