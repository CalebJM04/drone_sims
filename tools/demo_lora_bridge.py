#!/usr/bin/env python3
"""Command-line demo for a Heltec running the binary bridge firmware."""

from __future__ import annotations

import argparse
import secrets
import time

from drone_sims.companion_io import SerialFrameRadio
from drone_sims.protocol import ProtocolError, TelemetryFrame, decode, encode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", required=True, help="Heltec serial device")
    parser.add_argument("--node-id", type=int, help="also transmit as this node")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument(
        "--start-at-unix",
        type=float,
        help="wait for this Unix time so two hosts begin together",
    )
    args = parser.parse_args()

    radio = SerialFrameRadio(args.device)
    if args.start_at_unix is not None:
        while time.time() < args.start_at_unix:
            time.sleep(0.01)
    boot_id = secrets.randbelow(0xFFFF) + 1
    sequence = 0
    received = 0
    rejected = 0
    transmitted = 0
    started = time.monotonic()
    deadline = started + args.seconds
    next_transmit = started + 0.75
    if not args.quiet:
        print(f"LoRa bridge open: {args.device}", flush=True)

    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if args.node_id is not None and now >= next_transmit:
                frame = TelemetryFrame(
                    source=args.node_id,
                    sequence=sequence,
                    boot_id=boot_id,
                    timestamp_ms=round((now - started) * 1000),
                    position_cm=(args.node_id * 100, 0, 0),
                    velocity_cms=(0, 0, 0),
                    ttl=0,
                )
                radio.send(encode(frame))
                if not args.quiet:
                    print(f"TX -> node={args.node_id} seq={sequence}", flush=True)
                sequence = (sequence + 1) & 0xFFFF
                transmitted += 1
                next_transmit += args.interval

            for payload in radio.receive():
                try:
                    frame = decode(payload)
                except ProtocolError:
                    rejected += 1
                    continue
                received += 1
                if not args.quiet:
                    print(
                        f"RX <- node={frame.source} seq={frame.sequence} "
                        f"position_cm={frame.position_cm}",
                        flush=True,
                    )
            time.sleep(0.01)
    finally:
        radio.close()

    print(
        f"SUMMARY tx={transmitted} rx={received} rejected={rejected}",
        flush=True,
    )
    return 0 if received else 1


if __name__ == "__main__":
    raise SystemExit(main())
