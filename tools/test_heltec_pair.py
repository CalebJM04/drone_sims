#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import selectors
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--v2", default="/dev/ttyUSB0")
    parser.add_argument("--v3", default="/dev/ttyUSB1")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    ports = {
        "v2": serial.Serial(args.v2, 115200, timeout=0),
        "v3": serial.Serial(args.v3, 115200, timeout=0),
    }
    selector = selectors.DefaultSelector()
    buffers = {name: bytearray() for name in ports}
    events: list[dict[str, object]] = []
    for name, port in ports.items():
        selector.register(port.fileno(), selectors.EVENT_READ, name)

    deadline = time.monotonic() + args.seconds
    started_at = time.monotonic()
    try:
        while time.monotonic() < deadline:
            for key, _ in selector.select(timeout=0.25):
                name = key.data
                port = ports[name]
                buffers[name].extend(port.read(port.in_waiting or 1))
                while b"\n" in buffers[name]:
                    raw, _, remainder = buffers[name].partition(b"\n")
                    buffers[name] = bytearray(remainder)
                    try:
                        event = json.loads(raw)
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    event["port"] = name
                    event["observed_s"] = round(time.monotonic() - started_at, 4)
                    events.append(event)
                    if not args.quiet:
                        print(json.dumps(event, separators=(",", ":")), flush=True)
    finally:
        for port in ports.values():
            port.close()

    tx = {2: set(), 3: set()}
    received = {(2, 3): set(), (3, 2): set()}
    boots: dict[int, int] = {}
    radio_errors = 0
    rejected = 0
    rssi: list[float] = []
    snr: list[float] = []
    for event in events:
        kind = event.get("event")
        node = int(event.get("node", 0))
        if kind == "boot":
            boots[node] = int(event.get("radio_state", -999))
        elif kind == "tx" and int(event.get("state", -999)) == 0:
            tx.setdefault(node, set()).add(int(event["seq"]))
        elif kind == "rx":
            source = int(event["from"])
            received.setdefault((source, node), set()).add(int(event["seq"]))
            rssi.append(float(event["rssi"]))
            snr.append(float(event["snr"]))
        elif kind == "rx_error":
            radio_errors += 1
        elif kind == "rx_rejected":
            rejected += 1

    links = {}
    passed = all(state == 0 for state in boots.values())
    for source, destination in ((2, 3), (3, 2)):
        sent = tx.get(source, set())
        seen = received.get((source, destination), set())
        matched = len(sent & seen)
        ratio = matched / len(sent) if sent else 0.0
        links[f"{source}->{destination}"] = {
            "sent_observed": len(sent),
            "received": matched,
            "delivery_ratio": round(ratio, 4),
        }
        passed = passed and len(sent) >= 5 and matched >= 5 and ratio >= 0.8

    summary = {
        "passed": passed,
        "duration_s": args.seconds,
        "boots": boots,
        "links": links,
        "radio_errors": radio_errors,
        "rejected_frames": rejected,
        "rssi_dbm_mean": round(sum(rssi) / len(rssi), 2) if rssi else None,
        "snr_db_mean": round(sum(snr) / len(snr), 2) if snr else None,
    }
    print("SUMMARY " + json.dumps(summary, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
