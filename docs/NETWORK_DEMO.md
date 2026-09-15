# Multi-node software demonstration

## Purpose

This is the project's primary demonstration. It validates a drone awareness
network without requiring radio hardware, propellers, or a flyable aircraft.
Every simulated node runs the production `CompanionService` in an independent
operating-system process, uses the frozen telemetry protocol, and communicates
through the same LoRa airtime, range, half-duplex, loss, corruption, and
channel-access model used by the campaign suite.

The default scene contains six moving nodes:

- D1 and D2 begin in direct range. D2 moves away while D3 and D4 retain a relay
  path. Routing selects the future relay path before the direct RF link breaks.
- D5 and D6 move toward one another. Both report a collision-awareness alert,
  but no velocity command is sent.
- All nodes transmit telemetry at 1 Hz and maintain neighbor, route, link-quality,
  and collision-awareness state.

## Acceptance run

```bash
make mesh-acceptance
```

Pass/fail limits are frozen in
`requirements/network_demo_acceptance.json`. The six-node configuration requires:

- at least four nodes;
- at least 90% unique source-to-node delivery;
- no more than 1,500 ms p95 application latency;
- no more than 3 seconds maximum observed state age;
- at least one preemptive route handoff;
- at least one collision-awareness alert;
- zero flight-control commands; and
- zero application protocol-integrity errors.

The default six-node result is written to `results/mesh/demo.json`; the
self-contained replay is `results/mesh/dashboard.html`.

## Live presentation

```bash
make mesh-demo
```

Open `http://127.0.0.1:8080` while the command runs. Green, yellow, and red links
represent healthy, degrading, and critical/out-of-range predictions. The thick
blue line is the route selected using the four-second look-ahead topology.

For a quick non-real-time run or a different scale:

```bash
.venv/bin/drone-sims mesh-demo --nodes 8 --duration 20 --backend process
```

The supported range is 4 to 16 nodes. Six is the validated presentation size;
larger values are stress experiments and may require a longer telemetry interval:

```bash
.venv/bin/drone-sims mesh-demo --nodes 16 --duration 20 \
  --telemetry-interval 2
```

At the default modem settings, source telemetry alone consumes about 23% of the
modeled channel at six nodes and about 62% at sixteen nodes. Relays, discovery,
and retries add more traffic, so a 64-node configuration would not be credible
on one channel at 1 Hz. It is intentionally not offered by the CLI.

## Routing behavior

`routing.mode = "proactive"` predicts every known node's position at the configured
look-ahead time, builds a trusted-range graph, and forwards on deterministic
shortest paths. A route margin avoids waiting for the modeled absolute range
boundary. Periodic discovery floods repair partial topology and discover new or
moving nodes. Flooding, fixed relay, and probabilistic forwarding remain available
for comparisons and fallbacks.

Every companion snapshot includes:

- current and projected link distance;
- remaining modeled range margin and estimated time to loss;
- link state (`healthy`, `degrading`, `critical`, or `out_of_range`);
- immediate-neighbor RSSI/SNR and observed delivery ratio when supplied by the
  transport;
- future routes from that node; and
- collision distance, predicted closest approach, time to closest approach, and
  uncertainty margin.

## Process model

The parent process advances a virtual clock and applies the deterministic radio
model. Each node process owns its flight-state source, routing table, neighbor
table, collision tracker, and `CompanionService`. On every simulation step, a
node receives frames delivered by the hub and returns any new transmission plus
its current snapshot. The dashboard reports the backend, number of node
processes, process IDs, and modeled source-channel load.

Use `--backend inline` for a faster reference run in automated campaigns. It uses
the same node and radio logic in one process, but the process backend is the main
presentation and acceptance mode.

The old Raspberry Pi, PX4, and Heltec instructions remain in the repository as
optional future integration notes. They are outside the current project claim
and are not needed for acceptance.
