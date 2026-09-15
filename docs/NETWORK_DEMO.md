# 4+ node network demonstration

## Purpose

This is the project's primary demonstration. It validates a drone awareness
network without requiring propellers or a flyable aircraft. Every simulated node
runs the production `CompanionService`, uses the frozen telemetry protocol, and
communicates through the same LoRa airtime, range, half-duplex, loss, corruption,
and channel-access model used by the campaign suite.

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
`requirements/network_demo_acceptance.json`. The default requires:

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
.venv/bin/drone-sims mesh-demo --nodes 8 --duration 20
```

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

## Moving from simulation to four RF nodes

Each RF node needs one host running the companion service and one reflashed
Heltec bridge. A host can be a Raspberry Pi, laptop, or another Linux computer;
a physical drone is not required. Use `tools/fake_pixhawk.py` or PX4 SITL as the
position source. Props-off drone frames may carry the nodes for presentation.

The metadata-aware bridge uses:

- a 40-byte host-to-bridge record containing bridge marker/version, immediate
  transmitter ID, and the frozen 36-byte telemetry frame;
- a 38-byte over-air record containing immediate transmitter ID and telemetry;
  and
- a 44-byte bridge-to-host record adding signed RSSI/SNR values in tenths.

Re-run the two-radio bench acceptance after reflashing, then add two radios and
run a four-node stationary topology. Do not treat simulated range or RSSI as
calibration evidence. Physical range, obstruction, antenna, interference, and
coexistence tests remain required.

For each of four nodes, run `tools/soak_bench_node.py` with that node's config,
fake-Pixhawk endpoint, and the four-node acceptance switches:

```bash
.venv/bin/python tools/soak_bench_node.py \
  --config config/node-N.toml --fake-pixhawk tools/fake_pixhawk.py \
  --target 127.0.0.1:1455N --node-id N --north POSITION \
  --duration 900 --expected-peers 3 --minimum-neighbors 3 \
  --require-link-metadata --output results/bench/mesh-node-N.json
```

Replace `N`, `POSITION`, and the MAVLink port with the values for each host.
Every report checks observation-only operation and now preserves the link-quality
table for later comparison with distance.

After all four records are collected, combine them into one acceptance decision:

```bash
.venv/bin/python tools/evaluate_rf_mesh.py results/bench/mesh-node-*.json
```
