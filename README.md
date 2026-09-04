# Drone network simulation and verification

This repository is an executable software specification for the proposed
FPGA-accelerated, multi-hop LoRa collision-avoidance project. It exercises every
project layer available without physical radios, aircraft, GPS receivers, or an
FPGA board. PX4 SIH runs in its official container.

## What is simulated

- 3D closest-approach prediction, finite horizons, stale/future state handling
- alpha-beta tracking, uncertainty growth, noisy position, velocity and clocks
- a frozen byte-exact 36-byte v2 frame with CRC-16 and a boot/session ID
- corruption, replay, restart, loss, burst loss, duplicates and sequence rollover
- exact LoRa airtime for SF/BW/CR/header/CRC/LDRO and duty-cycle queues
- range/path loss, sensitivity, capture effect, half duplex and asymmetric links
- flooding, relay and deterministic probabilistic forwarding
- CSMA-like access and unslotted ALOHA congestion
- a seven-candidate, multi-threat planner with vertical geofence constraints
- command acknowledgement, delay, rejection, loss, timeout and acceleration limits
- GPS jump, clock skew, restart, telemetry blackout and 300-second endurance faults
- real MAVLink v2 telemetry and local-NED velocity-command serialization
- fixed-point math; RTL parser, neighbor table and collision predictor
- Yosys synthesis elaboration and resource/cell evidence
- real LoRa-packet-to-planner-to-MAVLink-to-PX4 closed-loop execution

See [the test matrix](docs/TEST_MATRIX.md), [protocol specification](docs/PROTOCOL_SPEC.md),
and [latest readiness verdict](results/full/PRE_HARDWARE_READINESS.md).

## Reproduce everything

```bash
cd /home/caleb/school/401/drone_sims
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[sitl]'
make test
make verify
make rtl
make synth
make network-matrix
make px4-sitl
make px4-closed-loop
make readiness
make visualize
```

Important artifacts are `verification.{md,json}`, `network_matrix.json`,
`fpga_synthesis.json`, `fpga_golden_vectors.csv`, `px4_sitl.json`,
`px4_closed_loop.json`, `PRE_HARDWARE_READINESS.md`, and the interactive
`dashboard.html` under `results/full/`.

Open `results/full/dashboard.html` directly in a browser. It is self-contained
and visualizes gate status, scenario safety margins, the network design space,
tracking tradeoffs, RTL footprint, and replayable NED flight trajectories.

## Current verified results

The current evidence executed 42 unit/integration tests, 25,000 protocol
corruptions, 25,000 noisy prediction trials, 25,000 fixed-point trials, 5,000
compiled RTL vectors, 1,500 complete scenario runs, and three 300-second endurance runs.

- All 13 deterministic checks and all 14 frozen pre-hardware gates pass.
- CRC rejected 25,000/25,000 single-bit corruptions. Fixed-point classification
  had zero mismatches away from threshold boundaries.
- Filtering plus uncertainty margins raised recall from 73.6% to 90.8%; precision
  fell from 86.0% to 64.0%, an explicit conservative-safety tradeoff.
- The 36-row network matrix selected SF7/BW250, CSMA, flooding, and 1 Hz telemetry:
  38.528 ms airtime, 0.778 mean PDR, and 516 ms mean latency.
- SF12/BW125 took 1.974 s/frame and collapsed under the tested load. It is not a
  viable default simply because it has a better link budget.
- Every required 100-seed safety campaign maintained the 4 m operational margin, including
  crossing traffic, simultaneous threats, GPS jump, clock skew, node restart,
  and telemetry blackout. Crossing was the tightest at 4.36 m. The deliberately
  unmitigated, command-loss and ALOHA
  overload cases still fail, as expected.
- RTL passed the parser/CRC/table integration test and 5,000 randomized predictor
  vectors; all three modules pass Yosys elaboration and synthesis checks.
- Basic PX4 SIH passed 12/12 lifecycle/control checks. The integrated LoRa-to-PX4
  run also passed 12/12 and maintained 7.39 m measured separation against the
  4 m operational gate.

Generated reports include their UTC time, source SHA-256, exact seeds, parameters,
Python/tool versions, Git revision when available, and pinned PX4 image digest.
GitHub Actions runs the software, RTL, synthesis, and dashboard checks on every push.

These are model results, not flight certification. The remaining hardware-only
work is listed in the readiness report.

## Project layout

```text
src/drone_sims/       simulator, protocol, algorithms, campaigns and CLI
tests/                deterministic and randomized automated tests
rtl/                  parser, neighbor table, predictor and integration testbench
tools/                RTL, synthesis, PX4 and readiness harnesses
requirements/         frozen quantitative acceptance thresholds
docs/                 architecture, protocol and coverage boundary
results/full/         reproducible machine-readable evidence and reports
```

## Limits

No simulator validates real RF range/interference, antenna placement, regulatory
compliance, GNSS multipath/jamming, airframe dynamics, FPGA timing closure/power,
electrical integration, or actual flight safety. Those remain explicit hardware
and field-test gates.
