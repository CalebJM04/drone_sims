# Resilient drone awareness mesh

This college project is a software testbed for a resilient network of simulated
drones. Each node shares position and velocity over a modeled multi-hop LoRa
mesh, predicts weakening links, selects relay paths before a direct link breaks,
and reports collision risk. The project does not require flight or radio
hardware.

The main demonstration uses six independent node processes on one computer.
Table-top drones may be used as visual stand-ins, but all movement, radio
behavior, and flight state are simulated.

## Main network demo

The primary acceptance run starts six independent companion-service processes,
feeds each one a moving flight-controller trajectory, and connects them through
the LoRa RF model. It verifies delivery, latency, freshness, proactive route
handoff, collision alerts, protocol integrity, and observation-only operation.

```bash
make mesh-acceptance
```

For a wall-clock presentation with a live topology dashboard:

```bash
make mesh-demo
# open http://127.0.0.1:8080 while it runs
```

The saved report and replay are `results/mesh/demo.json` and
`results/mesh/dashboard.html`. See [docs/NETWORK_DEMO.md](docs/NETWORK_DEMO.md).

## What it tests

- head-on, crossing, and multiple-drone situations
- dropped or damaged radio packets
- network congestion and multi-hop messages
- proactive topology prediction and pre-break relay handoff
- per-link PDR, RSSI/SNR, range margin, and last-heard age
- noisy GPS data, clock errors, restarts, and missing telemetry
- collision prediction and avoidance commands
- MAVLink messages and PX4 software-in-the-loop responses
- independent companion-process state, forwarding, and failure behavior

More details are in [docs/TEST_MATRIX.md](docs/TEST_MATRIX.md) and
[docs/PROTOCOL_SPEC.md](docs/PROTOCOL_SPEC.md). Hardware deployment documents are
retained only as possible future work and are not part of the deliverable.

## Running it

```bash
cd /home/caleb/school/401/drone_sims
python3 -m venv .venv
.venv/bin/python -m pip install -e .
make test
make network-matrix
make mesh-acceptance
make verify
make readiness
```

PX4 software-in-the-loop is optional and needs the `sitl` extra:

```bash
.venv/bin/python -m pip install -e '.[sitl]'
make px4-sitl
make px4-closed-loop
```

The main output is [results/full/results.txt](results/full/results.txt). It is a
short plain-text summary. The JSON and CSV files in the same folder contain the
full data used to make that summary.

There is also an optional dashboard. Run `make visualize`, then open
`results/full/dashboard.html` in a browser.

## Current result

The regenerated baseline passes 22/22 readiness checks, including eight dedicated
mesh gates. It includes 1,500 scenario runs, 25,000 damaged-packet tests,
companion-service tests, endurance testing, and two PX4 tests. The closest
required simulated scenario stayed 4.36 meters apart; the latest PX4 closed-loop
test stayed 8.30 meters apart. The required simulated minimum was 4 meters.

The dedicated six-node acceptance scenario uses one process per node, SF7/BW250
LoRa, CSMA, proactive routing, periodic discovery floods, and one position update
per second. The CLI accepts 4 to 16 nodes for scale experiments, but only the
six-node configuration is an acceptance target.

## Scope boundary

The project does not claim real RF performance, GNSS accuracy, airframe behavior,
or flight safety. A historical two-radio bench result and deployment code remain
in the repository, but hardware integration and flight testing are outside the
current project scope.

## Optional future deployment

The companion service and Heltec bridge are kept as a path for future hardware
work. They are not needed for the software demonstration. Deployment notes are
in [docs/COMPANION_DEPLOYMENT.md](docs/COMPANION_DEPLOYMENT.md), and the archived
hardware plan is in
[docs/HARDWARE_RECOMMENDATION.md](docs/HARDWARE_RECOMMENDATION.md).

```bash
.venv/bin/python -m pip install -e '.[hardware]'
cp config/companion.example.toml config/drone-1.toml
.venv/bin/drone-sims companion --config config/drone-1.toml
```

## Folders

```text
src/drone_sims/   simulation, node processes, routing, and dashboards
tests/            automated tests
config/           companion examples and bench configurations
deploy/           systemd service template
firmware/         optional Heltec V2/V3 bridge firmware
tools/            simulation, PX4, historical bench, and report scripts
requirements/     pass/fail limits
docs/             extra project notes
results/full/     saved results
```
