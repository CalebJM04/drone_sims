# Resilient drone awareness mesh

This college project demonstrates a resilient network for four or more simulated
or props-off drones. Each node shares position and velocity over a multi-hop LoRa
mesh, predicts weakening links, selects relay paths before a direct link breaks,
and reports collision risk. Flight control is optional and disabled in the main
network demonstration.

The project combines simulation with stationary RF validation. It uses PX4
software-in-the-loop and has passed a two-radio LoRa bench test, but it has not
flown on a real drone or exercised four physical radio nodes yet.

## Main network demo

The primary acceptance run starts six actual companion-service instances, feeds
each one a moving fake-flight-controller trajectory, and connects them through the
LoRa RF model. It verifies 4+ node delivery, latency, freshness, proactive route
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
- MAVLink messages and PX4 responses
- Raspberry Pi companion-service state, forwarding, and failure behavior

More details are in [docs/TEST_MATRIX.md](docs/TEST_MATRIX.md) and
[docs/PROTOCOL_SPEC.md](docs/PROTOCOL_SPEC.md). The proposed three-aircraft,
two-ground-station build is in
[docs/HARDWARE_RECOMMENDATION.md](docs/HARDWARE_RECOMMENDATION.md).
Pi installation and PX4 bring-up are covered in
[docs/COMPANION_DEPLOYMENT.md](docs/COMPANION_DEPLOYMENT.md).

## Running it

```bash
cd /home/caleb/school/401/drone_sims
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[sitl,firmware]'
make test
make firmware
make network-matrix
make mesh-acceptance
make px4-sitl
make px4-closed-loop
make verify
make readiness
```

The main output is [results/full/results.txt](results/full/results.txt). It is a
short plain-text summary. The JSON and CSV files in the same folder contain the
full data used to make that summary.

There is also an optional dashboard. Run `make visualize`, then open
`results/full/dashboard.html` in a browser.

## Current result

The regenerated baseline passes 22/22 readiness checks, including eight dedicated
4+ node mesh gates. It includes 1,500
scenario runs, 25,000 damaged-packet tests, companion-service tests, endurance
testing, and two PX4 tests. The closest required simulated scenario stayed 4.36
meters apart; the PX4 closed-loop test stayed 8.31 meters apart. The required
simulated minimum was 4 meters.

The dedicated six-node acceptance scenario uses SF7/BW250 LoRa, CSMA, proactive
routing, periodic discovery floods, and one position update per second.

## What is not done

A stationary two-radio LoRa/Pi bench test has passed at the selected 1 Hz rate;
see [the bench acceptance report](results/bench/HELTEC_PI_BENCH_ACCEPTANCE_2026-09-09.md).
The project still needs range and obstruction testing, real GPS data, Raspberry
Pi power and thermal tests, wiring to a flight controller, HIL tests, and actual
flight tests. Passing the simulation and stationary bench does not mean the
system is safe to fly.

## Raspberry Pi companion service

Install the hardware dependencies and copy the example configuration once per
aircraft. Every aircraft needs a unique node ID and the exact same surveyed
reference latitude, longitude, and altitude.

```bash
.venv/bin/python -m pip install -e '.[hardware]'
cp config/companion.example.toml config/drone-1.toml
.venv/bin/drone-sims companion --config config/drone-1.toml
```

The service defaults to observation only. After props-off and HIL testing, add
`--enable-control` to stream velocity setpoints. It never arms the aircraft or
changes flight mode. PX4 must be configured separately for Offboard mode and an
appropriate offboard-loss action. If PX4 telemetry becomes stale, the service
stops setpoints so PX4 can invoke that failsafe. It also suppresses state and
commands unless PX4 reports a healthy GPS and at least the configured fix type;
set `minimum_gps_fix_type = 6` when an RTK-fixed solution is required.

The example hardware configuration uses a 10 m safety distance. The X500 kit's
M10 GNSS is specified at 2 m CEP, so the simulation's 3 m design distance is not
an acceptable initial field-test envelope. Use validated RTK-fixed positioning
before attempting the close-envelope tests described by the simulation.

The serial LoRa adapter uses a binary bridge envelope around the frozen 36-byte
telemetry frame. It carries the immediate transmitter ID, RSSI, and SNR to the
companion for link awareness. Existing Heltec bridges must be reflashed after
upgrading. Set `radio.transport = "udp"` for bench/SITL tests.
Raspberry Pi system clocks must be synchronized. For a single-day test, a zero
`mission_epoch_unix_s` uses UTC midnight. For multi-day campaigns, configure the
same explicit Unix epoch on every aircraft; the 32-bit millisecond field permits
about 49.7 days from that epoch.

## Folders

```text
src/drone_sims/   simulation and Raspberry Pi companion service
tests/            automated tests
config/           companion examples and bench configurations
deploy/           systemd service template
firmware/         Heltec V2/V3 radio bridge firmware
tools/            PX4, radio bench, and report scripts
requirements/     pass/fail limits
docs/             extra project notes
results/full/     saved results
```
