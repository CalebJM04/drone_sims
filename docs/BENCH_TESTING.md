# Heltec/Pi bench testing without a Pixhawk

The bench setup treats the Pi plus Heltec V2 as aircraft 1 and this computer
plus Heltec V3 as aircraft 2. Both boards run the binary bridge firmware at
915 MHz, SF7, 250 kHz, coding rate 4/5, and 5 dBm.

## Quick radio demo

Run these simultaneously. The `--quiet` option prints only the final counts.

On this computer:

```bash
.venv/bin/python tools/demo_lora_bridge.py \
  --device /dev/ttyUSB2 --node-id 2 --seconds 30
```

On the Pi, or through SSH:

```bash
ssh caleb@192.168.50.35 \
  '/opt/drone-sims/.venv/bin/python /opt/drone-sims/tools/demo_lora_bridge.py \
  --device /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --node-id 1 --seconds 30'
```

## Fake Pixhawk

`tools/fake_pixhawk.py` emits PX4-style MAVLink heartbeat, GPS, system-health,
and global-position messages. It can generate a linear NED trajectory, suspend
position messages for a configured interval, inject a GPS jump, and record
velocity setpoints sent by the companion.

The checked-in `config/bench.node1.toml` and `config/bench.node2.toml` connect
the companion processes to local UDP MAVLink endpoints and the real serial
Heltec bridges. Their boot IDs are randomized on every run, packet forwarding
is disabled, and vehicle control is disabled by default.

## Full-stack soak

Run the following commands simultaneously for a one-hour stationary test:

```bash
.venv/bin/python tools/soak_bench_node.py \
  --config config/bench.node2.toml \
  --fake-pixhawk tools/fake_pixhawk.py \
  --target 127.0.0.1:14551 --node-id 2 --north 100 --duration 3600
```

```bash
ssh caleb@192.168.50.35 \
  '/opt/drone-sims/.venv/bin/python /home/caleb/drone-sims-deploy/soak_bench_node.py \
  --config /home/caleb/drone-sims-deploy/bench.node1.toml \
  --fake-pixhawk /home/caleb/drone-sims-deploy/fake_pixhawk.py \
  --target 127.0.0.1:14550 --node-id 1 --north -100 --duration 3600'
```

Never use `--enable-control` with real flight hardware until the separate
props-off and HIL acceptance gates have passed.
