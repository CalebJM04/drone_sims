# Raspberry Pi companion deployment

This procedure is intentionally observation-first. Do not install props while
bringing up power, serial links, or Offboard setpoints.

## Raspberry Pi preparation

Use 64-bit Raspberry Pi OS Lite on a Pi 4. Create a dedicated `drone-sims`
system user, add it to `dialout`, and install the project under
`/opt/drone-sims`. Create the virtual environment there and install the hardware
extra:

```bash
python3 -m venv /opt/drone-sims/.venv
/opt/drone-sims/.venv/bin/python -m pip install -e '/opt/drone-sims[hardware]'
```

In `raspi-config`, disable the login console on the serial port and enable the
hardware serial port. Reboot, then verify that `/dev/serial0` exists. Connect
Pixhawk TELEM2 TX to Pi RX, TELEM2 RX to Pi TX, and ground to ground. Do not
connect the TELEM2 power pin to the Pi.

Connect the LoRa HAT through its CP2102 USB port. Prefer its stable
`/dev/serial/by-id/...` name in the TOML rather than `/dev/ttyUSB0`. Program the
radio first, because its default 9600 baud does not match the example's 115200
baud.

Synchronize time and verify it before every test:

```bash
timedatectl status
```

All three Pis need the same surveyed frame origin and mission epoch. Clock sync,
the reference coordinates, GNSS quality, and the selected safety distance belong
on the pre-arm checklist.

## PX4 preparation

Give each vehicle a unique `MAV_SYS_ID` matching its companion `node.id`.
Configure TELEM2 for MAVLink at 57600 baud (or change both ends together). The
service requests `GLOBAL_POSITION_INT` at 10 Hz, `GPS_RAW_INT` at 5 Hz, and
`SYS_STATUS` at 1 Hz after the heartbeat. It rejects state until PX4 reports a
healthy GPS at or above `minimum_gps_fix_type`.

Configure PX4's RC loss, data-link loss, battery, position-loss, geofence, and
Offboard-loss actions before enabling companion control. In particular, choose
and test `COM_OF_LOSS_T` and `COM_OBL_RC_ACT`. The service deliberately stops
setpoints when state is stale or GNSS health/fix falls below the configured
minimum, allowing that PX4 failsafe to act.

The companion never arms and never changes flight mode. With control enabled it
streams zero velocity when clear and the selected bounded velocity during an
avoidance event. The pilot or mission controller remains responsible for the
deliberate transition into and out of Offboard mode.

## Install the service

Copy one configuration per aircraft and edit its node ID, origin, epoch, device
paths, GNSS threshold, and measured uncertainty:

```bash
sudo install -d -m 0750 -o root -g drone-sims /etc/drone-sims
sudo install -m 0640 -o root -g drone-sims config/companion.example.toml /etc/drone-sims/drone-1.toml
sudo install -m 0644 deploy/drone-sims-companion@.service /etc/systemd/system/drone-sims-companion@.service
sudo systemctl daemon-reload
sudo systemctl enable --now drone-sims-companion@drone-1.service
```

Keep `control_enabled = false` for initial deployment. Inspect the five-second
JSON health snapshots and watchdog behavior:

```bash
systemctl status drone-sims-companion@drone-1.service
journalctl -u drone-sims-companion@drone-1.service -f
```

The status snapshot should show a healthy GPS, the expected fix type, fresh own
state, and increasing receive/transmit counters. Pull power from the Pi during a
props-off test and confirm PX4 performs the configured Offboard-loss response.
Also test LoRa disconnect, GPS-fix downgrade, corrupt serial data, PX4 reboot,
Pi reboot, and UTC clock correction before moving to restrained propulsion tests.

## Promotion sequence

1. UDP loopback on the Pi with no flight controller.
2. PX4 and LoRa connected, props removed, observation only.
3. PX4 HITL/SITL bridge with control enabled.
4. Restrained single-aircraft hover with 10 m field configuration.
5. One flying aircraft plus two stationary, powered peers.
6. Three aircraft with large altitude/horizontal segregation and three pilots.
7. Reduced separation only after RTK-fixed multi-rover and RF-coexistence gates
   pass using measured data.

See `HARDWARE_RECOMMENDATION.md` for the bill of materials, RF coexistence risk,
and positioning decision.
