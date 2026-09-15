# Hardware recommendation: three aircraft and two ground stations

> Archived future-work document. The current deliverable is entirely software
> based and does not require aircraft, radios, Raspberry Pis, or flight tests.
> Nothing in this document is part of current project acceptance.

Snapshot date: 2026-09-09. Prices are planning estimates in US dollars before
tax and shipping; confirm stock, local radio rules, and connector variants before
ordering.

## Recommended system

Use three identical Holybro X500 V2 development kits with Pixhawk 6C flight
controllers, one Raspberry Pi 4 Model B (2 GB) companion per aircraft, and one
SX1262 transparent-UART LoRa radio per companion. The Pixhawk remains the sole
flight controller. The Pi reads state and offers velocity setpoints over MAVLink;
it never arms the vehicle or selects Offboard mode.

Ground station 1 is the primary QGroundControl station. It connects to the three
included SiK ground radios through a powered USB hub, using a unique SiK Net ID
and a unique PX4 `MAV_SYS_ID` for each aircraft. Ground station 2 runs
QGroundControl as an observer/safety station and receives a copy of MAVLink over
a wired local network from station 1. Retain a tested USB-hub transfer procedure
so station 2 can take the three physical SiK links after a primary-laptop failure.
This is monitoring redundancy, not two independent command links.

Each aircraft also has an independent RC transmitter/receiver with a hardware
mode/kill switch. During early multi-aircraft tests, use one safety pilot per
aircraft; the two QGroundControl operators do not replace those pilots.

## Bill of materials

| Item | Quantity | Planning unit price | Extended | Notes |
|---|---:|---:|---:|---|
| Holybro X500 V2 PX4 Development Kit, Pixhawk 6C, M10, 915 MHz | 3 | $579 | $1,737 | Includes frame, motors, ESCs, power module, GNSS, and one air/ground SiK pair |
| Raspberry Pi 4 Model B, 2 GB | 3 | $45–65 | $135–195 | More than enough CPU/RAM for this service; Pi 5 adds power and cooling burden without a useful benefit |
| High-endurance 64 GB microSD | 4 | $15–25 | $60–100 | One per aircraft plus a cold spare; keep identical imaged cards |
| 5.1 V, 5 A buck/UBEC, 6–38 V input or equivalent | 4 | $30–40 | $120–160 | One per aircraft plus spare; Pololu D24V50F5 is the reference design |
| Waveshare SX1262 915M LoRa HAT with antenna | 4 | $23.99 | $96 | Three installed plus one spare; connect by USB/CP2102, not the Pi GPIO UART |
| Pi heatsink, vibration-isolated mount, short USB/UART/power harness, ferrites, fuse | 3 sets | $40–60 | $120–180 | Fabricate strain-relieved harnesses; no breadboards in flight |
| 4S 5000 mAh 20C+ LiPo with matching XT60 | 6 | $50–80 | $300–480 | Two packs per aircraft; verify mass and center of gravity |
| PX4-compatible 2.4 GHz RC transmitter/receiver system | 3 | $120–200 | $360–600 | Independent link and positive mode/kill switch per aircraft |
| Dual-channel 4S balance charger | 2 | $100–180 | $200–360 | Include correct balance boards/leads and a field DC source if needed |
| x86-64 ground-station laptop, modern Core i5/Ryzen 5, 16 GB RAM, 512 GB SSD | 2 | $0 existing or $700–1,000 | $0 or $1,400–2,000 | Use Ubuntu 24.04 or another QGC-supported OS; avoid ARM laptops until every required USB driver is qualified |
| Powered 7-port USB hub | 1 | $50–100 | $50–100 | Powers the three SiK ground radios without loading the laptop |
| 5 GHz travel router/gigabit switch, Ethernet leads | 1 set | $100–180 | $100–180 | Prefer wired links between stations; reserve 5 GHz for local IP traffic |
| Ground-station UPS/power banks | 2 | $100–180 | $200–360 | Size against measured laptop/router runtime |
| Props, two motors, two ESCs, arms/landing gear, fasteners | 1 lot | $300–500 | Field-repair stock for the common failure items |
| LiPo bags, voltage checker, smoke stopper, multimeter, extinguisher, tether/test restraint | 1 lot | $250–450 | Required bench and field safety equipment |

Core system subtotal with two existing laptops: approximately **$4,000–$5,500**.
With two new laptops: approximately **$5,400–$7,500**. The wide range is
deliberate: batteries, RC equipment, chargers, and fabricated harnesses should
be selected for local support rather than the lowest web price.

## Positioning decision

The included Holybro M10 is specified at 2.0 m CEP. Two independent receivers can
therefore consume most or all of a 4 m separation envelope before vehicle-control
error, latency, and wind are considered. With the M10 hardware:

- retain the example's 10 m `safety_distance_m` or a larger value established by
  field data;
- begin with one powered aircraft and two static packet sources, then one flying
  aircraft and two parked aircraft;
- do not call the 4 m simulation gate a real-flight safety claim.

For close-envelope research, add three Holybro ZED-F9P RTK rovers and one F9P
base, budgeted at **$850–$1,100 total**. Holybro specifies the F9P rover at about
1 cm + 1 ppm in RTK-fixed mode, while PX4 supports F9P RTK corrections through
QGroundControl. PX4's current documentation says a single QGroundControl base can
theoretically serve multiple rovers but that this use case has not been tested.
Consequently, multi-rover correction delivery, RTK-fix loss behavior, and the
measured relative-position error are explicit acceptance tests—not assumptions.
Do not reduce `measurement_position_sigma_m` until logged field data supports it.

## Aircraft wiring and placement

```text
4S battery
  +-- X500 power module --> Pixhawk 6C --> ESCs/motors
  +-- fused 5.1 V / 5 A buck --> Raspberry Pi 4 USB-C power

Pixhawk GPS1  <--> M10 GNSS (replace with F9P rover for close tests)
Pixhawk TELEM1 <--> SiK air radio <~~~~> SiK ground radio --> primary GCS
Pixhawk TELEM2 TX/RX/GND <--> Pi GPIO UART (3.3 V only; no Pixhawk power)
Pi USB <--> SX1262 LoRa HAT/CP2102 <~~~~> other aircraft LoRa radios
Pi Wi-Fi/Ethernet --> maintenance only during initial flight qualification
```

Power the Pi from its own regulated battery branch; never from a Pixhawk telemetry
port. Tie signal ground at the UART, keep the Pi power lead short, and validate
the regulator under maximum CPU, USB-radio transmit, and servo/ESC transients.
Mount the Pi with airflow and a passive heatsink. Enable the Pi hardware watchdog
and use the supplied systemd unit's software watchdog.

Place GNSS above the high-current wiring with a clear sky view. Separate the SiK
and LoRa antennas as far as the X500 geometry permits, keep both away from GNSS,
and preserve antenna polarization. Both proposed data radios occupy the US 915
MHz region, so simultaneous-transmit receiver desensitization is a real
integration risk. Give the LoRa modem and each SiK pair distinct channel plans,
enable LoRa listen-before-talk, measure packet error while SiK is saturated, and
make coexistence a go/no-go test. If it fails, move routine GCS telemetry to a
validated 5 GHz IP link and reserve SiK for recovery rather than weakening the
collision channel.

## Configuration choices

- Give the aircraft node IDs 1, 2, and 3 and set PX4 `MAV_SYS_ID` to the same
  values. Use a new random boot ID at every companion-service start.
- Survey one shared latitude, longitude, and MSL altitude origin and copy it
  exactly to all three TOML files.
- Synchronize the Pi clocks before arming. Use one explicit
  `mission_epoch_unix_s` for a multi-day campaign.
- Configure every LoRa modem for the same network ID, RF channel, air rate, UART
  rate, and transparent-broadcast mode. Enable hardware LBT; disable hardware
  relay because the service performs TTL-limited forwarding and replay control.
- The Waveshare modem defaults may not match `baud = 115200`. Either program the
  modem UART to 115200 or change the TOML to the actual module rate.
- Keep `control_enabled = false` through bench, props-off, HITL, tethered, and
  single-aircraft tests. The service does not enter Offboard mode for the pilot.
- Use `minimum_gps_fix_type = 3` for M10 tests and require `6` for close-envelope
  work that depends on an RTK-fixed solution.
- Configure and test PX4 data-link, position-loss, battery, RC-loss, geofence, and
  Offboard-loss behavior. A companion crash must cause the selected PX4 failsafe,
  never continued stale motion.

## Ground-station operating model

Ground station 1 owns the three USB SiK radios during normal operation. Configure
each pair with a distinct Net ID and create three QGroundControl serial links.
Forward MAVLink over the isolated field LAN to ground station 2 for independent
display and logging. Do not allow both stations to issue mission/control commands
at the same time; assign one command authority and one safety observer in the
test card.

Keep the field LAN off 2.4 GHz so it does not unnecessarily compete with the RC
links. Record PX4 `.ulg` logs on every flight controller, companion JSON/service
logs on every Pi, and QGroundControl telemetry logs on both laptops. Clock-sync
all five computers so an incident can be reconstructed across logs.

## Procurement references

- X500 V2 kit: https://holybro.com/products/px4-development-kit-x500-v2
- Raspberry Pi 4 specifications: https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/
- PX4 Raspberry Pi companion wiring: https://docs.px4.io/main/en/companion_computer/pixhawk_rpi
- PX4 Offboard behavior: https://docs.px4.io/main/en/flight_modes/offboard
- Waveshare SX1262 915M HAT: https://www.waveshare.com/sx1262-868m-lora-hat.htm?sku=16804
- Pololu D24V50F5 regulator reference: https://www.pololu.com/product/2851
- Holybro M10 specification: https://holybro.com/products/m10-gps
- Holybro F9P specification: https://docs.holybro.com/gps-and-rtk-system/f9p-h-rtk-series/standard-f9p-uart/specification-and-comparison
- PX4 RTK setup and multi-rover caveat: https://docs.px4.io/main/en/gps_compass/rtk_gps
- Holybro multi-SiK setup: https://docs.holybro.com/radio/sik-telemetry-radio-v3/multiple-point-to-point-setup-with-sik-radio
- QGroundControl desktop requirements: https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html
