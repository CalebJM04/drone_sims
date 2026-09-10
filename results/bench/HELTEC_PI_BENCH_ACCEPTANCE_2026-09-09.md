# Heltec V2/V3 and Raspberry Pi bench acceptance

Date: 2026-09-09 (updated 2026-09-10)

## Outcome

The available hardware passed the stationary functional bench tests. The
Pixhawk, GNSS, airframe, power system, and motors were not available, so this is
not flight-hardware acceptance.

## Configuration

- Aircraft 1 surrogate: Raspberry Pi and Heltec WiFi LoRa 32 V2 / SX1276
- Aircraft 2 surrogate: workstation and Heltec WiFi LoRa 32 V3 / SX1262
- PHY: 915 MHz, SF7, 250 kHz, coding rate 4/5, private sync word 0x12
- PHY CRC and application CRC16/CCITT-FALSE enabled
- Transmit power: 5 dBm
- Application frame: fixed 36-byte protocol version 2 telemetry
- Production safety: control disabled and packet TTL 0

## Results

| Test | Result | Evidence |
| --- | --- | --- |
| V2 and V3 production firmware builds | PASS | Both bridge targets compiled and flashed successfully |
| Simultaneous 1 Hz bidirectional radio | PASS | 29/30 received in each direction; 96.7% PDR; zero rejected frames |
| 5 Hz per node offered load | CHARACTERIZED | 83/147 and 90/147 received; 56.5% and 61.2% PDR |
| 10 Hz per node offered load | CHARACTERIZED | 52/193 and 79/193 received; 26.9% and 40.9% PDR |
| Binary serial framing and resynchronization | PASS | No self-echo after IRQ fix; raw application frames decoded on both hosts |
| Corrupt host frame rejection | PASS | Corrupt source 77 frame dropped; five following source 78 frames received |
| Fake PX4 heartbeat/GPS/health/position | PASS | Both adapters reported healthy 3D GPS and valid local-NED state |
| Real-LoRa head-on encounter | PASS | Both nodes tracked one neighbor and generated complementary avoidance plans |
| Production control interlock | PASS | Risk plans generated while `commands_sent` remained zero |
| Emulator-only command path | PASS | Fake Pixhawks received streamed zero and non-zero NED velocity setpoints |
| PX4 state dropout | PASS | One stale-state event recorded, telemetry paused, then recovered automatically |
| LoRa peer expiry/recovery | PASS | Neighbor count transitioned 0 -> 1 -> 0 after the six-second expiry window |
| Current software regression suite | PASS | 58/58 unit and integration tests passed on 2026-09-10 |
| Continuous full-stack soak | PARTIAL | Stopped at 946.2 s for end-of-night cutoff; 936/937 received on node 2, 99.9% PDR, zero error/safety counters |
| Source-matched full-stack soak repeat | ACCEPTED PARTIAL | Operator ended the planned one-hour run at 418.0 s; node 2 received 413/414 packets (99.76% estimated PDR), retained one neighbor and healthy fake-PX4 GPS, and recorded zero CRC/protocol rejections, stale-state events, risk events, or commands |
| Raspberry Pi health | PASS | `throttled=0x0`; deployment present; production service disabled/inactive |

The load test supports retaining the current 1 Hz telemetry rate. Five or ten
hertz per aircraft materially overloads this shared profile and should not be
used without a different MAC/PHY design.

The repeat soak used revision `192c1b4` with a clean worktree and matching
source hash on both hosts. Its [machine-readable node 2 record](heltec_pi_soak_node2_2026-09-10.json)
intentionally has `passed: false` because the requested 3,600-second duration was not completed.
The remote runner exited with its SSH terminal before persisting a separate
node 1 summary; no soak, companion, or fake-PX4 processes remained on either
host afterward. The two partial runs are accepted as sufficient pre-hardware
endurance evidence. A longer soak is optional and can be revisited if later
testing exposes a reliability concern.

## Deferred hardware gates

- Physically separate the radios for indoor/outdoor range and obstruction tests.
- Perform an actual USB removal/reconnection and Raspberry Pi power-cycle test.
- Connect Pixhawk TELEM2 and validate electrical wiring, MAVLink setup, heartbeat,
  GNSS health, timestamps, and message rates.
- Replace the example global reference with a surveyed mission reference.
- Run props-off command acceptance, HIL, EMI/vibration, power, and flight tests.

At shutdown there were no running bench/emulator/companion processes. Both
Heltecs remained powered with production bridge firmware in receive/idle mode,
and the production Pi service was disabled and inactive.
