# Test matrix and evidence boundary

| Project responsibility | Executable verification | Remaining physical test |
|---|---|---|
| Collision math | Geometry tests, sampling oracle, 25k randomized comparisons | Logged GNSS versus independent truth |
| Frozen protocol | Golden bytes, 25k corruptions, CRC, malformed input | Radio/UART electrical integration |
| Restart/replay | Boot ID, rollover, delayed retired-session rejection | Repeated real power-cycle tests |
| Neighbor state | Reorder, capacity, expiry, eviction, 6 s uncertain propagation | Raspberry Pi soak and restart testing |
| LoRa PHY | Exact SF/BW/CR/header/CRC/LDRO airtime, duty-cycle queue | Selected module measurements |
| Radio/channel | Path loss, sensitivity, capture, half duplex, CSMA/ALOHA | Range, interference and antenna tests |
| Routing | Flood, relay, probabilistic, TTL, duplicates, 3-hop failover | Mobile field topology |
| Estimation | Alpha-beta tracking and uncertainty confusion matrix | Receiver-specific noise calibration |
| Avoidance | Seven candidates, multi-threat worst-case scoring, geofence | Safety review and real flight envelope |
| Fault handling | GPS jump, clock skew, restart, blackout, command failure | HIL fault injection |
| Flight response | Dynamics bounds and real PX4 SIH closed loop | Airframe dynamics and failsafes |
| Companion computer | Common-frame conversion, packet ingest/relay, stale-state command suppression | Pi power, thermals, UART, watchdog and HIL |
| Endurance | 300 simulated seconds, 12 nodes, relay routing | Thermal/power/soak testing |
| Acceptance | Frozen JSON thresholds, source-matched evidence, 4 m operational margin and 14-gate readiness report | Hardware gate extension |

## Scenario catalog

- `head_on`, `crossing`, `multi_threat`, and `vertical_clear`: core geometry.
- `no_avoidance` and `command_loss`: required negative controls.
- `noisy`, `gps_jump`, `clock_skew`, `node_restart`, and `telemetry_dropout`:
  estimator and fault behavior.
- `limited_dynamics`: acceleration-constrained response.
- `partition` and `asymmetric`: topology/link faults.
- `congested`: deliberate twelve-node ALOHA collapse.
- `endurance`: 300-second, twelve-node state/network stability run.

The full software campaign uses 100 seeds per scenario and three independent
endurance seeds. The 3 m value remains the collision-detection geometry boundary;
acceptance requires at least 4 m simulated separation to preserve one metre of
software margin before hardware-specific allowances are added.

## Model boundary

The LoRa timing equation is exact for configured modem parameters, but propagation
and CSMA remain abstractions until calibrated against the chosen radio. Motion is
a point-mass velocity model except for the PX4 SIH runs. The final readiness
report therefore lists RF, GNSS, Raspberry Pi, electrical, airframe and flight
tests as explicit hardware-only work.
