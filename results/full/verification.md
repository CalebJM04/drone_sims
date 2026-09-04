# Simulation Verification Report

Deterministic acceptance: **13/13 passed**

## Deterministic checks

- PASS — `head_on_detected`
- PASS — `head_on_avoided`
- PASS — `unmitigated_head_on_collides`
- PASS — `three_hop_failover`
- PASS — `command_loss_times_out`
- PASS — `vertical_separation_no_alarm`
- PASS — `aloha_congestion_detected`
- PASS — `crossing_avoided`
- PASS — `simultaneous_threats_avoided`
- PASS — `gps_jump_recovered_safely`
- PASS — `clock_skew_recovered_safely`
- PASS — `reboot_sequence_reset_accepted`
- PASS — `telemetry_dropout_survived`

## Stress verification

- Protocol corruptions detected: 25000/25000
- Noisy prediction accuracy: 0.8965
- Noisy prediction precision/recall: 0.8413 / 0.7256
- Raw vs filtered uncertainty-aware recall: 0.7360 / 0.9080
- Fixed-point classification mismatches away from boundaries: 0/24950
- Executed RTL vectors: 5000 (PASS)
- 300-second endurance seeds: 3; worst PDR 0.9965

## Scenario campaign

| Scenario | Runs | Mean PDR | P95 latency (ms) | Worst separation (m) | Avoidance success |
|---|---:|---:|---:|---:|---:|
| head_on | 100 | 0.775 | 574.6 | 8.12 | 1.000 |
| no_avoidance | 100 | 0.703 | 533.1 | 0.00 | 0.000 |
| noisy | 100 | 0.541 | 523.8 | 5.46 | 1.000 |
| command_loss | 100 | 0.704 | 529.7 | 0.00 | 0.000 |
| limited_dynamics | 100 | 0.756 | 552.5 | 5.21 | 1.000 |
| crossing | 100 | 0.954 | 305.7 | 4.36 | 1.000 |
| vertical_clear | 100 | 0.972 | 267.4 | 8.00 | 1.000 |
| partition | 100 | 0.695 | 400.1 | 4.63 | 1.000 |
| congested | 100 | 0.008 | 111.7 | 0.00 | 0.230 |
| asymmetric | 100 | 0.583 | 331.0 | 20.00 | 1.000 |
| multi_threat | 100 | 0.961 | 490.9 | 8.12 | 1.000 |
| gps_jump | 100 | 0.955 | 305.8 | 8.12 | 1.000 |
| clock_skew | 100 | 0.971 | 461.1 | 8.12 | 1.000 |
| node_restart | 100 | 0.955 | 305.8 | 8.12 | 1.000 |
| telemetry_dropout | 100 | 0.972 | 267.4 | 8.12 | 1.000 |

## Live PX4 SIH

- Lifecycle checks: **12/12 passed**
- Simulated takeoff altitude gain: 2.96 m
- Response to streamed lateral avoidance command: 4.15 m
- Heartbeat, telemetry, arm, takeoff, offboard, lateral response, land, on-ground state, and disarm were observed through live MAVLink.

## External integration readiness

- RTL simulator installed: **True**
- Native PX4 + Gazebo installed: **False**
- Official PX4 SIH container passed: **True**
- ArduPilot SITL installed: **False**
- pymavlink installed: **True**

Unavailable external tools are reported, not treated as successful simulation.
