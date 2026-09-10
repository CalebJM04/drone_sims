# Architecture

The package separates interfaces that should remain stable when simulated
components are replaced by hardware:

```text
PX4 global state ─► common NED frame ─► TelemetryFrame v2 ─► LoRa radio
                                                              │
                                                              ▼
                                                       parser/table
                                                              │
                                                              ▼
                                                      alpha-beta track
                                                              │
                                                              ▼
                                                   uncertainty assessment
                                                              │
                                                              ▼
                                                    multi-threat planner
                                                              │
                                                              ▼
                                                   MAVLink velocity stream
                                                              │
                                                              ▼
                                             PX4 control, estimation, failsafes
```

`collision.py` is the floating-point reference. `fixedpoint.py` is an independent
integer cross-check used to detect numeric regressions in the embedded
calculation.

`protocol.py` owns the frozen network byte order and units. Forwarding changes
only TTL and hop count; `(source, boot_id, sequence)` is the packet identity.

`simulation.py` combines motion, radio deliveries, state maintenance, risk checks,
and command execution on a deterministic virtual clock. All stochastic behavior
comes from a recorded seed.

`campaign.py` repeats complete scenarios across seeds and summarizes distributions
rather than presenting one favorable run.

`companion.py` is the hardware-independent onboard service. `companion_io.py`
connects it to PX4 `GLOBAL_POSITION_INT`, local-NED velocity setpoints, serial
LoRa modems, UDP bench networks, and systemd. All drones must share one surveyed
geodetic origin and synchronized system time. Monotonic time is used locally for
scheduling and expiry so wall-clock adjustments cannot stall the service.
Packet timestamps come from a synchronized shared epoch: UTC midnight for
single-day tests, or the explicitly configured `mission_epoch_unix_s` for longer
campaigns. The protocol can represent about 49.7 days after that epoch.

The companion computer does not arm the aircraft or change flight mode. PX4
remains the authority for stabilization, estimation, geofencing, and failsafes.
When PX4 telemetry is stale, its GPS health bit is false, or its reported fix is
below `minimum_gps_fix_type`, the companion stops sending commands and relies on
the configured PX4 offboard-loss action.

When event logging is enabled, `simulation.py` records position, velocity,
controller state, and separation over time. The main output is the plain-text
`results/full/results.txt` file. An optional replay page can still be made with
`make visualize`.
