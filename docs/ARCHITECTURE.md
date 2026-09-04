# Architecture

The package separates interfaces that should remain stable when simulated
components are replaced by hardware:

```text
Kinematic state
      │
      ▼
TelemetryFrame v2 ─► LoRa PHY/MAC/routing ─► parser/table ─► alpha-beta track
                                                                │
                                                                ▼
                                                    uncertainty assessment
                                                                │
                                                                ▼
                                                     multi-threat planner
                                                                │
                                                                ▼
                                                     MAVLink / live PX4 SIH
```

`collision.py` is the floating-point reference. `fixedpoint.py` is the integer
golden model. `rtl/collision_predictor.sv` must agree with the integer model. The
RTL byte parser and bounded table implement the protocol-to-predictor front end.

`protocol.py` owns the frozen network byte order and units. Forwarding changes
only TTL and hop count; `(source, boot_id, sequence)` is the packet identity.

`simulation.py` combines motion, radio deliveries, state maintenance, risk checks,
and command execution on a deterministic virtual clock. All stochastic behavior
comes from a recorded seed.

`campaign.py` repeats complete scenarios across seeds and summarizes distributions
rather than presenting one favorable run.

When event logging is enabled, `simulation.py` also records timestamped NED
position, velocity, controller phase and pair separation samples. `visualization.py`
combines these traces with the verification JSON into a self-contained interactive
dashboard; run `make visualize` and open `results/full/dashboard.html`.
