# Architecture

The primary system is a software-only network testbed. Its interfaces also keep
future hardware integration possible without making hardware part of the current
demonstration:

```text
simulated state ─► companion process ─► TelemetryFrame v2 ─► virtual LoRa hub
                                                              │
                                                              ▼
                                                parser/link-quality table
                                                              │
                                                              ▼
                                                topology look-ahead/routes
                                                              │
                                                              ▼
                                                alpha-beta/risk assessment
                                                   │                  │
                                                   ▼                  ▼
                                           dashboard/alerts     optional planner
                                                                      │
                                                                      ▼
                                                          MAVLink velocity stream
                                                                      │
                                                                      ▼
                                                       PX4 control and failsafes
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

`companion.py` is the hardware-independent node service. In the primary demo,
each node runs that service in its own operating-system process. The parent
process owns the deterministic virtual LoRa hub and exchanges frames and
snapshots with the node processes over local pipes. This separates node state and
failure boundaries while keeping the radio simulation repeatable.

`companion_io.py` contains optional PX4, serial-radio, UDP, and systemd adapters
retained for future work. They are not required by the software demonstration.

`network_awareness.py` predicts link margin and time-to-loss from shared motion,
builds present and future connectivity graphs, and selects deterministic paths.
`routing.py` is shared by the simulator and companion service. Proactive mode
uses a conservative future graph plus periodic discovery floods, so a relay path
can be active before the old direct edge disappears.

`mesh_demo.py` runs four to sixteen companion-service nodes over the simulated
LoRa hub. Six nodes are the validated acceptance configuration. The CLI defaults
to one process per node; an inline backend remains available for fast regression
runs. `mesh_visualization.py` provides both a localhost live dashboard and a
self-contained replay. The demo never arms an aircraft or enables flight
control.

The software-only acceptance path sends no flight-control commands. Optional PX4
integration tests exercise a simulated vehicle and keep PX4 as the authority for
stabilization, estimation, geofencing, and failsafes.

When event logging is enabled, `simulation.py` records position, velocity,
controller state, and separation over time. The main output is the plain-text
`results/full/results.txt` file. An optional replay page can still be made with
`make visualize`.
