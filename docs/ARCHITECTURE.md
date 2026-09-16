# Architecture of the team foundation

The current simulator runs all nodes in one process on a deterministic virtual
clock. `simulation.py` combines motion, modeled radio delivery, neighbor-state
maintenance, collision assessments, and simulated avoidance. Random behavior
comes from a recorded seed; `campaign.py` repeats scenarios across seeds.

`protocol.py` defines network byte order and units. Forwarding changes TTL and
hop count; `(source, boot_id, sequence)` identifies a packet. `state_table.py`
handles neighbor sequence ordering, restarts, and freshness. `radio.py` and
`lora_phy.py` provide the modeled radio medium, airtime, loss, and congestion.

`collision.py` is the floating-point reference. `fixedpoint.py` is the independent
integer cross-check. `tracking.py`, `planner.py`, and `avoidance.py` support
uncertainty-aware assessments and simulated responses.

`companion.py` retains the hardware-independent single-node service with injected
flight-state and radio ports. It broadcasts/forwards telemetry, tracks peers, and
provides collision alerts and counters through `snapshot()`. Basic routing modes
are flooding, fixed relay, and probabilistic forwarding. Flight control defaults
to disabled. Hardware adapters are preserved on `most-updated`.

The team's target architecture extends this foundation:

```text
simulated trajectories -> independent node services -> parent-owned LoRa model
                                   |
                        link prediction / proactive routes
                                   |
                          snapshots and measured events
                                   |
                    live dashboard / replay / acceptance report
```

`network_awareness.py`, `mesh_demo.py`, and `mesh_visualization.py` contain small
contracts for these unfinished features. See the three task writeups for ownership
and integration details. Core verification does not invoke those contracts.
