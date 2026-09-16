# Task 2: Independent node processes and multi-node demo

Build a software mesh demonstration with one operating-system process per drone.
The reduced project includes `CompanionService`, its flight-state and radio port
interfaces, telemetry encoding, and `RadioMedium`. The current scenario simulator
keeps nodes together in one process; it is the reference foundation. The separate
process runtime and mesh demo have been removed.

Implement `run_mesh_demo` in `src/drone_sims/mesh_demo.py`. Begin with six moving
nodes using basic flooding and `control_enabled=False`. Let a parent process own
the seeded radio model and virtual clock; exchange frames, time steps, and
serializable snapshots with isolated node services through local IPC. Add
startup checks, bounded waits, clear worker-error handling, and complete cleanup.
Use task 1's proactive routing once it is ready.

Publish updates through the provided callback so task 3 can build its dashboard.
Agree a shared report format early: seed/configuration, timestamped node
snapshots, packet delivery/latency/freshness data, and events. Node snapshots
already include IDs, state freshness, neighbor count, collision alerts, and stats;
include positions and velocities for plotting. Task 1 will add links and routes.
Add a `mesh-demo` CLI command and Make target once the runtime works.

Done means six distinct worker processes exchange telemetry, repeated seeded
runs reproduce the same simulated results, and a worker failure is reported
without hanging or leaving child processes alive. Test forwarding, malformed
packets, stale telemetry, and zero flight-controller commands. This task supplies
raw demo data; task 3 applies and reports the final acceptance limits.
