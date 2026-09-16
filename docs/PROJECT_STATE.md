# State of the reduced main branch

`main` is now the team's working foundation, with three mesh features left to
implement. The complete previous project is preserved on `most-updated` at commit
`a63595d`, including its source, tests, documentation, and historical results.
The reduction is a normal new commit, so earlier history remains available.

What works now: deterministic motion scenarios, telemetry encoding/CRC checks,
modeled LoRa airtime and packet loss, basic multi-hop forwarding, neighbor
freshness and duplicate handling, collision prediction, tracking, fixed-point
cross-checks, and simulated avoidance. A hardware-independent node service is
available as the foundation for independent processes. Unit/integration tests
and fresh core verification campaigns remain runnable without hardware.

What remains: predictive link monitoring and proactive relay selection; a
six-node demo with one process per node; and the live topology dashboard, replay,
and mesh acceptance report. Small Python entrypoint contracts mark these features
as unimplemented. They are not exposed as working CLI commands. The three task
writeups describe deliverables, interfaces, and completion criteria.

Optional hardware/PX4 adapters, firmware, deployment/bench scripts, completed
dashboards, mesh parameter comparisons, and saved acceptance evidence live on
`most-updated`. Generated results on `main` are local and ignored by Git. A passing
core test run supports the foundation only; it does not certify the unfinished
mesh features. Existing `.venv` or ignored build files may still reflect earlier
installs, so a fresh environment is the cleanest way to reproduce this branch.

Start with `make test`, `make scenario`, and `make quick`. The normal CLI supports
`scenario`, `campaign`, and `verify`. Task 2 can start with flooding, and task 3
can start with fixture data; integrate proactive routing when task 1 is ready.

Validation: the retained suite passed all 53 tests in the existing environment.
In a fresh environment without runtime dependencies, 51 tests passed and the two
optional pymavlink interoperability tests skipped. The CI-sized campaign passed
13/13 deterministic checks and 5/5 node-service checks, detected 2,500/2,500
corrupted packets, and completed 75 seeded scenario runs.
