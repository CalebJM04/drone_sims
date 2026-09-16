# Test scope on reduced main

`make test` runs the retained core unit and integration suite. It covers collision
geometry, fixed-point arithmetic, telemetry CRC/units, sequence handling and
expiry, LoRa airtime/duty cycle, tracking, planning, simulated avoidance, basic
forwarding, node-service packet rejection, and stale-state behavior. Pending
predictive routing is explicitly rejected rather than silently enabled.

`make quick`, `make verify-ci`, and `make verify` run progressively larger core
verification campaigns. They generate new protocol corruption checks, prediction
and tracking experiments, fixed-point comparisons, basic node-service checks,
deterministic scenario checks, seeded scenario campaigns, and endurance data.
Outputs are local JSON and a short text summary under the selected results folder.

These commands do not run a process mesh, predictive routing, dashboard, hardware,
or PX4 acceptance. Historical completed acceptance evidence is on `most-updated`.
New feature tests and mesh acceptance gates belong to the three teammate tasks.
