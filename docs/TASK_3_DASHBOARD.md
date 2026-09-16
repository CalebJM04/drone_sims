# Task 3: Live topology dashboard and mesh acceptance reporting

Build a clear presentation of the simulated mesh and a report that determines
whether it meets agreed requirements. The reduced project saves scenario events
and position traces and provides node-service snapshots plus core verification
JSON. Completed dashboards, mesh acceptance reports, parameter comparison tools,
and historical results have been removed.

Start in `src/drone_sims/mesh_visualization.py` with `build_mesh_dashboard`. Show
node positions, current/weakening links, selected relay paths, collision alerts,
and delivery/latency/freshness metrics. Add a localhost live view fed by task 2's
update callback and a self-contained HTML replay saved from its final report.
Begin with small fixture snapshots while the process demo is being built.
Coordinate the shared snapshot/report format with tasks 1 and 2.

Define explicit software mesh acceptance limits in a new requirements file, then
calculate named pass/fail checks from demo measurements. Cover delivery, latency,
neighbor freshness, pre-break route handoff, packet integrity, six independent
nodes, and observation-only operation. Mark unavailable evidence as pending or
failed; do not present core simulation checks as completed mesh acceptance.
Save concise text and JSON reports, and add a `mesh-acceptance` Make target whose
exit status reflects the result.

Done means a live demo and saved replay show the same data, the replay opens
without external dependencies, and tests exercise both passing and failing
acceptance reports. Explain metric definitions and chosen limits in the demo
instructions. This task owns presentation and acceptance reporting; tasks 1 and 2
own routing behavior and the process runtime.
