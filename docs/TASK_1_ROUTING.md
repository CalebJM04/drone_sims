# Task 1: Predictive link monitoring and proactive relay routing

Build the part of the mesh that notices a radio link is weakening and selects a
relay path before the direct connection disappears. The reduced project already
provides drone position/velocity, a LoRa radio model, neighbor state tracking, and
basic flooding, fixed-relay, and probabilistic forwarding. Prediction and
proactive path selection are unimplemented.

Start in `src/drone_sims/network_awareness.py`. Implement `predict_link` using the
supplied `LinkPrediction` fields and `planned_routes` returning destination IDs
mapped to ordered node paths. Track per-sender packet delivery, RSSI/SNR, and
last-heard age using `RadioReception` metadata. Add proactive forwarding to
`RoutingPolicy` and expose link quality, link predictions, and routes through
`CompanionService.snapshot()`. Agree the new snapshot fields with tasks 2 and 3;
keep packet encoding and packet identity unchanged.

Done means tests demonstrate stable links, links moving out of range, unavailable
routes, stale neighbors, and deterministic relay selection. A moving topology
must switch to a usable relay before losing the direct link, and duplicate
packets must not inflate link-quality metrics. Integrate with task 2's demo;
keep the existing basic forwarding tests passing. This task owns the routing
algorithms and their tests, while task 3 owns the final acceptance report.
