# Pre-hardware readiness

Overall simulated status: **PASS (14/14 gates)**

## Acceptance gates

- PASS — `protocol_crc`
- PASS — `fixed_point_equivalence`
- PASS — `uncertainty_recall`
- PASS — `uncertainty_precision`
- PASS — `deterministic_scenarios`
- PASS — `campaign_safety`
- PASS — `endurance_delivery`
- PASS — `network_delivery`
- PASS — `network_latency`
- PASS — `rtl_behavior`
- PASS — `rtl_synthesis`
- PASS — `px4_lifecycle`
- PASS — `px4_closed_loop`
- PASS — `px4_actual_separation`

## Selected network baseline

- fast LoRa profile, CSMA, flooding routing
- 1.0 s telemetry; 38.528 ms/frame
- Mean PDR 0.778; mean latency 515.7 ms

## Hardware-only work still required

- RF range, interference, antenna placement and regional duty-cycle compliance
- GNSS multipath/jamming behavior with the selected receiver and airframe
- FPGA timing closure, power, pin constraints and board-level I/O
- Autopilot/FPGA electrical integration and real flight-controller failsafes
- Propulsion, battery, vibration, mass, weather and flight-test validation

A PASS means the available software, network, RTL behavioral, synthesis-elaboration, and PX4 SIH gates passed. It is not flight certification.
