# Flight-controller SITL boundary

The project uses the official PX4 headless SIH container pinned to digest
`sha256:01866d912ac22ca6119a996b830cf628a6d47dfb60fdccc41cd9f44b62935a44`
as its selected autopilot baseline. `make px4-sitl` starts an isolated container,
connects through real MAVLink, receives live position/state telemetry, arms,
takes off, enters offboard mode while streaming the actual velocity-command
packet, lands, disarms, writes an evidence report, and always stops the container.

`make px4-closed-loop` goes further: a scripted collision track is encoded into
the frozen telemetry packet, delayed by exact LoRa airtime, decoded, filtered,
assessed, planned, and streamed into live PX4. The harness measures separation
using PX4's returned local position.

ArduPilot and Gazebo are not installed. The scope allows PX4 **or** ArduPilot, so
the repository tests one autopilot deeply instead of presenting two shallow mocks.

The simulator does exercise the flight-facing behavior that is independent of a
specific autopilot:

- command transmission and acknowledgement delay;
- command loss, rejection, and timeout;
- avoidance state transitions and cooldown;
- velocity restoration;
- bounded acceleration; and
- minimum-separation outcome.

`drone_sims.integrations.JsonUdpFlightController` is a dependency-free UDP test
double for the transport boundary. A real MAVLink adapter should implement that
same request/acknowledgement boundary once the team selects PX4 or ArduPilot and
freezes the commanded maneuver type.

Run `make readiness` to repeat the tool probe after installing an autopilot SITL.
