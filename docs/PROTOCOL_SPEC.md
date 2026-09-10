# Drone telemetry protocol v2 (frozen simulation baseline)

Status: **frozen for the pre-hardware baseline**. Any incompatible change must
increment the version byte and add a new golden-vector suite. All multibyte
fields use network byte order (big endian). A frame is exactly 36 bytes.

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 1 | magic | `0xD7` |
| 1 | 1 | version | `0x02` |
| 2 | 1 | type | `0x01` telemetry |
| 3 | 1 | flags | bit 0 emergency; bits 1–7 reserved/zero |
| 4 | 1 | TTL | remaining forwards, unsigned |
| 5 | 1 | hops | completed forwards, unsigned |
| 6 | 2 | source | aircraft/node ID, unsigned |
| 8 | 2 | sequence | increments modulo 65536 |
| 10 | 2 | boot ID | random/non-repeating per transmitter boot |
| 12 | 4 | timestamp | milliseconds in the synchronized mission timebase, unsigned |
| 16 | 12 | position | NED x/y/z, three signed 32-bit centimetre values |
| 28 | 6 | velocity | NED vx/vy/vz, three signed 16-bit cm/s values |
| 34 | 2 | CRC | CRC-16/CCITT-FALSE over bytes 0–33 |

CRC parameters are polynomial `0x1021`, initial value `0xFFFF`, no reflection,
no final XOR. A receiver rejects a bad length, magic, version, type, or CRC before
updating state.

The packet identity is `(source, boot_id, sequence)`. Sequence ordering uses the
standard half-range rule: a modulo-65536 difference from 1 through 32767 is
newer. A new boot ID permits sequence to reset; mission time remains shared
across a transmitter restart. Once a receiver
has observed a new boot ID for a source, packets from the retired boot session
are rejected, preventing delayed packets from rolling state backward.

Forwarding does not modify the packet identity or payload state. It decrements
TTL and increments hops, then recomputes the CRC. A frame with TTL zero may be
consumed locally but cannot be forwarded.

## Golden frame

The conformance vector uses source `0x1234`, sequence `0xABCD`, boot ID
`0xBEEF`, timestamp 12345 ms, position `(123, -456, 789)` cm, velocity
`(-100, 200, -300)` cm/s, flags 1, TTL 5 and hops 0:

```text
d702010105001234abcdbeef000030390000007bfffffe3800000315ff9c00c8fed4bce9
```

This byte string is asserted in the unit suite and is the software/radio
integration contract.
