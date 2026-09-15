# Heltec V2/V3 LoRa firmware

This project provides board-specific builds from one source for the Heltec WiFi
LoRa 32 V2 (SX1276) and V3 (SX1262). The initial `*_test` environments transmit
and receive the project's exact 36-byte telemetry frame and emit JSON Lines at
115200 baud for a host-side interoperability test.

The frozen test radio profile is:

- 915.0 MHz carrier
- SF7, 250 kHz bandwidth, coding rate 4/5
- private sync word `0x12`
- explicit header and LoRa PHY CRC
- 8-symbol preamble
- 5 dBm transmit power

Attach the correct antenna before powering or transmitting. These settings are
for short-range US bench testing; select a legal carrier and power for the
deployment jurisdiction.

From this directory, build all V2/V3 diagnostic and production images:

```bash
pio run
```

The V2 image uses node ID 2 and the V3 image uses node ID 3. Test firmware is
intentionally different from production bridge firmware: its USB serial output
is human-readable JSON rather than the binary-only stream expected by the
Raspberry Pi service.

Build a production USB-to-LoRa bridge for either board with:

```bash
pio run -e heltec_v2_bridge
pio run -e heltec_v3_bridge
```

Bridge builds use a small binary USB envelope around the CRC-valid 36-byte
application frame. Host-to-bridge records contain the immediate transmitter
node ID. Bridge-to-host records also contain that ID plus RSSI and SNR in tenths,
so the companion can maintain real link-quality history without changing the
frozen telemetry protocol. `SerialFrameRadio` constructs and parses these
records. Reflash existing bridge boards after upgrading the Python service.

The over-air record is 38 bytes: immediate-transmitter ID followed by the exact
36-byte application frame. A small queue, channel-activity detection, and receive
interrupts keep the bridge responsive while avoiding most same-channel
collisions.
