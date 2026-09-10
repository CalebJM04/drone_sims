#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <esp_system.h>

#include <cstring>

namespace {

constexpr size_t kFrameSize = 36;
constexpr uint8_t kMagic = 0xD7;
constexpr uint8_t kVersion = 0x02;
constexpr uint8_t kTelemetryType = 0x01;
#if defined(HELTEC_TEST_MODE)
constexpr uint32_t kMaximumReceiveWaitMs = 1000;
#endif

#if defined(HELTEC_BOARD_V2)
constexpr char kBoardName[] = "heltec-v2-sx1276";
constexpr int kRadioCs = 18;
constexpr int kRadioDio0 = 26;
constexpr int kRadioReset = 14;
constexpr int kRadioDio1 = 35;
constexpr int kRadioSck = 5;
constexpr int kRadioMiso = 19;
constexpr int kRadioMosi = 27;
constexpr int kVext = 21;
SX1276 radio = new Module(kRadioCs, kRadioDio0, kRadioReset, kRadioDio1);
#elif defined(HELTEC_BOARD_V3)
constexpr char kBoardName[] = "heltec-v3-sx1262";
constexpr int kRadioCs = 8;
constexpr int kRadioDio1 = 14;
constexpr int kRadioReset = 12;
constexpr int kRadioBusy = 13;
constexpr int kRadioSck = 9;
constexpr int kRadioMiso = 11;
constexpr int kRadioMosi = 10;
constexpr int kVext = 36;
SX1262 radio = new Module(kRadioCs, kRadioDio1, kRadioReset, kRadioBusy);
#else
#error "Select either HELTEC_BOARD_V2 or HELTEC_BOARD_V3"
#endif

#if defined(HELTEC_TEST_MODE)
uint16_t sequenceNumber = 0;
uint32_t nextTransmitMs = 0;
uint32_t txAttempts = 0;
uint32_t txSuccesses = 0;
uint32_t rxSuccesses = 0;
uint32_t rxRejected = 0;
uint32_t channelDeferrals = 0;
#endif
#if defined(HELTEC_BRIDGE_MODE)
constexpr size_t kTransmitQueueDepth = 8;
uint8_t transmitQueue[kTransmitQueueDepth][kFrameSize] = {};
size_t transmitQueueHead = 0;
size_t transmitQueueCount = 0;
uint8_t serialFrame[kFrameSize] = {};
size_t serialFrameLength = 0;
volatile bool radioPacketReceived = false;
uint32_t nextChannelAttemptMs = 0;
#endif

#if defined(HELTEC_TEST_MODE)
void writeU16Be(uint8_t* destination, uint16_t value) {
  destination[0] = static_cast<uint8_t>(value >> 8);
  destination[1] = static_cast<uint8_t>(value);
}

void writeU32Be(uint8_t* destination, uint32_t value) {
  destination[0] = static_cast<uint8_t>(value >> 24);
  destination[1] = static_cast<uint8_t>(value >> 16);
  destination[2] = static_cast<uint8_t>(value >> 8);
  destination[3] = static_cast<uint8_t>(value);
}
#endif

uint16_t readU16Be(const uint8_t* source) {
  return static_cast<uint16_t>((static_cast<uint16_t>(source[0]) << 8) |
                               source[1]);
}

uint16_t crc16CcittFalse(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

#if defined(HELTEC_TEST_MODE)
void makeTelemetryFrame(uint8_t* frame, uint16_t sequence) {
  std::memset(frame, 0, kFrameSize);
  frame[0] = kMagic;
  frame[1] = kVersion;
  frame[2] = kTelemetryType;
  frame[3] = 0;  // flags
  frame[4] = 0;  // no forwarding during the direct-link test
  frame[5] = 0;
  writeU16Be(frame + 6, NODE_ID);
  writeU16Be(frame + 8, sequence);
  writeU16Be(frame + 10, static_cast<uint16_t>(0xB000U | NODE_ID));
  writeU32Be(frame + 12, millis());
  // A distinct, valid NED position in centimetres for each diagnostic node.
  writeU32Be(frame + 16, static_cast<uint32_t>(NODE_ID * 100));
  const uint16_t crc = crc16CcittFalse(frame, kFrameSize - 2);
  writeU16Be(frame + kFrameSize - 2, crc);
}
#endif

bool validTelemetryFrame(const uint8_t* frame, size_t length) {
  if (length != kFrameSize || frame[0] != kMagic || frame[1] != kVersion ||
      frame[2] != kTelemetryType) {
    return false;
  }
  return readU16Be(frame + kFrameSize - 2) ==
         crc16CcittFalse(frame, kFrameSize - 2);
}

bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

#if defined(HELTEC_TEST_MODE)
void scheduleRegularTransmit() {
  // Jitter prevents two independently booted peers from phase-locking.
  nextTransmitMs = millis() + 900U + (esp_random() % 201U);
}

void printBoot(int16_t state) {
  Serial.printf(
      "{\"event\":\"boot\",\"node\":%u,\"board\":\"%s\","
      "\"radio_state\":%d,\"frequency_mhz\":%.1f,"
      "\"bandwidth_khz\":%.1f,\"sf\":%u,\"cr\":\"4/%u\","
      "\"power_dbm\":%d}\n",
      NODE_ID, kBoardName, state, LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ,
      LORA_SPREADING_FACTOR, LORA_CODING_RATE, LORA_TX_POWER_DBM);
}

void transmitFrame() {
  ++txAttempts;
  const int16_t scanState = radio.scanChannel();
  if (scanState != RADIOLIB_CHANNEL_FREE) {
    ++channelDeferrals;
    nextTransmitMs = millis() + 25U + (esp_random() % 126U);
    Serial.printf(
        "{\"event\":\"defer\",\"node\":%u,\"scan_state\":%d,"
        "\"deferrals\":%lu}\n",
        NODE_ID, scanState, static_cast<unsigned long>(channelDeferrals));
    return;
  }

  uint8_t frame[kFrameSize];
  const uint16_t sentSequence = sequenceNumber++;
  makeTelemetryFrame(frame, sentSequence);
  const int16_t state = radio.transmit(frame, sizeof(frame));
  if (state == RADIOLIB_ERR_NONE) {
    ++txSuccesses;
  }
  Serial.printf(
      "{\"event\":\"tx\",\"node\":%u,\"seq\":%u,\"state\":%d,"
      "\"tx_ok\":%lu,\"tx_attempts\":%lu}\n",
      NODE_ID, sentSequence, state, static_cast<unsigned long>(txSuccesses),
      static_cast<unsigned long>(txAttempts));
  scheduleRegularTransmit();
}

void receiveUntil(uint32_t timeoutMs) {
  uint8_t frame[255] = {};
  const int16_t state = radio.receive(frame, 0, timeoutMs);
  if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    return;
  }
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("{\"event\":\"rx_error\",\"node\":%u,\"state\":%d}\n",
                  NODE_ID, state);
    return;
  }

  const size_t length = radio.getPacketLength(false);
  const bool valid = validTelemetryFrame(frame, length);
  if (!valid) {
    ++rxRejected;
    Serial.printf(
        "{\"event\":\"rx_rejected\",\"node\":%u,\"length\":%u,"
        "\"rejected\":%lu,\"rssi\":%.1f,\"snr\":%.1f}\n",
        NODE_ID, static_cast<unsigned>(length),
        static_cast<unsigned long>(rxRejected), radio.getRSSI(), radio.getSNR());
    return;
  }

  ++rxSuccesses;
  Serial.printf(
      "{\"event\":\"rx\",\"node\":%u,\"from\":%u,\"seq\":%u,"
      "\"length\":%u,\"rssi\":%.1f,\"snr\":%.1f,\"rx_ok\":%lu}\n",
      NODE_ID, readU16Be(frame + 6), readU16Be(frame + 8),
      static_cast<unsigned>(length), radio.getRSSI(), radio.getSNR(),
      static_cast<unsigned long>(rxSuccesses));
}
#endif

#if defined(HELTEC_BRIDGE_MODE)
void IRAM_ATTR onRadioPacketReceived() { radioPacketReceived = true; }

void enqueueTransmit(const uint8_t* frame) {
  // If the host outruns the radio, prefer current position data over stale data.
  if (transmitQueueCount == kTransmitQueueDepth) {
    transmitQueueHead = (transmitQueueHead + 1) % kTransmitQueueDepth;
    --transmitQueueCount;
  }
  const size_t tail =
      (transmitQueueHead + transmitQueueCount) % kTransmitQueueDepth;
  std::memcpy(transmitQueue[tail], frame, kFrameSize);
  const bool queueWasEmpty = transmitQueueCount == 0;
  ++transmitQueueCount;
  if (queueWasEmpty) {
    // Hosts commonly emit telemetry on exact one-second boundaries. Independent
    // jitter prevents peers from phase-locking into a persistent collision or
    // turnaround race before CAD can distinguish which one started first.
    nextChannelAttemptMs = millis() + 20U + (esp_random() % 181U);
  }
}

void resynchronizeSerialFrame() {
  for (size_t index = 1; index < serialFrameLength; ++index) {
    if (serialFrame[index] == kMagic) {
      const size_t remaining = serialFrameLength - index;
      std::memmove(serialFrame, serialFrame + index, remaining);
      serialFrameLength = remaining;
      return;
    }
  }
  serialFrameLength = 0;
}

void ingestHostSerial() {
  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      break;
    }
    const uint8_t byte = static_cast<uint8_t>(value);
    if (serialFrameLength == 0 && byte != kMagic) {
      continue;
    }
    serialFrame[serialFrameLength++] = byte;
    if (serialFrameLength != kFrameSize) {
      continue;
    }
    if (validTelemetryFrame(serialFrame, serialFrameLength)) {
      enqueueTransmit(serialFrame);
      serialFrameLength = 0;
    } else {
      resynchronizeSerialFrame();
    }
  }
}

void forwardReceivedRadioPacket() {
  if (!radioPacketReceived) {
    return;
  }
  radioPacketReceived = false;

  uint8_t frame[255] = {};
  const size_t length = radio.getPacketLength();
  const int16_t state = radio.readData(frame, length);
  if (state == RADIOLIB_ERR_NONE && validTelemetryFrame(frame, length)) {
    Serial.write(frame, kFrameSize);
  }
  // Explicitly re-arm continuous RX and clear the completed packet IRQ. This
  // is required for consistent behavior across both SX1276 and SX1262.
  radioPacketReceived = false;
  radio.startReceive();
}

void transmitQueuedFrame() {
  if (transmitQueueCount == 0 ||
      !deadlineReached(millis(), nextChannelAttemptMs)) {
    return;
  }

  // RxDone, CADDone and TxDone share interrupt lines on these radios. Detach
  // the receive callback while changing modes so non-RX IRQs cannot make the
  // host see a stale RX/TX FIFO as a newly received packet.
  radio.clearPacketReceivedAction();
  radioPacketReceived = false;
  radio.standby();
  const int16_t scanState = radio.scanChannel();
  if (scanState != RADIOLIB_CHANNEL_FREE) {
    nextChannelAttemptMs = millis() + 25U + (esp_random() % 126U);
    radio.setPacketReceivedAction(onRadioPacketReceived);
    radioPacketReceived = false;
    radio.startReceive();
    return;
  }

  const int16_t transmitState =
      radio.transmit(transmitQueue[transmitQueueHead], kFrameSize);
  radioPacketReceived = false;
  if (transmitState == RADIOLIB_ERR_NONE) {
    transmitQueueHead = (transmitQueueHead + 1) % kTransmitQueueDepth;
    --transmitQueueCount;
    nextChannelAttemptMs = millis() + 5U;
  } else {
    nextChannelAttemptMs = millis() + 25U + (esp_random() % 126U);
  }
  radio.setPacketReceivedAction(onRadioPacketReceived);
  radioPacketReceived = false;
  radio.startReceive();
}
#endif

}  // namespace

void setup() {
#if defined(HELTEC_BRIDGE_MODE)
  Serial.setRxBufferSize(1024);
#endif
  Serial.begin(115200);
#if defined(HELTEC_TEST_MODE)
  delay(750);
#else
  delay(100);
#endif

  pinMode(kVext, OUTPUT);
  digitalWrite(kVext, LOW);
  SPI.begin(kRadioSck, kRadioMiso, kRadioMosi, kRadioCs);

#if defined(HELTEC_BOARD_V2)
  const int16_t state = radio.begin(
      LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
      LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER_DBM, 8, 0);
#else
  const int16_t state = radio.begin(
      LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
      LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER_DBM, 8, 1.6, false);
#endif

#if defined(HELTEC_TEST_MODE)
  printBoot(state);
#endif
  if (state != RADIOLIB_ERR_NONE) {
    while (true) {
      delay(1000);
    }
  }
#if defined(HELTEC_TEST_MODE)
  nextTransmitMs = millis() + 250U + (NODE_ID * 173U);
#else
  radio.setPacketReceivedAction(onRadioPacketReceived);
  radio.startReceive();
#endif
}

void loop() {
#if defined(HELTEC_TEST_MODE)
  const uint32_t now = millis();
  if (deadlineReached(now, nextTransmitMs)) {
    transmitFrame();
  } else {
    // Stay in RX continuously until the next local transmit deadline. Short,
    // repeated receive windows can cut a packet in half at every boundary and
    // create artificial direction-dependent loss when two peers phase-lock.
    const uint32_t untilTransmit = nextTransmitMs - now;
    receiveUntil(min(untilTransmit, kMaximumReceiveWaitMs));
  }
#else
  ingestHostSerial();
  forwardReceivedRadioPacket();
  transmitQueuedFrame();
#endif
}
