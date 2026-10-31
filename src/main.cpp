#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <string.h>

#ifndef CAN_LOOPBACK
#define CAN_LOOPBACK 1
#endif

namespace {
constexpr uint8_t kCsPin = 7;
constexpr uint8_t kIntPin = 8;
constexpr uint8_t kResetPin = 9;
constexpr uint32_t kSpiHz = 1000000;
constexpr uint32_t kIntervalMs = 1000;

// Construct after Arduino and SPI initialization, not during global startup.
MCP2515 &can() {
  static MCP2515 controller(kCsPin, kSpiHz, &SPI);
  return controller;
}

void requireOk(MCP2515::ERROR result, const char *step) {
  if (result == MCP2515::ERROR_OK) {
    return;
  }
  Serial.print("INIT FAIL: ");
  Serial.print(step);
  Serial.print(" error=");
  Serial.println(static_cast<int>(result));
  Serial.println("Check power, JP1, SPI wiring, CS D7 and RESET D9; then reset.");
  while (true) {
    delay(1000);
  }
}

void printFrame(const can_frame &frame) {
  Serial.print("RX ID=0x");
  Serial.print(frame.can_id & CAN_EFF_MASK, HEX);
  Serial.print((frame.can_id & CAN_EFF_FLAG) ? " EXT" : " STD");
  Serial.print(" DLC=");
  Serial.print(frame.can_dlc);
  if (frame.can_id & CAN_RTR_FLAG) {
    Serial.println(" RTR");
    return;
  }
  Serial.print(" DATA=");
  for (uint8_t i = 0; i < frame.can_dlc; ++i) {
    if (frame.data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(frame.data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}
}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t started = millis();
  while (!Serial && millis() - started < 3000) {
    delay(10);
  }

  pinMode(kCsPin, OUTPUT);
  digitalWrite(kCsPin, HIGH);
  pinMode(kIntPin, INPUT);  // Active low; polled, no interrupt handler needed.
  pinMode(kResetPin, OUTPUT);
  digitalWrite(kResetPin, LOW);
  delay(10);
  digitalWrite(kResetPin, HIGH);
  delay(10);
  SPI.begin();  // Extension board: SCK D13, MISO D12, MOSI D11.

  Serial.println("SPRESENSE + MCP25625 / 16 MHz / CAN 500 kbps");
  requireOk(can().reset(), "reset");
  requireOk(can().setBitrate(CAN_500KBPS, MCP_16MHZ), "bitrate");
#if CAN_LOOPBACK
  requireOk(can().setLoopbackMode(), "loopback mode");
  Serial.println("LOOPBACK ready: expect TX, RX and LOOPBACK PASS every second.");
#else
  requireOk(can().setNormalMode(), "normal mode");
  Serial.println("NORMAL ready: TX ID 0x123 every second; receive all CAN IDs.");
#endif
}

void loop() {
  static uint32_t lastTx = millis();
  static uint8_t counter = 0;
  if (millis() - lastTx >= kIntervalMs) {
    lastTx = millis();
    can_frame tx = {};
    tx.can_id = 0x123;  // Standard 11-bit data frame.
    tx.can_dlc = 8;
    const uint8_t payload[] = {counter++, 0x11, 0x22, 0x33,
                              0x44, 0x55, 0x66, 0x77};
    memcpy(tx.data, payload, sizeof(payload));
    const MCP2515::ERROR result = can().sendMessage(&tx);
    Serial.print("TX ");
    Serial.println(result == MCP2515::ERROR_OK ? "queued ID=0x123" : "FAIL");

#if CAN_LOOPBACK
    can_frame rx = {};
    bool received = false;
    const uint32_t started = millis();
    while (result == MCP2515::ERROR_OK && millis() - started < 100) {
      if (can().readMessage(&rx) == MCP2515::ERROR_OK) {
        received = true;
        printFrame(rx);
        break;
      }
      delay(1);
    }
    const bool matches = received && rx.can_id == tx.can_id &&
                         rx.can_dlc == tx.can_dlc &&
                         memcmp(rx.data, tx.data, tx.can_dlc) == 0;
    Serial.println(matches ? "LOOPBACK PASS" : "LOOPBACK FAIL (TX/RX mismatch or timeout)");
#else
    Serial.print("EFLG=0x");
    Serial.println(can().getErrorFlags(), HEX);
#endif
  }

#if !CAN_LOOPBACK
  // Poll both the wired INT signal and the receive status. This also avoids
  // blocking reception if INT is accidentally left disconnected.
  if (digitalRead(kIntPin) == LOW || can().checkReceive()) {
    can_frame rx = {};
    if (can().readMessage(&rx) == MCP2515::ERROR_OK) {
      printFrame(rx);
    }
  }
#endif
  delay(1);
}
