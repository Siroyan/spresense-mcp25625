// Host side of the B-G431 CAN link over the MCP25625: bus setup, keepalive,
// command / result exchange and telemetry reassembly. See bg431_protocol.h.
//
// Call poll() as often as possible (no delay() in the caller's loop): the
// board sends a frame every ~0.2 ms while a command runs (1 kHz telemetry =
// 4 frames per ms) and the MCP25625 holds only two received frames.
#pragma once
#include <Arduino.h>
#include <mcp2515.h>
#include "bg431_protocol.h"

class Bg431Host {
 public:
  struct Result {
    bool     valid = false;
    bg431::ResultHeader hdr = {};
    uint8_t  payload[1024];
    template <typename T> const T& as() const { return *reinterpret_cast<const T*>(payload); }
  };

  // What one command() call produced.
  struct Command {
    bool     completed = false;   // CMD_END seen (false: timed out or the board rebooted)
    uint32_t t_start_us = 0, t_end_us = 0;
    Result   result;
  };

  // Called for every reassembled telemetry sample.
  typedef void (*SampleFn)(const bg431::Sample& s, void* ctx);

  Bg431Host(uint8_t cs, uint8_t intPin, uint8_t resetPin);

  // Reset the MCP25625, find the fastest SPI clock that reads back
  // correctly (starting at spi_hz), set 1 Mbps / 75% and the filters, go
  // normal. Prints what it did on `log`. False if the chip does not answer.
  bool begin(uint32_t spi_hz, Print& log);
  uint32_t spiHz() const { return spi_hz_; }

  // Receive everything pending, send the keepalive when due.
  void poll();

  // Send one command line and wait for its CMD_END, polling all the time.
  // `fn` gets the samples stamped between CMD_START and CMD_END.
  bool command(const char* line, Command& out, uint32_t timeout_ms = 10000,
               SampleFn fn = nullptr, void* ctx = nullptr);

  void config(uint8_t key, uint32_t value);
  void estop();

  // Set when a BOOT event arrives (the board lost origin and settings);
  // cleared by the caller.
  bool booted = false;
  bool estopEvent = false;        // ESTOP / WATCHDOG event seen

  // Link health
  uint32_t samples = 0;           // telemetry samples reassembled
  uint32_t seqGaps = 0;           // samples lost (board drop or RX overflow)
  uint32_t rxOverflows = 0;       // MCP25625 receive-buffer overflows (EFLG RXnOVR)
  uint32_t txBusy = 0;            // sends that found all three TX buffers busy

 private:
  bool send(uint32_t id, const void* data, uint8_t len);
  void handle(const can_frame& f);
  void telemFrame(uint8_t k, const uint8_t* d);
  void resultFrame(const uint8_t* d, uint8_t len);
  void writeCnf(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);
  void readCnf(uint8_t out[3]);

  MCP2515* can_ = nullptr;
  uint8_t cs_, int_, reset_;
  uint32_t spi_hz_ = 0;
  uint32_t lastKeepaliveMs_ = 0;

  // telemetry reassembly
  bg431::Sample part_;
  uint8_t partNext_ = 0;          // next expected frame index, 0 = waiting for T1
  int16_t lastSeq_ = -1;
  uint8_t seqSkip_ = 6;           // stale samples after (re)connect, not counted as gaps

  // result reassembly
  Result rx_;
  uint16_t rxGot_ = 0;
  bool rxInHeader_ = true;

  // the command in flight
  Command* cmd_ = nullptr;
  SampleFn fn_ = nullptr;
  void* ctx_ = nullptr;
};
