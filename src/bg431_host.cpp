#include "bg431_host.h"

#include <SPI.h>
#include <math.h>
#include <string.h>

using namespace bg431;

namespace {
// MCP2515 SPI instructions and registers used directly (the library keeps its
// register access private).
constexpr uint8_t INSTR_WRITE = 0x02, INSTR_READ = 0x03;
constexpr uint8_t INSTR_READ_STATUS = 0xA0;       // bit 0 RX0IF, bit 1 RX1IF
constexpr uint8_t INSTR_READ_RX0 = 0x90, INSTR_READ_RX1 = 0x94;   // from SIDH; clears RXnIF on CS high
constexpr uint8_t REG_CNF3 = 0x28;            // CNF3, CNF2, CNF1 are consecutive
constexpr uint8_t REG_TXB0D0 = 0x36;          // 8 bytes of scratch for the SPI test

// 1 Mbps from the 16 MHz crystal: BRP 0 -> tq = 125 ns, 8 tq per bit:
//   Sync 1 + Prop 2 + PS1 3 | PS2 2  -> sample point 6/8 = 75%, SJW 2
// The library's own 1 Mbps setting (00 D0 82) samples at 62.5%; the board
// samples at 75.3% (1 + 127 + 42 of 170 tq).
constexpr uint8_t CNF1 = 0x40;   // SJW 2 (bits 7:6 = 1), BRP 0
constexpr uint8_t CNF2 = 0x91;   // BTLMODE, SAM 0, PHSEG1 3 (bits 5:3 = 2), PRSEG 2 (bits 2:0 = 1)
constexpr uint8_t CNF3 = 0x01;   // PHSEG2 2

int16_t le16(const uint8_t* p) { return (int16_t)(p[0] | (p[1] << 8)); }
}  // namespace

const char* bg431::statusName(uint8_t s) {
  static const char* const n[] = {"OK", "INVALID_ARG", "REFUSED", "FAILED", "ABORTED", "TIMEOUT", "UNKNOWN_CMD"};
  return s < 7 ? n[s] : "?";
}

const char* bg431::detailName(uint8_t d) {
  static const char* const n[] = {"", "disabled", "sensor_fault", "runaway", "motor_disabled",
                                  "moving", "no_map", "too_far", "i2c", "init_foc"};
  return d < 10 ? n[d] : "?";
}

Bg431Host::Bg431Host(uint8_t cs, uint8_t intPin, uint8_t resetPin)
    : cs_(cs), int_(intPin), reset_(resetPin) {}

void Bg431Host::writeCnf(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3) {
  SPI.beginTransaction(SPISettings(spi_hz_, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_, LOW);
  SPI.transfer(INSTR_WRITE);
  SPI.transfer(REG_CNF3);
  SPI.transfer(cnf3);
  SPI.transfer(cnf2);
  SPI.transfer(cnf1);
  digitalWrite(cs_, HIGH);
  SPI.endTransaction();
}

void Bg431Host::readCnf(uint8_t out[3]) {
  SPI.beginTransaction(SPISettings(spi_hz_, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_, LOW);
  SPI.transfer(INSTR_READ);
  SPI.transfer(REG_CNF3);
  for (int i = 0; i < 3; i++) out[i] = SPI.transfer(0);   // CNF3, CNF2, CNF1
  digitalWrite(cs_, HIGH);
  SPI.endTransaction();
}

// Write/read back pseudo-random bytes through a TX buffer: a clock the
// wiring (and the extension board's level shifter) cannot carry shows up as
// mismatches long before frames go missing.
static bool spiSelfTest(uint8_t cs, uint32_t hz) {
  uint32_t x = 0x12345678;
  for (int round = 0; round < 64; round++) {
    uint8_t w[8], r[8];
    for (int i = 0; i < 8; i++) {
      x = x * 1103515245u + 12345u;
      w[i] = (uint8_t)(x >> 16);
    }
    SPI.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    SPI.transfer(INSTR_WRITE);
    SPI.transfer(REG_TXB0D0);
    for (int i = 0; i < 8; i++) SPI.transfer(w[i]);
    digitalWrite(cs, HIGH);
    digitalWrite(cs, LOW);
    SPI.transfer(INSTR_READ);
    SPI.transfer(REG_TXB0D0);
    for (int i = 0; i < 8; i++) r[i] = SPI.transfer(0);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
    if (memcmp(w, r, 8) != 0) return false;
  }
  return true;
}

bool Bg431Host::begin(uint32_t spi_hz, Print& log) {
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
  pinMode(int_, INPUT);
  pinMode(reset_, OUTPUT);
  digitalWrite(reset_, LOW);
  delay(10);
  digitalWrite(reset_, HIGH);
  delay(10);
  SPI.begin();

  // Fastest clock that passes the self-test, halving down to 1 MHz.
  for (spi_hz_ = spi_hz; spi_hz_ >= 1000000; spi_hz_ /= 2) {
    delete can_;
    can_ = new MCP2515(cs_, spi_hz_, &SPI);
    if (can_->reset() != MCP2515::ERROR_OK) continue;
    bool ok = spiSelfTest(cs_, spi_hz_);
    log.print("SPI ");
    log.print(spi_hz_ / 1000);
    log.println(ok ? " kHz: self-test OK" : " kHz: self-test FAILED");
    if (ok) break;
  }
  if (spi_hz_ < 1000000) return false;

  if (can_->setConfigMode() != MCP2515::ERROR_OK) return false;
  writeCnf(CNF1, CNF2, CNF3);
  uint8_t cnf[3];
  readCnf(cnf);
  if (cnf[0] != CNF3 || cnf[1] != CNF2 || cnf[2] != CNF1) {
    log.println("CNF read-back mismatch");
    return false;
  }

  // Only what the host uses reaches the receive buffers, and everything
  // that arrives back to back goes to RXB0, which rolls over into RXB1 when
  // full -- two frames of slack instead of one:
  //   RXB0, mask 0x74F (ignores bits 4, 5 and 7):
  //     RXF0 0x101 -> telemetry 0x101/111/121/131
  //     RXF1 0x201 -> event 0x201 and result 0x281
  //     (the other IDs these also pass, 0x181.. / 0x211.., are unused)
  //   RXB1: status 0x701 (10 Hz, not urgent)
  // Console text (0x581) and everything else is dropped in hardware.
  if (can_->setFilterMask(MCP2515::MASK0, false, 0x74F) != MCP2515::ERROR_OK ||
      can_->setFilter(MCP2515::RXF0, false, ID_TELEM) != MCP2515::ERROR_OK ||
      can_->setFilter(MCP2515::RXF1, false, ID_EVENT) != MCP2515::ERROR_OK ||
      can_->setFilterMask(MCP2515::MASK1, false, 0x7FF) != MCP2515::ERROR_OK ||
      can_->setFilter(MCP2515::RXF2, false, ID_STATUS) != MCP2515::ERROR_OK ||
      can_->setFilter(MCP2515::RXF3, false, ID_STATUS) != MCP2515::ERROR_OK ||
      can_->setFilter(MCP2515::RXF4, false, ID_STATUS) != MCP2515::ERROR_OK ||
      can_->setFilter(MCP2515::RXF5, false, ID_STATUS) != MCP2515::ERROR_OK)
    return false;
  if (can_->setNormalMode() != MCP2515::ERROR_OK) return false;
  log.println("CAN 1 Mbps, sample point 75%, normal mode");
  lastKeepaliveMs_ = millis();
  return true;
}

bool Bg431Host::send(uint32_t id, const void* data, uint8_t len) {
  can_frame f = {};
  f.can_id = id;
  f.can_dlc = len;
  if (len) memcpy(f.data, data, len);
  uint32_t t0 = millis();
  while (can_->sendMessage(&f) != MCP2515::ERROR_OK) {
    txBusy++;
    if (millis() - t0 > 20) return false;   // nobody ACKing
    poll();
  }
  return true;
}

void Bg431Host::config(uint8_t key, uint32_t value) {
  uint8_t d[5] = {key};
  memcpy(d + 1, &value, 4);
  send(ID_CONFIG, d, 5);
}

void Bg431Host::estop() {
  uint8_t d = 1;
  send(ID_ESTOP, &d, 1);
}

// One received frame in two SPI transactions (READ STATUS, READ RX BUFFER)
// instead of the library's five: the board sends a frame every ~0.2 ms and
// the MCP25625 holds two.
bool Bg431Host::readFrame(can_frame& f) {
  SPI.beginTransaction(SPISettings(spi_hz_, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_, LOW);
  SPI.transfer(INSTR_READ_STATUS);
  uint8_t st = SPI.transfer(0);
  digitalWrite(cs_, HIGH);
  uint8_t instr = (st & 0x01) ? INSTR_READ_RX0 : (st & 0x02) ? INSTR_READ_RX1 : 0;
  if (instr) {      // RXB0 first: under rollover it holds the older frame
    uint8_t h[5];
    digitalWrite(cs_, LOW);
    SPI.transfer(instr);
    for (int i = 0; i < 5; i++) h[i] = SPI.transfer(0);   // SIDH SIDL EID8 EID0 DLC
    uint8_t dlc = h[4] & 0x0F;
    if (dlc > 8) dlc = 8;
    for (int i = 0; i < dlc; i++) f.data[i] = SPI.transfer(0);
    digitalWrite(cs_, HIGH);
    f.can_id = ((uint32_t)h[0] << 3) | (h[1] >> 5);
    f.can_dlc = dlc;
    if (h[1] & 0x08) f.can_id |= CAN_EFF_FLAG;   // extended: not ours, handle() ignores it
  }
  SPI.endTransaction();
  return instr != 0;
}

void Bg431Host::poll() {
  uint32_t now_us = micros();
  if (lastPollUs_ && now_us - lastPollUs_ > maxPollGapUs) maxPollGapUs = now_us - lastPollUs_;
  lastPollUs_ = now_us;
  // INT is low while a frame waits (or an error flag is set).
  for (int n = 0; n < 8 && digitalRead(int_) == LOW; n++) {
    can_frame f;
    if (readFrame(f)) {
      handle(f);
      continue;
    }
    uint8_t eflg = can_->getErrorFlags();
    if (eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
      rxOverflows++;
      can_->clearRXnOVRFlags();
    }
    can_->clearERRIF();     // only the error flag: clearRXnOVR() would drop received frames too
    break;
  }
  uint32_t now = millis();
  if (now - lastKeepaliveMs_ >= 100) {
    lastKeepaliveMs_ = now;
    send(ID_KEEPALIVE, nullptr, 0);
  }
}

void Bg431Host::handle(const can_frame& f) {
  if (f.can_id & CAN_EFF_FLAG) return;
  uint32_t id = f.can_id & CAN_SFF_MASK;
  if (id >= ID_TELEM && id <= ID_TELEM + 0x30 && ((id - ID_TELEM) & 0x0F) == 0 && f.can_dlc == 8) {
    telemFrame((uint8_t)((id - ID_TELEM) >> 4), f.data);
  } else if (id == ID_RESULT) {
    resultFrame(f.data, f.can_dlc);
  } else if (id == ID_EVENT && f.can_dlc >= sizeof(Event)) {
    Event e;
    memcpy(&e, f.data, sizeof(e));
    if (e.type == EVT_BOOT) {
      booted = true;
      lastSeq_ = -1;
      seqSkip_ = 6;
      rxInHeader_ = true;
    } else if (e.type == EVT_ESTOP || e.type == EVT_WATCHDOG) {
      estopEvent = true;
    }
    if (cmd_) {
      if (e.type == EVT_CMD_START && !cmd_->t_start_us) {
        cmd_->t_start_us = e.t_us;
      } else if (e.type == EVT_CMD_END && cmd_->t_start_us) {
        cmd_->t_end_us = e.t_us;
        cmd_->completed = true;
      }
    }
  }
}

void Bg431Host::telemFrame(uint8_t k, const uint8_t* d) {
  // T1..T4 leave the board back to back; a missing one drops the sample.
  if (k == 0) {
    memcpy(&part_.t_us, d, 4);
    part_.seq = d[4];
    uint8_t wf = d[5];
    part_.flags = (wf & 0x0F) | ((uint16_t)(wf >> 4) << FLAG_MODE_SHIFT);
    part_.est_ext = le16(d + 6) * CURRENT_LSB;
    partNext_ = 1;
    return;
  }
  if (k != partNext_) {
    partNext_ = 0;
    return;
  }
  if (k == 1) {
    int32_t a;
    memcpy(&a, d, 4);
    part_.measured_angle = a * ANGLE_LSB;
    memcpy(&part_.target_angle, d + 4, 4);
  } else if (k == 2) {
    part_.target_Iq = le16(d) * CURRENT_LSB;
    part_.measured_Iq = le16(d + 2) * CURRENT_LSB;
    part_.measured_Id = le16(d + 4) * CURRENT_LSB;
    part_.voltage_q = le16(d + 6) * VOLTAGE_LSB;
  } else {
    part_.voltage_d = le16(d) * VOLTAGE_LSB;
    part_.target_velocity = le16(d + 2) * VEL_LSB;
    part_.measured_velocity = le16(d + 4) * VEL_LSB;
  }
  partNext_ = k + 1;
  if (k < 3) return;
  partNext_ = 0;

  samples++;
  if (seqSkip_) {
    seqSkip_--;
    lastSeq_ = -1;           // stale samples queued while nobody listened
  } else {
    if (lastSeq_ >= 0) seqGaps += (uint8_t)(part_.seq - lastSeq_ - 1);
    lastSeq_ = part_.seq;
  }
  // START overtakes samples still queued on the board: keep those stamped after it
  if (cmd_ && fn_ && cmd_->t_start_us && !cmd_->completed &&
      (int32_t)(part_.t_us - cmd_->t_start_us) >= 0)
    fn_(part_, ctx_);
}

void Bg431Host::resultFrame(const uint8_t* d, uint8_t len) {
  if (rxInHeader_) {
    if (len < sizeof(ResultHeader)) return;
    memcpy(&rx_.hdr, d, sizeof(ResultHeader));
    if (rx_.hdr.len > sizeof(rx_.payload)) rx_.hdr.len = sizeof(rx_.payload);
    rxGot_ = 0;
    rxInHeader_ = false;
  } else {
    uint16_t n = len;
    if (rxGot_ + n > rx_.hdr.len) n = rx_.hdr.len - rxGot_;
    memcpy(rx_.payload + rxGot_, d, n);
    rxGot_ += n;
  }
  if (rxGot_ < rx_.hdr.len) return;
  rxInHeader_ = true;
  rx_.valid = true;
  if (cmd_ && cmd_->t_start_us && !cmd_->completed) {
    cmd_->result.hdr = rx_.hdr;
    memcpy(cmd_->result.payload, rx_.payload, rx_.hdr.len);
    cmd_->result.valid = true;
  }
}

bool Bg431Host::command(const char* line, Command& out, uint32_t timeout_ms, SampleFn fn, void* ctx) {
  out.completed = false;
  out.t_start_us = out.t_end_us = 0;
  out.result.valid = false;
  size_t n = strlen(line);
  if (n == 0 || n > MAX_LINE) return false;

  cmd_ = &out;
  fn_ = fn;
  ctx_ = ctx;
  char buf[MAX_LINE + 1];
  memcpy(buf, line, n);
  buf[n++] = '\n';
  for (size_t off = 0; off < n; off += 8) {
    uint8_t len = (uint8_t)((n - off < 8) ? n - off : 8);
    if (!send(ID_CONSOLE_IN, buf + off, len)) break;
  }
  uint32_t t0 = millis();
  while (!out.completed && !booted && millis() - t0 < timeout_ms) poll();
  cmd_ = nullptr;
  fn_ = nullptr;
  return out.completed;
}
