// SPRESENSE + MCP25625 host for the B-G431B-ESC1 motor board over CAN.
//
// Runs SimpleFOC_B-G431's how_to_jump.md procedure -- origin, max-speed
// settings, then "one output rev (H) + normalize (GN)" N times -- the same
// way test/serial_diagnostics/jump_rev.py does from a PC, and measures each
// rev from the board's 1 kHz telemetry.
//
// Operated from the USB serial monitor (115200), one command per line:
//   jump [n]        origin + settings + n revs (default 3); any key aborts (E-stop)
//   jumpk [n]       the same without setting the origin (keep the current one)
//   c <command>     send one board command (e.g. "c Q", "c MVP") and print its result
//   s               E-stop
//   stat            link counters
#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "bg431_host.h"

using namespace bg431;

namespace {
constexpr uint8_t kCsPin = 9;
constexpr uint8_t kIntPin = 8;
constexpr uint8_t kResetPin = 7;
#ifndef BG431_SPI_HZ
#define BG431_SPI_HZ 8000000   // MCP25625 max 10 MHz; begin() halves it until the self-test passes
#endif

constexpr float kGearRatio = 13.78f;
constexpr float kRev = kGearRatio * 2.0f * (float)M_PI;   // one output rev on the motor shaft

// how_to_jump.md step 2, and what to restore afterwards
const char* const kMoveSettings[] = {"A10000", "V460", "MVR1000", "MVP0.05"};
const char* const kDefaults[] = {"A1000", "V145", "MVR150", "MVP0.02"};
constexpr int kDirection = -1;          // negative: the tested direction
constexpr uint32_t kHoldTimeoutMs = 2500;
constexpr float kMaxErr = 0.2f;         // rad: a rev ending farther off stops the run
constexpr float kMaxCurrent = 11.0f;    // A: peak above this stops the run

Bg431Host host(kCsPin, kIntPin, kResetPin);
Bg431Host::Command cmd;

// ---- one rev, measured from the telemetry ----
struct RevStats {
  float target = 0;
  float arrival_ms = NAN;      // first crossing of the target (interpolated)
  float beyond = 0;            // farthest past the target [rad]
  float iq_peak = 0, v_peak = 0, vel_max = 0;
  uint32_t n = 0;
  uint32_t t0 = 0;             // CMD_START time
  bool have_prev = false;
  float prev_t = 0, prev_a = 0;
};

void onSample(const Sample& s, void* ctx) {
  RevStats& r = *static_cast<RevStats*>(ctx);
  if (r.n++ == 0) r.t0 = s.t_us;
  float t = (float)(uint32_t)(s.t_us - r.t0) / 1000.0f;
  float a = s.measured_angle;
  float past = (a - r.target) * kDirection;
  if (isnan(r.arrival_ms) && past >= 0 && r.have_prev) {
    float span = a - r.prev_a;
    float f = span != 0 ? (r.target - r.prev_a) / span : 0;
    r.arrival_ms = r.prev_t + f * (t - r.prev_t);
  }
  if (past > r.beyond) r.beyond = past;
  r.iq_peak = fmaxf(r.iq_peak, fabsf(s.measured_Iq));
  r.v_peak = fmaxf(r.v_peak, hypotf(s.voltage_q, s.voltage_d));
  r.vel_max = fmaxf(r.vel_max, fabsf(s.measured_velocity));
  r.prev_t = t;
  r.prev_a = a;
  r.have_prev = true;
}

// ---- helpers ----
void printResult(const char* line) {
  Serial.print("  ");
  Serial.print(line);
  Serial.print(" -> ");
  if (!cmd.completed || !cmd.result.valid) {
    Serial.println(host.booted ? "board rebooted" : "no answer");
    return;
  }
  const ResultHeader& h = cmd.result.hdr;
  Serial.print(statusName(h.status));
  if (h.detail) {
    Serial.print(" (");
    Serial.print(detailName(h.detail));
    Serial.print(")");
  }
  // values of the payloads this program cares about
  switch (h.cmd) {
    case 'Q': {
      const Sample& q = cmd.result.as<Sample>();
      Serial.print(" angle=");
      Serial.print(q.measured_angle, 4);
      Serial.print(" vel=");
      Serial.print(q.measured_velocity, 2);
      Serial.print(" Iq=");
      Serial.print(q.measured_Iq, 3);
      break;
    }
    case 'M':
      if (h.len >= sizeof(ResMotor)) {
        Serial.print(" value=");
        Serial.print(cmd.result.as<ResMotor>().value, 4);
      }
      break;
    case 'G':
      if (h.sub == 'N' && h.len >= sizeof(ResNormalize)) {
        Serial.print(" after=");
        Serial.print(cmd.result.as<ResNormalize>().after, 4);
      } else if (h.len >= sizeof(ResAngleMode)) {
        Serial.print(" target=");
        Serial.print(cmd.result.as<ResAngleMode>().target, 4);
      }
      break;
    case 'H':
    case 'W':
      if (h.len >= sizeof(ResMove)) {
        Serial.print(" elapsed_ms=");
        Serial.print(cmd.result.as<ResMove>().elapsed_ms);
        Serial.print(" err=");
        Serial.print(cmd.result.as<ResMove>().err, 4);
      }
      break;
    default:
      if (h.len == sizeof(ResFloat)) {
        Serial.print(" value=");
        Serial.print(cmd.result.as<ResFloat>().value, 4);
      }
  }
  Serial.println();
}

// Run one board command; false unless it completed with status OK.
bool run(const char* line, uint32_t timeout_ms = 10000, Bg431Host::SampleFn fn = nullptr, void* ctx = nullptr) {
  host.command(line, cmd, timeout_ms, fn, ctx);
  printResult(line);
  return cmd.completed && cmd.result.valid && cmd.result.hdr.status == ST_OK;
}

// Any key but a line ending (the rest of the "jump" line's CR LF) aborts.
bool abortRequested() {
  bool key = false;
  while (Serial.available()) {
    int c = Serial.read();
    if (c != '\r' && c != '\n') key = true;
  }
  if (key) {
    host.estop();
    Serial.println("!!! key pressed: E-stop sent. Re-enable with G then ME1 after checking the position");
    return true;
  }
  return false;
}

void stopDrive(const char* why) {
  Serial.print("!!! stopping: ");
  Serial.println(why);
  if (!host.booted && run("ME0")) Serial.println("    drive disabled (ME0). Re-enable with G then ME1");
}

bool setOrigin() {
  Serial.println("== 1. origin (current position)");
  if (!run("MC1") || !run("ME1") || !run("MSM0") || !run("Q")) return false;
  char line[24];
  snprintf(line, sizeof(line), "MSM%.4f", cmd.result.as<Sample>().measured_angle);
  return run(line) && run("Q") && run("G");
}

void jump(int count, bool keepOrigin) {
  host.booted = false;
  host.estopEvent = false;
  float arr_sum = 0, arr_min = 1e9, arr_max = 0;
  int done = 0, ok = 0;

  bool good = keepOrigin ? (run("ME1") && run("G")) : setOrigin();
  if (good) {
    Serial.println("== 2. max-speed settings");
    for (const char* c : kMoveSettings) good = good && run(c);
  }
  for (int i = 1; good && i <= count; i++) {
    if (abortRequested()) break;
    uint32_t t = millis();
    while (millis() - t < 500) host.poll();

    RevStats rs;
    rs.target = kDirection * kRev;
    char line[24];
    snprintf(line, sizeof(line), "H%.4f,%lu", rs.target, (unsigned long)kHoldTimeoutMs);
    run(line, kHoldTimeoutMs + 10000, onSample, &rs);

    if (!cmd.completed || !cmd.result.valid) {
      stopDrive(host.booted ? "board rebooted" : "no result for H");
      break;
    }
    const ResultHeader& h = cmd.result.hdr;
    const ResMove& m = cmd.result.as<ResMove>();
    Serial.print("     #");
    Serial.print(i);
    Serial.print(": arrival ");
    Serial.print(rs.arrival_ms, 1);
    Serial.print(" ms (");
    Serial.print(isnan(rs.arrival_ms) ? NAN : 1000.0f / rs.arrival_ms, 2);
    Serial.print(" rps)  overshoot ");
    Serial.print(rs.beyond / kGearRatio * 180.0f / (float)M_PI, 1);
    Serial.print(" deg  settle ");
    Serial.print(m.elapsed_ms);
    Serial.print(" ms  err ");
    Serial.print(m.err * 1000.0f, 0);
    Serial.print(" mrad  |Iq|pk ");
    Serial.print(rs.iq_peak, 1);
    Serial.print(" A  |V|pk ");
    Serial.print(rs.v_peak, 2);
    Serial.print(" V  |vel|max ");
    Serial.print(rs.vel_max, 0);
    Serial.print(" rad/s  samples ");
    Serial.println(rs.n);

    if (h.status != ST_OK && h.status != ST_TIMEOUT) { stopDrive("H did not finish normally"); break; }
    if (fabsf(m.err) > kMaxErr) { stopDrive("final error too large"); break; }
    if (rs.iq_peak > kMaxCurrent) { stopDrive("current peak too high"); break; }
    done++;
    if (h.status == ST_OK) ok++;
    if (!isnan(rs.arrival_ms)) {
      arr_sum += rs.arrival_ms;
      arr_min = fminf(arr_min, rs.arrival_ms);
      arr_max = fmaxf(arr_max, rs.arrival_ms);
    }

    bool normalized = false;
    for (int k = 0; k < 10 && !normalized; k++) {
      normalized = run("GN");
      if (!normalized) {
        t = millis();
        while (millis() - t < 200) host.poll();   // REFUSED (moving): wait for the rotor to stop
      }
    }
    if (!normalized) { stopDrive("GN refused"); break; }
  }

  if (!host.booted) {
    Serial.println("== restore defaults");
    for (const char* c : kDefaults) run(c);
  }
  Serial.print("== ");
  Serial.print(done);
  Serial.print(" revs, ");
  Serial.print(ok);
  Serial.print(" settled OK");
  if (done) {
    Serial.print(", arrival min/mean/max ");
    Serial.print(arr_min, 1);
    Serial.print("/");
    Serial.print(arr_sum / done, 1);
    Serial.print("/");
    Serial.print(arr_max, 1);
    Serial.print(" ms");
  }
  Serial.println();
}

void printStats() {
  Serial.print("SPI ");
  Serial.print(host.spiHz() / 1000);
  Serial.print(" kHz, samples ");
  Serial.print(host.samples);
  Serial.print(", seq gaps ");
  Serial.print(host.seqGaps);
  Serial.print(", RX overflows ");
  Serial.print(host.rxOverflows);
  Serial.print(", TX busy ");
  Serial.print(host.txBusy);
  Serial.print(", max poll gap ");
  Serial.print(host.maxPollGapUs);
  Serial.println(" us");
  host.maxPollGapUs = 0;
}

void handleLine(char* line) {
  if (strncmp(line, "jumpk", 5) == 0) {
    int n = atoi(line + 5);
    jump(n > 0 ? n : 3, true);
  } else if (strncmp(line, "jump", 4) == 0) {
    int n = atoi(line + 4);
    jump(n > 0 ? n : 3, false);
  } else if (line[0] == 'c' && line[1] == ' ') {
    run(line + 2);
  } else if (strcmp(line, "s") == 0) {
    host.estop();
    Serial.println("E-stop sent");
  } else if (strcmp(line, "stat") == 0) {
    printStats();
  } else if (line[0]) {
    Serial.println("commands: jump [n] | jumpk [n] | c <board command> | s | stat");
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println("SPRESENSE + MCP25625 -> B-G431B-ESC1 (CAN 1 Mbps)");
  if (!host.begin(BG431_SPI_HZ, Serial)) {
    Serial.println("INIT FAIL: check power, JP1, SPI wiring, CS D9 and RESET D7; then reset.");
    while (true) delay(1000);
  }
  Serial.println("ready: jump [n] | jumpk [n] | c <board command> | s | stat");
}

void loop() {
  static char line[64];
  static uint8_t len = 0;
  host.poll();                 // keepalive + receive; no delay() here
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      line[len] = 0;
      if (len) handleLine(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}
