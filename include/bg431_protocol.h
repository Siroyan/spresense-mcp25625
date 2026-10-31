// CAN protocol of the B-G431B-ESC1 motor board (SimpleFOC_B-G431, branch
// main / can-75pct): the subset a host needs to run commands, read their
// structured results and the telemetry stream.
//
// Mirrors include/can_link.h and include/cmd_results.h of that repository --
// keep the two in sync. Classic CAN 2.0, 1 Mbps, sample point 75%, standard
// IDs, everything little-endian and packed.
#pragma once
#include <stdint.h>

namespace bg431 {

constexpr uint32_t NODE = 1;

// host -> board
constexpr uint32_t ID_ESTOP      = 0x010 + NODE;   // any payload
constexpr uint32_t ID_CONFIG     = 0x300 + NODE;   // u8 key, u32 value
constexpr uint32_t ID_CONSOLE_IN = 0x600 + NODE;   // command text, '\n' ends a line
constexpr uint32_t ID_KEEPALIVE  = 0x710 + NODE;   // any payload, feeds the watchdog

// board -> host
constexpr uint32_t ID_TELEM      = 0x100 + NODE;   // T1..T4 at +0x00, +0x10, +0x20, +0x30
constexpr uint32_t ID_EVENT      = 0x200 + NODE;
constexpr uint32_t ID_RESULT     = 0x280 + NODE;
constexpr uint32_t ID_STATUS     = 0x700 + NODE;

constexpr uint32_t WATCHDOG_MS   = 500;            // board disables the motor after this much silence
constexpr uint8_t  MAX_LINE      = 20;             // Commander's line limit

enum EventType : uint8_t { EVT_CMD_START = 1, EVT_CMD_END = 2, EVT_ESTOP = 3, EVT_WATCHDOG = 4, EVT_BOOT = 5 };

struct __attribute__((packed)) Event {
  uint8_t  type;
  uint8_t  reserved;
  uint16_t cmd_seq;
  uint32_t t_us;
};

enum ConfigKey : uint8_t {
  CFG_TELEM_PERIOD_US      = 1,   // while a command runs (default 1000), 0 = off
  CFG_WATCHDOG_MS          = 2,
  CFG_TELEM_IDLE_PERIOD_US = 3,   // otherwise (default 20000), 0 = off
  CFG_TX_GAP_US            = 4,   // min spacing of the board's frames (default 200)
  CFG_CONSOLE_OUT          = 5,
};

// ---- command results (0x281): header frame, then `len` payload bytes ----
// ST_ prefix: the Spresense SDK (NuttX) already defines OK.
enum Status : uint8_t {
  ST_OK = 0, ST_INVALID_ARG = 1, ST_REFUSED = 2, ST_FAILED = 3, ST_ABORTED = 4, ST_TIMEOUT = 5,
  ST_UNKNOWN_CMD = 6,
};
enum Detail : uint8_t {
  DET_NONE = 0, DET_DISABLED = 1, DET_SENSOR_FAULT = 2, DET_RUNAWAY = 3, DET_MOTOR_DISABLED = 4,
  DET_MOVING = 5, DET_NO_MAP = 6, DET_TOO_FAR = 7, DET_I2C = 8, DET_INIT_FOC = 9,
};

struct __attribute__((packed)) ResultHeader {
  char     cmd;
  char     sub;
  uint8_t  status;
  uint8_t  detail;
  uint16_t cmd_seq;
  uint16_t len;
};

// payloads used here (the full list is in cmd_results.h)
struct __attribute__((packed)) ResFloat { float value; };                       // T A V R F
struct __attribute__((packed)) ResMotor { char group; char field; uint16_t r; float value; };  // M..
struct __attribute__((packed)) ResMove { uint32_t elapsed_ms; float target; float err; };      // W H
struct __attribute__((packed)) ResAngleMode { float p_angle_limit; float target; };            // G
struct __attribute__((packed)) ResNormalize { float before; float after; float sensor_offset; };  // GN

// Q's payload and one reassembled telemetry sample, in host units
struct __attribute__((packed)) Sample {
  uint32_t t_us;
  float target_Iq, measured_Iq, measured_Id;     // A
  float voltage_q, voltage_d;                    // V
  float target_velocity, measured_velocity;      // rad/s
  float target_angle, measured_angle;            // rad (target NaN outside angle mode / W,H)
  float est_ext;                                 // A equivalent
  uint16_t seq;
  uint16_t flags;                                // FLAG_* below, mode in bits 8..11
};
static_assert(sizeof(Sample) == 48, "Sample must match the Q payload");

constexpr uint16_t FLAG_ENABLED = 1, FLAG_PROFILE_ACTIVE = 2, FLAG_SENSOR_FAULT = 4, FLAG_CMD_BUSY = 8;
constexpr int      FLAG_MODE_SHIFT = 8;

// ---- telemetry on the wire: 4 frames per sample ----
//   T1  u32 t_us | u8 seq | u8 flags (bits 0..3 flags, 4..7 mode) | i16 est_ext [mA]
//   T2  i32 measured_angle [1e-5 rad] | f32 target_angle
//   T3  i16 target_Iq, measured_Iq, measured_Id [mA] | i16 voltage_q [mV]
//   T4  i16 voltage_d [mV] | i16 target_velocity, measured_velocity [0.02 rad/s] | u16 0
constexpr float ANGLE_LSB = 1e-5f, CURRENT_LSB = 1e-3f, VOLTAGE_LSB = 1e-3f, VEL_LSB = 0.02f;

const char* statusName(uint8_t s);
const char* detailName(uint8_t d);

}  // namespace bg431
