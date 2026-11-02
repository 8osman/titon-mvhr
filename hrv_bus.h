#pragma once
// Titon HMB RS-485 line parser and decoders. Receive-only. Plain C++ with no
// Arduino dependency so it can be unit tested on a PC.
//
// Framing (1200 8N1, ASCII, CRLF terminated), from the captured decode:
//   request  "AAA R/W sign DDDDD"  10 chars   R/W digit: 1 = read, 0 = write
//   reply    "AAA sign DDDDD"       9 chars
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace hrv {

enum : uint8_t { KIND_INVALID = 0, KIND_REQUEST = 1, KIND_REPLY = 2 };

struct Frame {
  uint8_t kind;
  uint8_t write;     // requests only: 1 = write
  int16_t addr;
  int32_t value;
};

inline bool parseDigits(const char* s, int n, int32_t& out) {
  int32_t v = 0;
  for (int i = 0; i < n; i++) {
    if (s[i] < '0' || s[i] > '9') return false;
    v = v * 10 + (s[i] - '0');
  }
  out = v;
  return true;
}

// Parse one line that has already had CR/LF removed.
inline bool parseLine(const char* s, size_t n, Frame& f) {
  f.kind = KIND_INVALID; f.write = 0; f.addr = 0; f.value = 0;
  int32_t addr = 0, val = 0;
  if (n == 10) {
    if (!parseDigits(s, 3, addr)) return false;
    if (s[3] != '0' && s[3] != '1') return false;
    if (s[4] != '+' && s[4] != '-') return false;
    if (!parseDigits(s + 5, 5, val)) return false;
    f.kind = KIND_REQUEST;
    f.write = (s[3] == '0') ? 1 : 0;
    f.addr = (int16_t)addr;
    f.value = (s[4] == '-') ? -val : val;
    return true;
  }
  if (n == 9) {
    if (!parseDigits(s, 3, addr)) return false;
    if (s[3] != '+' && s[3] != '-') return false;
    if (!parseDigits(s + 4, 5, val)) return false;
    f.kind = KIND_REPLY;
    f.addr = (int16_t)addr;
    f.value = (s[3] == '-') ? -val : val;
    return true;
  }
  return false;
}

// Collects bytes into lines. A gap of more than kGapMs inside a line discards the
// partial line, which resynchronises after stray strings such as the "FW0046-102"
// the Aura-T sends at power-up/power-loss without a CRLF.
class LineAssembler {
 public:
  static const size_t kMax = 24;
  static const uint32_t kGapMs = 50;

  LineAssembler() : n_(0), outLen_(0), last_(0), junk_(0) { out_[0] = 0; }

  // Returns true when a complete non-empty line is available.
  bool feed(uint8_t b, uint32_t now) {
    if (n_ > 0 && (now - last_) > kGapMs) { junk_++; n_ = 0; }
    last_ = now;
    if (b == '\n') {
      size_t len = n_;
      if (len && buf_[len - 1] == '\r') len--;
      n_ = 0;
      if (len == 0) return false;
      memcpy(out_, buf_, len);
      out_[len] = 0;
      outLen_ = len;
      return true;
    }
    if (n_ >= kMax) { junk_++; n_ = 0; }
    buf_[n_++] = (char)b;
    return false;
  }

  const char* line() const { return out_; }
  size_t length() const { return outLen_; }
  uint32_t junk() const { return junk_; }

 private:
  char buf_[kMax];
  char out_[kMax + 1];
  size_t n_, outLen_;
  uint32_t last_;
  uint32_t junk_;
};

// ---- register numbers seen in the decode ----------------------------------
constexpr int REG_HOURS = 70;     // total machine running hours
constexpr int REG_FAN_SUP = 380;  // 0.1 % of drive
constexpr int REG_FAN_EXT = 381;
constexpr int REG_TEMP_SUP = 382; // x10 C (identity not yet verified)
constexpr int REG_TEMP_EXT = 383; // x10 C, extract (verified by heating)
constexpr int REG_STATUS = 384;   // status bit mask
constexpr int REG_SYS = 388;
constexpr int REG_FUNC_S1 = 330;  // S1/S2/S3 function codes
constexpr int REG_FUNC_S2 = 331;
constexpr int REG_FUNC_S3 = 332;

struct StatusBits {
  uint16_t raw;
  uint8_t speed;        // 1..4, or 0 when not exactly one speed bit is set
  bool switchClosed;    // bit 5
  bool overrun;         // bit 9
  bool bypass;          // bit 12
  bool summerboost;     // bit 15
  bool fault;           // any of bits 11, 13, 14
  bool boostInhibit;    // bit 8 (256): seen with the padlock icon, confirmed 2026-11-02
  bool rhControl;       // bit 6 (64): speed is being controlled by the humidity sensor (confirmed 2026-11-02: set point 30 -> 68, set point 100 -> 1)
};

inline StatusBits decodeStatus(int32_t value) {
  StatusBits s;
  uint16_t r = (uint16_t)(value & 0xFFFF);
  s.raw = r;
  int count = 0, idx = 0;
  for (int i = 0; i < 4; i++) if (r & (1u << i)) { count++; idx = i + 1; }
  s.speed = (count == 1) ? (uint8_t)idx : 0;
  s.switchClosed = (r & (1u << 5)) != 0;
  s.overrun = (r & (1u << 9)) != 0;
  s.bypass = (r & (1u << 12)) != 0;
  s.summerboost = (r & (1u << 15)) != 0;
  s.fault = (r & ((1u << 11) | (1u << 13) | (1u << 14))) != 0;
  s.boostInhibit = (r & (1u << 8)) != 0;
  s.rhControl = (r & (1u << 6)) != 0;
  return s;
}

inline const char* funcName(int code) {
  switch (code) {
    case 1: return "Wet room boost";
    case 2: return "Kitchen boost";
    case 3: return "Speed 1 / Setback";
    case 5: return "SUMMERboost disable";
    case 6: return "Speed 4 (unproven)";
    case 7: return "Off (NO)";
    case 8: return "Off (NC)";
    case 9: return "Manual summer bypass";
    case 10: return "Boost inhibit (unproven)";
    default: return "unknown code";
  }
}

// Registers whose values are kept as "settings" (seen in replies).
inline bool isSettingAddr(int a) {
  switch (a) {
    case 28: case 124: case 125: case 160: case 161: case 231: case 232:
    case 240: case 241: case 250: case 253: case 290: case 330: case 331:
    case 332: case 345: case 385:
      return true;
    default:
      return a >= 300 && a <= 307;
  }
}

class SettingsStore {
 public:
  static const int kMax = 32;
  SettingsStore() : n_(0) {}
  // Returns true if the value is new or changed.
  bool set(int addr, int32_t v) {
    for (int i = 0; i < n_; i++) {
      if (addr_[i] == addr) {
        if (val_[i] == v) return false;
        val_[i] = v;
        return true;
      }
    }
    if (n_ >= kMax) return false;
    addr_[n_] = (int16_t)addr; val_[n_] = v; n_++;
    return true;
  }
  bool get(int addr, int32_t& v) const {
    for (int i = 0; i < n_; i++) if (addr_[i] == addr) { v = val_[i]; return true; }
    return false;
  }
  int count() const { return n_; }
  int addrAt(int i) const { return addr_[i]; }
  int32_t valueAt(int i) const { return val_[i]; }
 private:
  int16_t addr_[kMax];
  int32_t val_[kMax];
  int n_;
};

// Build a request frame "AAAWsDDDDD\r\n" (R/W digit: 1 = read, 0 = write).
// out must hold 13 bytes. Returns the length (12), or 0 if the arguments are invalid.
inline size_t buildRequest(char* out, int addr, bool write, int32_t value) {
  if (addr < 0 || addr > 999 || value < 0 || value > 99999) return 0;
  out[0] = (char)('0' + addr / 100);
  out[1] = (char)('0' + (addr / 10) % 10);
  out[2] = (char)('0' + addr % 10);
  out[3] = write ? '0' : '1';
  out[4] = '+';
  for (int i = 0; i < 5; i++) { out[9 - i] = (char)('0' + value % 10); value /= 10; }
  out[10] = '\r'; out[11] = '\n'; out[12] = 0;
  return 12;
}

// Registers this firmware may write. The first group is the relaxed set
// (160/161/300-307/385). The second group (switch functions 330-332 and the other
// EEPROM-backed settings) is OFF unless allowSettings is set at compile time.
inline bool isWritableAddr(int a, bool allowSettings) {
  if (a == 385 || a == 160 || a == 161 || (a >= 300 && a <= 307)) return true;
  if (!allowSettings) return false;
  switch (a) {
    case 330: case 331: case 332: case 231: case 232: case 241:
    case 250: case 253: case 290: case 345: return true;   // 240 and 028 are unknown: never writable
    default: return false;
  }
}

}  // namespace hrv
