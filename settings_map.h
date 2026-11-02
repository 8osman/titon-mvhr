#pragma once
// Configuration registers of the HMB unit, as read/written by the Aura-T's Setup menu.
// Mapping cross-checked by Greg against Aura-T captures (screen value vs bus write):
//   231  Summer bypass extract threshold, x10 degC (220 = 22.0). Aura-T wrote 231=230.
//   232  Summer bypass supply threshold,  x10 degC (150 = 15.0). Aura-T wrote 232=160.
//   241  Humidity set point,              x10 %RH  (700 = 70.0). Aura-T wrote 241=650, 700.
//   250  Kitchen boost overrun, minutes.                          Aura-T wrote 250=5.
//   253  Wet room boost overrun, minutes. (value on a given board follows the PCB pot,
//        so 31 on the new board is plausible.)                    Aura-T wrote 253=25.
//   345  Filter change interval, months.                          Aura-T wrote 345=5.
//   290  SUMMERboost enable, INVERTED: 0 = enabled, 1 = disabled. Round trip 0>1>0 proven.
//   240  NOT SUMMERboost. Meaning unknown (reads 1). Read only.
//   028  Unknown (reads 0). Read only.
// Ranges are the Aura-T manual's min/max (DO 5666 Iss 01, Configurable Defaults).
#include <stdint.h>

namespace hrv {

struct CfgReg {
  int16_t addr;
  const char* key;      // JSON key / raw sensor key
  const char* label;    // raw sensor name (after "HRV ")
  const char* name;     // number/switch name (after "HRV "), only if writable
  bool writable;
  int16_t scale;        // raw units per displayed unit (10 = tenths)
  int16_t lo, hi;       // allowed raw range
  const char* unit;
  bool invBool;         // on/off switch where raw 0 = ON
};

static const CfgReg kCfgRegs[] = {
  {  28, "r028", "setting 028 unknown",                 nullptr,                           false,  1,   0,    0, "",       false },
  { 231, "r231", "setting 231 bypass extract (x10 C)",  "Summer bypass extract threshold", true,  10, 170,  350, "\xC2\xB0""C", false },
  { 232, "r232", "setting 232 bypass supply (x10 C)",   "Summer bypass supply threshold",  true,  10, 100,  200, "\xC2\xB0""C", false },
  { 240, "r240", "setting 240 unknown (not SUMMERboost)", nullptr,                         false,  1,   0,    0, "",       false },
  { 241, "r241", "setting 241 humidity set point (x10 %)", "Humidity set point",           true,  10, 300, 1000, "%",        false },
  { 250, "r250", "setting 250 kitchen overrun (min)",   "Kitchen boost overrun",           true,   1,   0,   60, "min",      false },
  { 253, "r253", "setting 253 wet room overrun (min)",  "Wet room boost overrun",          true,   1,   0,   60, "min",      false },
  { 290, "r290", "setting 290 SUMMERboost (0=on, 1=off)", "SUMMERboost enabled",           true,   1,   0,    1, "",         true  },
  { 345, "r345", "setting 345 filter interval (months)", "Filter change interval",         true,   1,   1,   24, "months",   false },
};
static const int kNumCfgRegs = (int)(sizeof(kCfgRegs) / sizeof(kCfgRegs[0]));

// Read-only register scan (button "scan registers"): look for filter time remaining, boost
// inhibit state, the third temperature and the humidity reading. Reads only, never writes.
// Hints come from the other Titon protocol document (BM893, a DIFFERENT version, so only a
// hint): filter remaining at 341, boost inhibit at 326, humidity at 036. Edit this list freely.
// Scan 2 (11:42): 320-329, 340-349 and 386-389 are already done (found 320,326,345,346,347,386,
// 387,388). This list covers the blocks next to the live registers 380-385, the gaps in the 33x
// block, and 030-039 (BM893 hint puts humidity near 036). 387 is repeated to watch it change.
// Scan 3 (12:15): the other Titon register list (an ESPHome file Greg found) matches ours at 326
// (boost inhibit) and 290 (inverted SUMMERboost) but has runtime hours at 060 (ours 070) and the
// filter at 341 (ours maybe 346), so some blocks may be shifted by +10 or +5. Its humidity at 036
// reads 180 here (a temperature), so RH may be at 046. 068 is NEVER read (factory reset).
static const int16_t kScanAddrs[] = {
  40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
  50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
  60, 61, 62, 63, 64, 65, 66, 67, 69,
  71, 72, 73, 74, 75, 76, 77, 78, 79,
  150, 151, 152, 153, 154, 155, 156, 157,
  230, 233, 234, 235, 236, 237, 238, 239 };
static const int kNumScan = (int)(sizeof(kScanAddrs) / sizeof(kScanAddrs[0]));

inline const CfgReg* findCfg(int addr) {
  for (int i = 0; i < kNumCfgRegs; i++) if (kCfgRegs[i].addr == addr) return &kCfgRegs[i];
  return nullptr;
}

}  // namespace hrv
