#pragma once
// Speed-table save sequence planner. Plain C++ with no Arduino dependency.
//
// The stored speed table is 8 registers in 0.1 % units:
//   300/301 speed 4 supply/extract    302/303 speed 2    304/305 speed 3    306/307 speed 1
// The Aura-T's own Save is: W160=1, W300..W307 (~0.5 s apart), W161=0. This plan
// brackets that with a full read of 300..307 before (so nothing is written unless the
// whole table was read first and a backup taken) and a full read after (verify).
// NOTE: the minimal form without a W160=0 commissioning session is UNPROVEN on the unit.
#include <stdint.h>
#include "settings_map.h"

namespace hrv {

constexpr int kTableRegs = 8;
constexpr int kTableBase = 300;
constexpr int kMaxSteps = 72;

enum StepKind : uint8_t { STEP_READ = 0, STEP_WRITE_CONST = 1, STEP_WRITE_TABLE = 2 };

struct Step {
  StepKind kind;
  int16_t addr;
  int16_t value;      // STEP_WRITE_CONST only
  uint16_t delayMs;   // wait this long before sending the step
};

struct Plan {
  Step s[kMaxSteps];
  int n;
  int preReads;       // number of leading read steps (the table snapshot)
};

inline void planAdd(Plan& p, StepKind k, int addr, int value, int delayMs) {
  if (p.n >= kMaxSteps) return;
  Step& st = p.s[p.n++];
  st.kind = k; st.addr = (int16_t)addr; st.value = (int16_t)value; st.delayMs = (uint16_t)delayMs;
}

inline void buildReadPlan(Plan& p) {
  p.n = 0; p.preReads = kTableRegs;
  for (int i = 0; i < kTableRegs; i++) planAdd(p, STEP_READ, kTableBase + i, 0, i ? 300 : 0);
}

// Read every candidate configuration register plus the three switch functions (never writes).
inline void buildSettingsReadPlan(Plan& p) {
  p.n = 0; p.preReads = 0;
  for (int i = 0; i < kNumCfgRegs; i++) planAdd(p, STEP_READ, kCfgRegs[i].addr, 0, p.n ? 300 : 0);
  for (int a = 330; a <= 332; a++) planAdd(p, STEP_READ, a, 0, 300);
}

// Read-only scan of kScanAddrs.
inline void buildScanPlan(Plan& p) {
  p.n = 0; p.preReads = 0;
  for (int i = 0; i < kNumScan; i++) planAdd(p, STEP_READ, kScanAddrs[i], 0, i ? 200 : 0);
}

// One configuration register: read the old value, write the new one, optionally send
// W161=0, then read it back. 161=0 is what the Aura-T sends whenever it leaves a menu,
// even when nothing changed (per Greg's captures). It is a "menu closed" marker, NOT a
// proven commit. A setting takes effect on its own write. Whether it needs 161=0 to
// survive a power cycle is untested.
inline void buildCfgWritePlan(Plan& p, int addr, int value, bool menuExit = true) {
  p.n = 0; p.preReads = 1;
  planAdd(p, STEP_READ, addr, 0, 0);
  planAdd(p, STEP_WRITE_CONST, addr, value, 300);
  if (menuExit) planAdd(p, STEP_WRITE_CONST, 161, 0, 500);
  planAdd(p, STEP_READ, addr, 0, 300);
}

inline void buildSavePlan(Plan& p) {
  buildReadPlan(p);
  planAdd(p, STEP_WRITE_CONST, 160, 1, 500);
  for (int i = 0; i < kTableRegs; i++) planAdd(p, STEP_WRITE_TABLE, kTableBase + i, 0, 500);
  planAdd(p, STEP_WRITE_CONST, 161, 0, 500);
  for (int i = 0; i < kTableRegs; i++) planAdd(p, STEP_READ, kTableBase + i, 0, 300);
}

// Speed 1..4 -> supply register (extract = supply + 1). Returns 0 for a bad speed.
inline int speedSupplyAddr(int speed) {
  switch (speed) { case 1: return 306; case 2: return 302; case 3: return 304; case 4: return 300; default: return 0; }
}

inline bool tableValueOk(long v, int lo, int hi) { return v >= lo && v <= hi; }

}  // namespace hrv
