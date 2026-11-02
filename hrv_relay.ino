// ============================================================================
//  HRV relay controller - ESP32 4-channel relay board + Titon HMB bus listener
//
//  * Relays 1-3 close the volt-free switch inputs S1..S3 on the HRV, with a
//    software interlock (never more than MAX_CLOSED_SWITCHES closed together),
//    timed holds, boot hold-off, and fail-open on network loss.
//  * RS-485 is receive-only unless relay 4 (Aura-T power cut) is energised and
//    the transmit interlock in config.h passes; only allow-listed writes are sent.
//  * MQTT + Home Assistant discovery.
//
//  Libraries: PubSubClient (Nick O'Leary), ESP32 Arduino core (WiFi, ArduinoOTA,
//  Preferences). Board: "ESP32 Dev Module".
//  Before wiring anything to the HRV, read the bring-up checklist.
// ============================================================================
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Preferences.h>

#include "secrets.h"
#include "config.h"
#include "relay_guard.h"
#include "hrv_bus.h"
#include "mini_json.h"
#include "table_seq.h"

#define NCH 4
static const int RELAY_PINS[NCH] = { RELAY_PIN_1, RELAY_PIN_2, RELAY_PIN_3, RELAY_PIN_4 };
static const int RELAY_INACTIVE = (RELAY_ACTIVE_LEVEL == HIGH) ? LOW : HIGH;

static hrv::RelayGuard guard(CH_LIMITS, NCH, MAX_CLOSED_SWITCHES,
                             BOOT_HOLD_MS, MIN_OFF_MS, MIN_GAP_MS);

static WiFiClient net;
static PubSubClient mqtt(net);
static Preferences prefs;

// ---- bus state ------------------------------------------------------------
struct Reg { int32_t v; bool ok; };
static Reg r070, r380, r381, r382, r383, r384, r388;
static Reg r050;   // PROBABLE room humidity, x10 %RH (reads 610-616 with the room near 61 %). Only filled if the Aura-T itself polls it.
// Census of the register addresses the Aura-T (not us) asks for on the bus, in first-seen order.
static int16_t seenAddr[24];
static int seenN = 0;
static void noteSeen(int addr) {
  for (int i = 0; i < seenN; i++) if (seenAddr[i] == addr) return;
  if (seenN < 24) seenAddr[seenN++] = (int16_t)addr;
}
static hrv::LineAssembler lineAsm;
static hrv::SettingsStore settings;
static uint32_t lastFrameMs = 0, frames = 0, badLines = 0;
static char lastWrite[24] = "";           // last write seen from another master
struct AuraWr { int16_t addr; int32_t val; uint32_t ms; };
static AuraWr auraWr[6];                  // newest first: writes the Aura-T made (124/125 left out: too chatty)
static int auraWrN = 0;
static void noteAuraWrite(int addr, int32_t v, uint32_t now) {
  if (addr == 124 || addr == 125) return;
  if (auraWrN > 0 && auraWr[0].addr == addr && auraWr[0].val == v) { auraWr[0].ms = now; return; }
  for (int i = 5; i > 0; i--) auraWr[i] = auraWr[i - 1];
  auraWr[0].addr = (int16_t)addr; auraWr[0].val = v; auraWr[0].ms = now;
  if (auraWrN < 6) auraWrN++;
}
static int funcCode[3] = { -1, -1, -1 };  // switch function seen/stored for S1..S3

// ---- transmit state (see TRANSMIT INTERLOCK in config.h) -------------------
struct Cmd { bool active; bool write; int addr; int32_t value; uint32_t since; };
static Cmd cmd = { false, false, 0, 0, 0 };
static bool awaiting = false, awaitWrite = false;
static int awaitAddr = 0;
static int32_t awaitValue = 0;
static uint32_t awaitDeadline = 0, lastRxMs = 0;
static bool autoOwnsRelay = false;   // relay 4 was energised by the firmware for this command
static bool relayDirty = false, eepromDirty = false;
static bool otaStarted = false;
static bool eepromEn = false;        // runtime EEPROM-write enable (self-clearing)
static uint32_t eepromEnUntil = 0;

// ---- speed-table edit/save sequence -----------------------------------------
static hrv::Plan plan;
static bool planActive = false, planWritesTable = false, planIsRestore = false, planIsCfg = false;
static bool planCfgWrite = false, cfgIsUndo = false;      // single-setting write sequence
static bool planScan = false;                              // read-only register scan
struct ScanRes { int16_t addr; int8_t st; int32_t val; };  // st: 1 = value, 0 = no reply, 2 = error reply (999)
static ScanRes scanRes[72];
static int scanN = 0;
static char scanSummary[420] = "not run";
static void scanRecord(int addr, int32_t v, int st) {
  if (scanN < 72) { scanRes[scanN].addr = (int16_t)addr; scanRes[scanN].st = (int8_t)st; scanRes[scanN].val = v; scanN++; }
}
static int cfgAddr = 0;
static int32_t cfgWant = 0;
static int cfgPrevA = -1, cfgPrevV = -1;                  // the value replaced by the last change (also kept in flash)
static bool stepInFlight = false, stepDone = false, stepBad = false;
static uint8_t readTries = 0;        // retries used for the read in flight (the unit often ignores the first request after the Aura-T is cut)
static int planIdx = 0, curStep = 0;
static uint32_t planNextAt = 0;
static int32_t staged[hrv::kTableRegs];     // edits waiting to be saved (0.1 % units)
static uint8_t stagedMask = 0;
static int32_t backupTbl[hrv::kTableRegs];  // table as read just before the last save
static bool backupOk = false;
static char planStatus[72] = "idle";

static uint32_t lastPublish = 0, lastMqttTry = 0, lastMqttOk = 0, lastDisc = 0;
static bool statePending = true;

static String T(const char* suffix) { return String(MQTT_BASE) + "/" + suffix; }

// ---- relays ---------------------------------------------------------------
static void applyRelays() {
  for (int i = 0; i < NCH; i++)
    digitalWrite(RELAY_PINS[i], guard.isOn(i) ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE);
}

static void updateBlocks() {
  for (int ch = 0; ch < 3; ch++) {
    int code = funcCode[ch];
    bool blocked;
    if (code < 0) blocked = REQUIRE_KNOWN_FUNCTION != 0;
    else blocked = !(code < 16 && (CH_ALLOWED_FUNCS[ch] & (1u << code)));
    guard.setBlocked(ch, blocked);
  }
}

static void setFunc(int ch, int code, bool persist) {
  if (ch < 0 || ch > 2 || funcCode[ch] == code) return;
  funcCode[ch] = code;
  if (persist) {
    char k[4] = { 'f', (char)('0' + ch), 0 };
    prefs.putInt(k, code);
  }
  updateBlocks();
  statePending = true;
}

// ---- bus ------------------------------------------------------------------
static void publishBusResult(const char* msg) {
  Serial.printf("bus: %s\n", msg);
  if (mqtt.connected()) mqtt.publish(T("bus/result").c_str(), msg, false);
}

static void setPlanStatus(const char* s) {
  strncpy(planStatus, s, sizeof planStatus - 1);
  planStatus[sizeof planStatus - 1] = 0;
  statePending = true;
  publishBusResult(s);
}

// Release relay 4 again if the firmware energised it for this command.
static void finishCmd() {
  if (!autoOwnsRelay) return;
  guard.request(3, false, 0, millis());
  applyRelays();
  autoOwnsRelay = false;
  relayDirty = true;
  statePending = true;
}

static void handleFrame(const hrv::Frame& f) {
  bool fromUs = false;
  if (awaiting && planActive && planScan && f.kind == hrv::KIND_REPLY && f.addr == 999) {   // "no such register"
    scanRecord(awaitAddr, 0, 2);
    awaiting = false; fromUs = true; readTries = 0;
    stepDone = true; stepBad = false;
  }
  if (awaiting && f.addr == awaitAddr &&
      (f.kind == hrv::KIND_REPLY || (awaitWrite && f.kind == hrv::KIND_REQUEST && f.write))) {
    char m[56];
    if (!awaitWrite)
      snprintf(m, sizeof m, "R %03d = %ld", awaitAddr, (long)f.value);
    else if (f.value == awaitValue)
      snprintf(m, sizeof m, "W %03d %ld echoed ok", awaitAddr, (long)awaitValue);
    else
      snprintf(m, sizeof m, "W %03d %ld ECHO MISMATCH, got %ld", awaitAddr, (long)awaitValue, (long)f.value);
    awaiting = false;
    fromUs = true;
    readTries = 0;
    if (planActive && planScan) scanRecord(awaitAddr, f.value, 1);
    bool bad = awaitWrite && f.value != awaitValue;
    if (!planActive || bad) publishBusResult(m);      // a table sequence reports once, at the end
    if (awaitWrite && f.value == awaitValue &&
        awaitAddr >= hrv::REG_FUNC_S1 && awaitAddr <= hrv::REG_FUNC_S3)
      setFunc(awaitAddr - hrv::REG_FUNC_S1, (int)awaitValue, true);   // keep the guard in step
    if (planActive) { stepDone = true; stepBad = bad; }
    else finishCmd();
  }
  frames++;
  lastFrameMs = millis();
  statePending = true;
  if (f.kind == hrv::KIND_REQUEST) {
    if (f.write) {
      snprintf(lastWrite, sizeof lastWrite, "%03d=%ld", f.addr, (long)f.value);
      if (!fromUs) noteAuraWrite(f.addr, f.value, millis());
    }
    return;   // only replies carry the unit's data
  }
  Reg* r = nullptr;
  switch (f.addr) {
    case hrv::REG_HOURS:    r = &r070; break;
    case hrv::REG_FAN_SUP:  r = &r380; break;
    case hrv::REG_FAN_EXT:  r = &r381; break;
    case hrv::REG_TEMP_SUP: r = &r382; break;
    case hrv::REG_TEMP_EXT: r = &r383; break;
    case hrv::REG_STATUS:   r = &r384; break;
    case hrv::REG_SYS:      r = &r388; break;
    case 50:                r = &r050; break;
    default: break;
  }
  if (r) { r->v = f.value; r->ok = true; }
  if (!fromUs && f.addr != 999) noteSeen(f.addr);
  if (f.addr >= hrv::REG_FUNC_S1 && f.addr <= hrv::REG_FUNC_S3)
    setFunc(f.addr - hrv::REG_FUNC_S1, (int)f.value, true);
  if (hrv::isSettingAddr(f.addr)) settings.set(f.addr, f.value);
}

static void pollBus() {
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    uint32_t tnow = millis();
    if (!awaiting) lastRxMs = tnow;      // the unit's replies to our own requests are not other traffic
    if (lineAsm.feed(b, tnow)) {
      hrv::Frame f;
      if (hrv::parseLine(lineAsm.line(), lineAsm.length(), f)) handleFrame(f);
      else badLines++;
    }
  }
}

// ---- transmit (every path goes through txAllowed) ---------------------------
static bool txAllowed(uint32_t now) {
  if (!guard.isOn(3)) return false;                                  // relay 4 must be energised
  if (guard.elapsedOnMs(3, now) < TX_SETTLE_MS) return false;        // Aura-T has had time to drop out
  if (guard.remainingMs(3, now) < TX_MIN_REMAIN_MS) return false;    // not about to re-power it
  if (now - lastRxMs < TX_QUIET_MS) return false;                    // bus is silent
  return true;
}

static bool sendFrame(const Cmd& c) {
  char b[24];
  size_t n;
  if (c.addr < 0) { n = 16; memset(b, 0x55, n); }   // transmit test pattern
  else n = hrv::buildRequest(b, c.addr, c.write, c.value);
  if (!n || !txAllowed(millis())) return false;    // re-checked immediately before driving the bus
  if (BUS_DE_PIN >= 0) digitalWrite(BUS_DE_PIN, HIGH);   // -1 = module switches direction itself
  delay(2);
  Serial.printf("TX start (fw 8s-test build): DE pin %d reads back %d\n", (int)BUS_DE_PIN, BUS_DE_PIN >= 0 ? digitalRead(BUS_DE_PIN) : -1);
  uint32_t txStart = millis();
  Serial2.write((const uint8_t*)b, n);
  Serial2.flush();
  if (c.addr < 0 && c.value > 0) {                 // long test: repeat bursts, DE stays high
    uint32_t t0 = millis();
    while (hrv::since(millis(), t0) < (uint32_t)c.value * 1000UL && guard.isOn(3)) {
      Serial2.write((const uint8_t*)b, n);
      Serial2.flush();
    }
  }
  delay(20);                                       // last stop bit + margin (about 8 ms per char at 1200)
  Serial.printf("TX end after %lu ms\n", (unsigned long)(millis() - txStart));
  if (BUS_DE_PIN >= 0) digitalWrite(BUS_DE_PIN, LOW);
  while (Serial2.available()) Serial2.read();      // discard our own echo if the receiver was live
  return true;
}

// End of a command or a table sequence: report why (if given) and release relay 4.
static void planAbort(const char* why);
static void endCmd(const char* why) {
  if (planActive) { planAbort(why); return; }
  if (why) publishBusResult(why);
  finishCmd();
}

static bool tableKnown() {
  int32_t v;
  for (int i = 0; i < hrv::kTableRegs; i++) if (!settings.get(hrv::kTableBase + i, v)) return false;
  return true;
}

static int32_t tableWant(int i) { return ((stagedMask >> i) & 1) ? staged[i] : backupTbl[i]; }

// Called when the pre-read block is complete: snapshot the table before anything is written.
static bool backupTable() {
  for (int i = 0; i < hrv::kTableRegs; i++) {
    int32_t v;
    if (!settings.get(hrv::kTableBase + i, v)) return false;
    backupTbl[i] = v;
  }
  backupOk = true;
  if (!planIsRestore) {                              // a restore must not overwrite the good backup
    for (int i = 0; i < hrv::kTableRegs; i++) {
      char k[5] = { 't', 'b', (char)('0' + i), 0, 0 };
      prefs.putInt(k, (int)backupTbl[i]);
    }
  }
  return true;
}

// Called when the pre-read of a single setting is complete: remember the old value for undo.
static bool backupCfg() {
  int32_t v;
  if (!settings.get(cfgAddr, v)) return false;
  if (!cfgIsUndo) {                                   // an undo must not overwrite the value it restores
    cfgPrevA = cfgAddr; cfgPrevV = (int)v;
    prefs.putInt("cpa", cfgPrevA);
    prefs.putInt("cpv", cfgPrevV);
  }
  return true;
}

static void planAbort(const char* why) {
  planActive = false; stepInFlight = false; stepDone = false;
  cmd.active = false;
  char m[72];
  snprintf(m, sizeof m, "table ABORTED: %s", why ? why : "?");
  setPlanStatus(m);
  finishCmd();
}

static void planFinish() {
  planActive = false; stepInFlight = false;
  if (planScan) {
    int n = 0, none = 0, err = 0;
    n += snprintf(scanSummary + n, sizeof scanSummary - n, "found: ");
    int nf = 0;
    for (int i = 0; i < scanN; i++) {
      if (scanRes[i].st == 1) {
        if (n < (int)sizeof scanSummary - 16) n += snprintf(scanSummary + n, sizeof scanSummary - n, "%s%03d=%ld", nf ? ", " : "", scanRes[i].addr, (long)scanRes[i].val);
        nf++;
      } else if (scanRes[i].st == 2) err++; else none++;
    }
    if (nf == 0) n += snprintf(scanSummary + n, sizeof scanSummary - n, "none");
    n += snprintf(scanSummary + n, sizeof scanSummary - n, " | no reply %d, error %d", none, err);
    if (none > 0 && n < (int)sizeof scanSummary - 24) {      // name the addresses that never answered
      n += snprintf(scanSummary + n, sizeof scanSummary - n, " [no reply:");
      for (int i = 0; i < scanN && n < (int)sizeof scanSummary - 8; i++)
        if (scanRes[i].st == 0) n += snprintf(scanSummary + n, sizeof scanSummary - n, " %03d", scanRes[i].addr);
      snprintf(scanSummary + n, sizeof scanSummary - n, "]");
    }
    setPlanStatus("register scan done");
    finishCmd();
    return;
  }
  if (planCfgWrite) {
    int32_t got = 0;
    bool ok = settings.get(cfgAddr, got) && got == cfgWant;
    char m[72];
    if (ok) snprintf(m, sizeof m, "setting %03d %s OK (read back %ld)", cfgAddr, cfgIsUndo ? "undo" : "write", (long)got);
    else snprintf(m, sizeof m, "setting %03d VERIFY FAILED (want %ld got %ld): undo", cfgAddr, (long)cfgWant, (long)got);
    setPlanStatus(m);
    finishCmd();
    return;
  }
  if (planIsCfg) { setPlanStatus("settings read OK"); finishCmd(); return; }
  if (!planWritesTable) { setPlanStatus("table read OK"); finishCmd(); return; }
  int bad = -1; int32_t got = 0;
  for (int i = 0; i < hrv::kTableRegs; i++)
    if (!settings.get(hrv::kTableBase + i, got) || got != tableWant(i)) { bad = i; break; }
  if (bad < 0) {
    stagedMask = 0;
    setPlanStatus("table save OK (read-back verified)");
  } else {
    char m[72];
    snprintf(m, sizeof m, "table VERIFY FAILED at %d (want %ld got %ld): restore", hrv::kTableBase + bad,
             (long)tableWant(bad), (long)got);
    setPlanStatus(m);
  }
  finishCmd();
}

static bool planStart(bool save, bool restore) {
  if (planActive || cmd.active || awaiting) { publishBusResult("rejected: busy"); return false; }
  if (save && !eepromEn) { publishBusResult("rejected: EEPROM write not enabled"); return false; }
  if (!guard.isOn(3)) {
    hrv::Result r = guard.request(3, true, TABLE_CUT_MS, millis());
    if (r != hrv::Result::Ok) { publishBusResult(hrv::resultName(r)); return false; }
    applyRelays();
    autoOwnsRelay = true; relayDirty = true; statePending = true;
  }
  if (save) hrv::buildSavePlan(plan); else hrv::buildReadPlan(plan);
  planActive = true; planWritesTable = save; planIsRestore = restore; planIsCfg = false; planCfgWrite = false; planScan = false;
  planIdx = 0; planNextAt = millis(); stepInFlight = false; stepDone = false; stepBad = false;
  setPlanStatus(save ? (restore ? "table restore: started" : "table save: started") : "table read: started");
  return true;
}

static bool planStartSettings() {
  if (planActive || cmd.active || awaiting) { publishBusResult("rejected: busy"); return false; }
  if (!guard.isOn(3)) {
    hrv::Result r = guard.request(3, true, TABLE_CUT_MS, millis());
    if (r != hrv::Result::Ok) { publishBusResult(hrv::resultName(r)); return false; }
    applyRelays();
    autoOwnsRelay = true; relayDirty = true; statePending = true;
  }
  hrv::buildSettingsReadPlan(plan);
  planActive = true; planWritesTable = false; planIsRestore = false; planIsCfg = true; planCfgWrite = false; planScan = false;
  planIdx = 0; planNextAt = millis(); stepInFlight = false; stepDone = false; stepBad = false;
  setPlanStatus("settings read: started");
  return true;
}

static bool planStartScan() {
  if (planActive || cmd.active || awaiting) { publishBusResult("rejected: busy"); return false; }
  if (!guard.isOn(3)) {
    hrv::Result r = guard.request(3, true, SCAN_CUT_MS, millis());
    if (r != hrv::Result::Ok) { publishBusResult(hrv::resultName(r)); return false; }
    applyRelays();
    autoOwnsRelay = true; relayDirty = true; statePending = true;
  }
  hrv::buildScanPlan(plan);
  scanN = 0;
  snprintf(scanSummary, sizeof scanSummary, "running...");
  planActive = true; planWritesTable = false; planIsRestore = false; planIsCfg = false; planCfgWrite = false; planScan = true;
  planIdx = 0; planNextAt = millis(); stepInFlight = false; stepDone = false; stepBad = false;
  setPlanStatus("register scan: started");
  return true;
}

static bool planStartCfgWrite(int addr, int32_t raw, bool undo) {
  if (planActive || cmd.active || awaiting) { publishBusResult("rejected: busy"); return false; }
  if (!eepromEn) { publishBusResult("rejected: EEPROM write not enabled"); return false; }
  const hrv::CfgReg* r = hrv::findCfg(addr);
  if (!r || !r->writable || !hrv::isWritableAddr(addr, ALLOW_WRITE_SETTINGS != 0 && eepromEn) ||
      raw < r->lo || raw > r->hi) { publishBusResult("rejected: setting or value not allowed"); return false; }
  if (!guard.isOn(3)) {
    hrv::Result res = guard.request(3, true, TABLE_CUT_MS, millis());
    if (res != hrv::Result::Ok) { publishBusResult(hrv::resultName(res)); return false; }
    applyRelays();
    autoOwnsRelay = true; relayDirty = true; statePending = true;
  }
  hrv::buildCfgWritePlan(plan, addr, (int)raw, CFG_SEND_MENU_EXIT != 0);
  planActive = true; planWritesTable = true; planIsRestore = false; planIsCfg = false; planCfgWrite = true; planScan = false;
  cfgIsUndo = undo; cfgAddr = addr; cfgWant = raw;
  planIdx = 0; planNextAt = millis(); stepInFlight = false; stepDone = false; stepBad = false;
  setPlanStatus(undo ? "setting undo: started" : "setting write: started");
  return true;
}

static void serviceCmd(uint32_t now) {
  if (awaiting) {
    if ((int32_t)(now - awaitDeadline) > 0) {
      awaiting = false;
      if (!awaitWrite && readTries < READ_RETRIES) {     // reads are harmless to repeat; writes are not
        readTries++;
        Serial.printf("bus: R %03d no reply, retry %u\n", awaitAddr, (unsigned)readTries);
        cmd.write = false; cmd.addr = awaitAddr; cmd.value = 0;
        cmd.active = true; cmd.since = now;
        return;
      }
      readTries = 0;
      if (planActive && planScan) {                      // an absent register is a result, not a failure
        scanRecord(awaitAddr, 0, 0);
        stepDone = true; stepBad = false;
        return;
      }
      char m[40];
      snprintf(m, sizeof m, "%s %03d: no reply", awaitWrite ? "W" : "R", awaitAddr);
      endCmd(m);
    }
    return;
  }
  if (planActive) {
    if (!guard.isOn(3)) { planAbort("relay 4 released"); return; }
    if (stepDone) {
      stepDone = false; stepInFlight = false;
      if (stepBad) { planAbort("echo mismatch"); return; }
      if (planWritesTable && curStep == plan.preReads - 1 && !(planCfgWrite ? backupCfg() : backupTable())) {
        planAbort("could not read the old value first (nothing written)"); return;
      }
      planIdx = curStep + 1;
      if (planIdx >= plan.n) { planFinish(); return; }
      planNextAt = now + plan.s[planIdx].delayMs;
    }
    if (!stepInFlight && !cmd.active && planIdx < plan.n && (int32_t)(now - planNextAt) >= 0) {
      const hrv::Step& st = plan.s[planIdx];
      cmd.write = st.kind != hrv::STEP_READ;
      cmd.addr = st.addr;
      cmd.value = (st.kind == hrv::STEP_WRITE_CONST) ? st.value
                : (st.kind == hrv::STEP_WRITE_TABLE) ? tableWant(st.addr - hrv::kTableBase) : 0;
      cmd.active = true; cmd.since = now;
      stepInFlight = true; curStep = planIdx;
      readTries = (planScan && planIdx > 0) ? READ_RETRIES : 0;   // only the first read after the cut is retried in a scan
    }
  }
  if (!cmd.active) return;
  if (!guard.isOn(3)) { cmd.active = false; endCmd("rejected: relay 4 not energised"); return; }
  if (now - cmd.since > TX_WAIT_MAX_MS) { cmd.active = false; endCmd("rejected: transmit conditions not met in time"); return; }
  if (sendFrame(cmd)) {
    if (cmd.addr < 0) {                      // test pattern: nothing to wait for
      cmd.active = false;
      publishBusResult("test pattern sent (16 x 0x55 at 1200 8N1)");
      finishCmd();
      return;
    }
    awaiting = true; awaitDeadline = millis() + TX_REPLY_MS;
    awaitWrite = cmd.write; awaitAddr = cmd.addr; awaitValue = cmd.value;
    cmd.active = false;
  }
}

static void queueCmd(const char* msg) {
  const char* p = msg;
  while (*p == ' ') p++;
  bool autoRelay = false;                          // "A R 385" / "A W 385 2": firmware drives relay 4
  if ((*p == 'A' || *p == 'a') && p[1] == ' ') { autoRelay = true; p += 2; }
  if ((*p == 'T' || *p == 't') && (p[1] == 0 || p[1] == ' ')) {   // transmit test: 16 x 'U' (0x55), no unit needed
    if (cmd.active || awaiting || planActive) { publishBusResult("rejected: busy"); return; }
    if (!guard.isOn(3)) {
      if (!autoRelay) { publishBusResult("rejected: relay 4 not energised"); return; }
      hrv::Result r = guard.request(3, true, AUTO_CUT_MS, millis());
      if (r != hrv::Result::Ok) { publishBusResult(hrv::resultName(r)); return; }
      applyRelays();
      autoOwnsRelay = true; relayDirty = true; statePending = true;
    }
    long secs = atol(p + 1);                       // "A T 8": keep sending for 8 s (max 10) so a meter can see it
    if (secs < 0) secs = 0;
    if (secs > 10) secs = 10;
    cmd.active = true; cmd.write = false; cmd.addr = -1; cmd.value = (int32_t)secs; cmd.since = millis();
    return;
  }
  char c = 0; int a = 0; long v = 0;
  int n = sscanf(p, " %c %d %ld", &c, &a, &v);
  if (c >= 'a' && c <= 'z') c -= 32;
  if (n < 2 || (c != 'R' && c != 'W') || (c == 'W' && n < 3) || a < 0 || a > 999 || v < 0 || v > 99999) {
    publishBusResult("rejected: use '[A ]R <addr>' or '[A ]W <addr> <value>'"); return;
  }
  if (c == 'R') v = 0;
  if (c == 'W' && !hrv::isWritableAddr(a, ALLOW_WRITE_SETTINGS != 0 && eepromEn)) {
    publishBusResult((ALLOW_WRITE_SETTINGS != 0 && hrv::isWritableAddr(a, true))
                         ? "rejected: EEPROM write not enabled"
                         : "rejected: register not writable (see config.h)");
    return;
  }
  if (cmd.active || awaiting || planActive) { publishBusResult("rejected: busy"); return; }
  if (!guard.isOn(3)) {
    if (!autoRelay) { publishBusResult("rejected: relay 4 not energised"); return; }
    hrv::Result r = guard.request(3, true, AUTO_CUT_MS, millis());
    if (r != hrv::Result::Ok) { publishBusResult(hrv::resultName(r)); return; }
    applyRelays();
    autoOwnsRelay = true; relayDirty = true; statePending = true;
  }
  cmd.active = true; cmd.write = (c == 'W'); cmd.addr = a; cmd.value = (int32_t)v; cmd.since = millis();
}

// ---- MQTT -----------------------------------------------------------------
static void publishRelayState(int ch) {
  mqtt.publish(T(("ch/" + String(ch + 1) + "/state").c_str()).c_str(),
               guard.isOn(ch) ? "ON" : "OFF", true);
}

static void publishResult(int ch, const char* msg) {
  mqtt.publish(T(("ch/" + String(ch + 1) + "/result").c_str()).c_str(), msg, false);
  Serial.printf("ch%d: %s\n", ch + 1, msg);
}

static void publishState() {
  char buf[2900];
  hrv::JsonOut j(buf, sizeof buf);
  uint32_t now = millis();
  j.begin();
  for (int i = 0; i < NCH; i++) {
    char k[12];
    snprintf(k, sizeof k, "ch%d", i + 1);
    j.str(k, guard.isOn(i) ? "ON" : "OFF");
    snprintf(k, sizeof k, "ch%d_left_s", i + 1);
    j.integer(k, (long)(guard.remainingMs(i, now) / 1000UL), true);
  }
  for (int i = 0; i < 3; i++) {
    char k[20];                                    // "s1_func_name" is 12 chars: a 12-byte buffer cut it to "s1_func_nam"
    snprintf(k, sizeof k, "s%d_func", i + 1);
    j.integer(k, funcCode[i], funcCode[i] >= 0);
    snprintf(k, sizeof k, "s%d_func_name", i + 1);
    j.str(k, funcCode[i] >= 0 ? hrv::funcName(funcCode[i]) : nullptr);
    snprintf(k, sizeof k, "s%d_blocked", i + 1);
    j.boolean(k, guard.isBlocked(i), true);
  }
  String ipStr = WiFi.localIP().toString();
  j.str("ip", ipStr.c_str());
  j.str("eeprom", eepromEn ? "ON" : "OFF");
  j.integer("eeprom_left_s", eepromEn ? (long)((eepromEnUntil - now) / 1000UL) : 0, true);
  j.num("fan_sup", r380.v / 10.0, 1, r380.ok);
  j.num("fan_ext", r381.v / 10.0, 1, r381.ok);
  j.num("temp_sup", r382.v / 10.0, 1, r382.ok);
  j.num("temp_ext", r383.v / 10.0, 1, r383.ok);
  j.integer("hours", r070.v, r070.ok);
  j.integer("status_raw", r384.v, r384.ok);
  hrv::StatusBits sb = hrv::decodeStatus(r384.v);
  j.integer("speed", sb.speed, r384.ok && sb.speed);
  j.boolean("switch_closed", sb.switchClosed, r384.ok);
  j.boolean("overrun", sb.overrun, r384.ok);
  j.boolean("bypass", sb.bypass, r384.ok);
  j.boolean("summerboost", sb.summerboost, r384.ok);
  j.boolean("fault", sb.fault, r384.ok);
  j.boolean("boost_inhibit", sb.boostInhibit, r384.ok);
  j.boolean("rh_control", sb.rhControl, r384.ok);
  j.num("rh_050", r050.v / 10.0, 1, r050.ok);
  {
    char cs[130]; int n = 0; cs[0] = 0;
    for (int i = 0; i < seenN && n < (int)sizeof cs - 6; i++)
      n += snprintf(cs + n, sizeof cs - n, "%s%03d", i ? " " : "", (int)seenAddr[i]);
    j.str("census", seenN ? cs : "none yet");
  }
  j.integer("bus_age_s", (long)((now - lastFrameMs) / 1000UL), frames > 0);
  j.integer("frames", (long)frames, true);
  j.integer("bad_lines", (long)badLines, true);
  j.str("last_write", lastWrite[0] ? lastWrite : nullptr);
  for (int i = 0; i < hrv::kTableRegs; i++) {
    char k[6];
    int32_t v = 0;
    bool have = ((stagedMask >> i) & 1) ? (v = staged[i], true) : settings.get(hrv::kTableBase + i, v);
    snprintf(k, sizeof k, "t%d", hrv::kTableBase + i);
    j.num(k, v / 10.0, 1, have);
  }
  j.boolean("table_pending", stagedMask != 0, true);
  j.boolean("table_known", tableKnown(), true);
  j.str("table_status", planStatus);
  for (int i = 0; i < hrv::kNumCfgRegs; i++) {
    int32_t v = 0;
    bool have = settings.get(hrv::kCfgRegs[i].addr, v);
    j.integer(hrv::kCfgRegs[i].key, v, have);
  }
  {
    char aw[170]; int n = 0;
    if (auraWrN == 0) snprintf(aw, sizeof aw, "none yet");
    else {
      aw[0] = 0;
      for (int i = 0; i < auraWrN; i++)
        n += snprintf(aw + n, sizeof aw - n, "%s%03d=%ld (%lus ago)", i ? "; " : "",
                      (int)auraWr[i].addr, (long)auraWr[i].val, (unsigned long)((now - auraWr[i].ms) / 1000UL));
    }
    j.str("aura_writes", aw);
  }
  {
    char cp[40];
    if (cfgPrevA >= 0) snprintf(cp, sizeof cp, "%03d was %d", cfgPrevA, cfgPrevV); else snprintf(cp, sizeof cp, "none");
    j.str("cfg_prev", cp);
  }
  {
    // Home Assistant shows a sensor state longer than 255 characters as "unknown", so a long
    // scan result goes out as two sensors, split at the last space before character 230.
    char s1[256], s2[200];
    size_t L = strlen(scanSummary);
    if (L <= 230) { memcpy(s1, scanSummary, L + 1); s2[0] = 0; }
    else {
      size_t cut = 230;
      while (cut > 100 && scanSummary[cut] != ' ') cut--;
      memcpy(s1, scanSummary, cut); s1[cut] = 0;
      snprintf(s2, sizeof s2, "%s", scanSummary + cut + 1);
    }
    j.str("scan", s1);
    j.str("scan2", s2[0] ? s2 : "-");
  }
  j.end();
  if (j.ok()) mqtt.publish(T("state").c_str(), j.c_str(), false);
}

static void discoverySwitch(int ch) {
  char topic[96], pl[640];
  snprintf(topic, sizeof topic, "homeassistant/switch/%s_ch%d/config", MQTT_BASE, ch + 1);
  snprintf(pl, sizeof pl,
    "{\"name\":\"%s\",\"uniq_id\":\"%s_ch%d\",\"cmd_t\":\"%s/ch/%d/set\","
    "\"stat_t\":\"%s/ch/%d/state\",\"avty_t\":\"%s/status\","
    "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev\":{\"ids\":[\"%s\"],"
    "\"name\":\"HRV relay\",\"mf\":\"DIY\",\"mdl\":\"ESP32 4ch relay\"}}",
    CH_NAMES[ch], MQTT_BASE, ch + 1, MQTT_BASE, ch + 1, MQTT_BASE, ch + 1, MQTT_BASE, MQTT_BASE);
  mqtt.publish(topic, pl, true);
}

static void discoveryEntity(const char* comp, const char* key, const char* name,
                            const char* unit, const char* devClass, bool binary) {
  char topic[112], pl[700];
  snprintf(topic, sizeof topic, "homeassistant/%s/%s_%s/config", comp, MQTT_BASE, key);
  int n = snprintf(pl, sizeof pl,
    "{\"name\":\"HRV %s\",\"uniq_id\":\"%s_%s\",\"stat_t\":\"%s/state\",\"avty_t\":\"%s/status\",",
    name, MQTT_BASE, key, MQTT_BASE, MQTT_BASE);
  if (binary)
    n += snprintf(pl + n, sizeof pl - n, "\"val_tpl\":\"{{ 'ON' if value_json.%s else 'OFF' }}\",", key);
  else
    n += snprintf(pl + n, sizeof pl - n, "\"val_tpl\":\"{{ value_json.%s }}\",", key);
  if (unit && unit[0]) n += snprintf(pl + n, sizeof pl - n, "\"unit_of_meas\":\"%s\",", unit);
  if (devClass && devClass[0]) n += snprintf(pl + n, sizeof pl - n, "\"dev_cla\":\"%s\",", devClass);
  snprintf(pl + n, sizeof pl - n, "\"dev\":{\"ids\":[\"%s\"]}}", MQTT_BASE);
  mqtt.publish(topic, pl, true);
}

static void discoverySelect(const char* key, const char* name, const char* cmdTopic,
                            const char* valTpl, const char* optionsJson) {
  char topic[112], pl[900];
  snprintf(topic, sizeof topic, "homeassistant/select/%s_%s/config", MQTT_BASE, key);
  snprintf(pl, sizeof pl,
    "{\"name\":\"%s\",\"uniq_id\":\"%s_%s\",\"cmd_t\":\"%s\",\"stat_t\":\"%s/state\","
    "\"val_tpl\":\"%s\",\"avty_t\":\"%s/status\",\"options\":%s,\"dev\":{\"ids\":[\"%s\"]}}",
    name, MQTT_BASE, key, cmdTopic, MQTT_BASE, valTpl, MQTT_BASE, optionsJson, MQTT_BASE);
  mqtt.publish(topic, pl, true);
}

static void publishDiscoveryControls() {
  char opts[420], cmdT[80], tpl[200], name[40], key[16];
  // Speed dropdown -> W 385 n
  snprintf(cmdT, sizeof cmdT, "%s/speed/set", MQTT_BASE);
  discoverySelect("set_speed", "HRV set speed", cmdT,
    "{{ 'Speed ' ~ value_json.speed if value_json.speed else 'None' }}",
    "[\"Speed 1\",\"Speed 2\",\"Speed 3\",\"Speed 4\"]");
  // Switch function dropdowns -> W 330/331/332 code (needs the EEPROM enable)
  int n = snprintf(opts, sizeof opts, "[");
  for (int i = 0; i < N_FUNC_OPTIONS; i++)
    n += snprintf(opts + n, sizeof opts - n, "%s\"%s\"", i ? "," : "", FUNC_OPTIONS[i].name);
  snprintf(opts + n, sizeof opts - n, "]");
  for (int ch = 0; ch < 3; ch++) {
    snprintf(cmdT, sizeof cmdT, "%s/func_set/%d/set", MQTT_BASE, ch + 1);
    snprintf(name, sizeof name, "HRV S%d function", ch + 1);
    snprintf(key, sizeof key, "s%d_function", ch + 1);
    snprintf(tpl, sizeof tpl, "{{ value_json.s%d_func_name if value_json.s%d_func_name else 'None' }}", ch + 1, ch + 1);
    discoverySelect(key, name, cmdT, tpl, opts);
  }
  // EEPROM write enable switch
  char topic[96], pl[500];
  snprintf(topic, sizeof topic, "homeassistant/switch/%s_eeprom/config", MQTT_BASE);
  snprintf(pl, sizeof pl,
    "{\"name\":\"HRV EEPROM write enable\",\"uniq_id\":\"%s_eeprom\",\"cmd_t\":\"%s/eeprom/set\","
    "\"stat_t\":\"%s/eeprom/state\",\"avty_t\":\"%s/status\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
    "\"icon\":\"mdi:lock-open-alert\",\"dev\":{\"ids\":[\"%s\"]}}",
    MQTT_BASE, MQTT_BASE, MQTT_BASE, MQTT_BASE, MQTT_BASE);
  mqtt.publish(topic, pl, true);
  // Last bus command result
  snprintf(topic, sizeof topic, "homeassistant/sensor/%s_bus_result/config", MQTT_BASE);
  snprintf(pl, sizeof pl,
    "{\"name\":\"HRV last bus result\",\"uniq_id\":\"%s_bus_result\",\"stat_t\":\"%s/bus/result\","
    "\"avty_t\":\"%s/status\",\"dev\":{\"ids\":[\"%s\"]}}",
    MQTT_BASE, MQTT_BASE, MQTT_BASE, MQTT_BASE);
  mqtt.publish(topic, pl, true);
}

static void publishDiscoveryTable() {
  static const char* const SPEED_NAME[5] = { "", "Speed 1 (Setback)", "Speed 2 (Continuous)", "Speed 3 (Boost)", "Speed 4 (SUMMERboost)" };
  char topic[112], pl[560];
  for (int sp = 1; sp <= 4; sp++) {
    for (int ext = 0; ext < 2; ext++) {
      int addr = hrv::speedSupplyAddr(sp) + ext;
      snprintf(topic, sizeof topic, "homeassistant/number/%s_t%d/config", MQTT_BASE, addr);
      snprintf(pl, sizeof pl,
        "{\"name\":\"HRV %s %s %%\",\"uniq_id\":\"%s_t%d\",\"cmd_t\":\"%s/table/set/%d\","
        "\"stat_t\":\"%s/state\",\"val_tpl\":\"{{ value_json.t%d }}\",\"avty_t\":\"%s/status\","
        "\"min\":%d,\"max\":%d,\"step\":0.1,\"mode\":\"box\",\"unit_of_meas\":\"%%\","
        "\"dev\":{\"ids\":[\"%s\"]}}",
        SPEED_NAME[sp], ext ? "extract" : "supply", MQTT_BASE, addr, MQTT_BASE, addr,
        MQTT_BASE, addr, MQTT_BASE, TABLE_MIN / 10, TABLE_MAX / 10, MQTT_BASE);
      mqtt.publish(topic, pl, true);
    }
  }
  static const char* const BTN_KEY[4]  = { "table_apply", "table_discard", "table_restore", "table_read" };
  static const char* const BTN_NAME[4] = { "HRV save table to unit", "HRV discard table edits",
                                           "HRV restore previous table", "HRV read table from unit" };
  static const char* const BTN_TOPIC[4] = { "apply", "discard", "restore", "read" };
  for (int i = 0; i < 4; i++) {
    snprintf(topic, sizeof topic, "homeassistant/button/%s_%s/config", MQTT_BASE, BTN_KEY[i]);
    snprintf(pl, sizeof pl,
      "{\"name\":\"%s\",\"uniq_id\":\"%s_%s\",\"cmd_t\":\"%s/table/%s\",\"payload_press\":\"PRESS\","
      "\"avty_t\":\"%s/status\",\"dev\":{\"ids\":[\"%s\"]}}",
      BTN_NAME[i], MQTT_BASE, BTN_KEY[i], MQTT_BASE, BTN_TOPIC[i], MQTT_BASE, MQTT_BASE);
    mqtt.publish(topic, pl, true);
  }
  snprintf(topic, sizeof topic, "homeassistant/sensor/%s_table_status/config", MQTT_BASE);
  snprintf(pl, sizeof pl,
    "{\"name\":\"HRV table status\",\"uniq_id\":\"%s_table_status\",\"stat_t\":\"%s/state\","
    "\"val_tpl\":\"{{ value_json.table_status }}\",\"avty_t\":\"%s/status\",\"dev\":{\"ids\":[\"%s\"]}}",
    MQTT_BASE, MQTT_BASE, MQTT_BASE, MQTT_BASE);
  mqtt.publish(topic, pl, true);
  snprintf(topic, sizeof topic, "homeassistant/binary_sensor/%s_table_pending/config", MQTT_BASE);
  snprintf(pl, sizeof pl,
    "{\"name\":\"HRV table edits pending\",\"uniq_id\":\"%s_table_pending\",\"stat_t\":\"%s/state\","
    "\"val_tpl\":\"{{ 'ON' if value_json.table_pending else 'OFF' }}\",\"avty_t\":\"%s/status\","
    "\"dev\":{\"ids\":[\"%s\"]}}",
    MQTT_BASE, MQTT_BASE, MQTT_BASE, MQTT_BASE);
  mqtt.publish(topic, pl, true);
}

static void publishDiscoverySettings() {
  char topic[112], pl[760], tpl[120], mn[16], mx[16], st[8];
  for (int i = 0; i < hrv::kNumCfgRegs; i++) {
    const hrv::CfgReg& r = hrv::kCfgRegs[i];
    discoveryEntity("sensor", r.key, r.label, "", "", false);
    if (!r.writable) continue;
    if (r.invBool) {                                   // 290: raw 0 = SUMMERboost enabled = switch ON
      snprintf(topic, sizeof topic, "homeassistant/switch/%s_cfg%d/config", MQTT_BASE, r.addr);
      snprintf(pl, sizeof pl,
        "{\"name\":\"HRV %s\",\"uniq_id\":\"%s_cfg%d\",\"cmd_t\":\"%s/cfg/set/%d\",\"stat_t\":\"%s/state\","
        "\"val_tpl\":\"{{ none if value_json.%s is none else ('ON' if value_json.%s == 0 else 'OFF') }}\","
        "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"avty_t\":\"%s/status\",\"dev\":{\"ids\":[\"%s\"]}}",
        r.name, MQTT_BASE, r.addr, MQTT_BASE, r.addr, MQTT_BASE, r.key, r.key, MQTT_BASE, MQTT_BASE);
    } else {
      if (r.scale == 10) {
        snprintf(mn, sizeof mn, "%.1f", r.lo / 10.0); snprintf(mx, sizeof mx, "%.1f", r.hi / 10.0);
        snprintf(st, sizeof st, "0.1");
        snprintf(tpl, sizeof tpl, "{{ (value_json.%s / 10) if value_json.%s is not none else none }}", r.key, r.key);
      } else {
        snprintf(mn, sizeof mn, "%d", r.lo); snprintf(mx, sizeof mx, "%d", r.hi);
        snprintf(st, sizeof st, "1");
        snprintf(tpl, sizeof tpl, "{{ value_json.%s }}", r.key);
      }
      snprintf(topic, sizeof topic, "homeassistant/number/%s_cfg%d/config", MQTT_BASE, r.addr);
      snprintf(pl, sizeof pl,
        "{\"name\":\"HRV %s\",\"uniq_id\":\"%s_cfg%d\",\"cmd_t\":\"%s/cfg/set/%d\",\"stat_t\":\"%s/state\","
        "\"val_tpl\":\"%s\",\"avty_t\":\"%s/status\",\"min\":%s,\"max\":%s,\"step\":%s,\"mode\":\"box\","
        "\"unit_of_meas\":\"%s\",\"dev\":{\"ids\":[\"%s\"]}}",
        r.name, MQTT_BASE, r.addr, MQTT_BASE, r.addr, MQTT_BASE, tpl, MQTT_BASE, mn, mx, st, r.unit, MQTT_BASE);
    }
    mqtt.publish(topic, pl, true);
  }
  discoveryEntity("sensor", "aura_writes", "Aura-T writes seen", "", "", false);
  discoveryEntity("sensor", "cfg_prev", "Setting before last change", "", "", false);
  discoveryEntity("sensor", "scan", "register scan result", "", "", false);
  discoveryEntity("sensor", "scan2", "register scan result (continued)", "", "", false);
  static const char* const BKEY[3]  = { "settings_read", "cfg_undo", "scan_run" };
  static const char* const BNAME[3] = { "HRV read unit settings", "HRV undo last setting change", "HRV scan registers (read only)" };
  static const char* const BTOP[3]  = { "settings/read", "cfg/undo", "scan/run" };
  for (int i = 0; i < 3; i++) {
    snprintf(topic, sizeof topic, "homeassistant/button/%s_%s/config", MQTT_BASE, BKEY[i]);
    snprintf(pl, sizeof pl,
      "{\"name\":\"%s\",\"uniq_id\":\"%s_%s\",\"cmd_t\":\"%s/%s\","
      "\"payload_press\":\"PRESS\",\"avty_t\":\"%s/status\",\"dev\":{\"ids\":[\"%s\"]}}",
      BNAME[i], MQTT_BASE, BKEY[i], MQTT_BASE, BTOP[i], MQTT_BASE, MQTT_BASE);
    mqtt.publish(topic, pl, true);
  }
}

static void publishDiscovery() {
  publishDiscoveryControls();
  publishDiscoveryTable();
  publishDiscoverySettings();
  for (int i = 0; i < NCH; i++) discoverySwitch(i);
  discoveryEntity("sensor", "fan_sup", "supply fan", "%", "", false);
  discoveryEntity("sensor", "fan_ext", "extract fan", "%", "", false);
  discoveryEntity("sensor", "temp_sup", "outside air temp (382)", "\xC2\xB0""C", "temperature", false);
  discoveryEntity("sensor", "temp_ext", "extract temp", "\xC2\xB0""C", "temperature", false);
  discoveryEntity("sensor", "hours", "running hours", "h", "", false);
  discoveryEntity("sensor", "speed", "speed", "", "", false);
  discoveryEntity("sensor", "status_raw", "status code", "", "", false);
  discoveryEntity("sensor", "bus_age_s", "bus age", "s", "", false);
  discoveryEntity("sensor", "last_write", "last bus write", "", "", false);
  discoveryEntity("sensor", "ip", "IP address", "", "", false);
  discoveryEntity("binary_sensor", "overrun", "overrun", "", "", true);
  discoveryEntity("binary_sensor", "bypass", "bypass", "", "", true);
  discoveryEntity("binary_sensor", "summerboost", "SUMMERboost", "", "", true);
  discoveryEntity("binary_sensor", "fault", "fault", "", "problem", true);
  discoveryEntity("binary_sensor", "switch_closed", "switch input closed", "", "", true);
  discoveryEntity("sensor", "rh_050", "humidity 050 (probable RH, only if the Aura-T polls it)", "%", "", false);
  discoveryEntity("sensor", "census", "registers the Aura-T polls", "", "", false);
  discoveryEntity("binary_sensor", "boost_inhibit", "boost inhibit (padlock)", "", "", true);
  discoveryEntity("binary_sensor", "rh_control", "humidity control (speed set by RH sensor)", "", "", true);
}

static void onMessage(char* topic, byte* payload, unsigned int len) {
  char msg[24];
  if (len >= sizeof msg) len = sizeof msg - 1;
  memcpy(msg, payload, len);
  msg[len] = 0;

  String t(topic);
  if (t == T("bus/cmd")) { queueCmd(msg); return; }
  if (t == T("speed/set")) {                     // "Speed 1".."Speed 4" or "1".."4"
    char d = len ? msg[len - 1] : 0;
    if (d < '1' || d > '4') { publishBusResult("rejected: speed must be 1-4"); return; }
    char cm[24];
    snprintf(cm, sizeof cm, "A W 385 %c", d);
    queueCmd(cm);
    return;
  }
  if (t.startsWith(T("table/set/"))) {            // number entity: percent value for one table register
    int addr = t.substring(T("table/set/").length()).toInt();
    int idx = addr - hrv::kTableBase;
    long iv = (long)(atof(msg) * 10.0 + 0.5);
    if (planActive) { publishBusResult("rejected: table sequence running"); return; }
    if (idx < 0 || idx >= hrv::kTableRegs || !hrv::tableValueOk(iv, TABLE_MIN, TABLE_MAX)) {
      publishBusResult("rejected: table value out of range"); statePending = true; return;
    }
    staged[idx] = (int32_t)iv; stagedMask |= (uint8_t)(1u << idx); statePending = true;
    return;
  }
  if (t == T("table/discard")) { stagedMask = 0; statePending = true; return; }
  if (t == T("table/read"))    { planStart(false, false); return; }
  if (t == T("settings/read")) { planStartSettings(); return; }
  if (t == T("scan/run"))      { planStartScan(); return; }
  if (t.startsWith(T("cfg/set/"))) {             // one unit setting, in displayed units (290: ON = enabled)
    int addr = t.substring(T("cfg/set/").length()).toInt();
    const hrv::CfgReg* r = hrv::findCfg(addr);
    if (!r || !r->writable) { publishBusResult("rejected: setting not writable"); return; }
    long raw = r->invBool ? (strcasecmp(msg, "ON") == 0 ? 0 : 1) : (long)(atof(msg) * r->scale + 0.5);
    if (raw < r->lo || raw > r->hi) { publishBusResult("rejected: value out of range"); statePending = true; return; }
    planStartCfgWrite(addr, (int32_t)raw, false);
    return;
  }
  if (t == T("cfg/undo")) {
    const hrv::CfgReg* r = hrv::findCfg(cfgPrevA);
    if (!r || !r->writable || cfgPrevV < r->lo || cfgPrevV > r->hi) { publishBusResult("rejected: no previous setting stored"); return; }
    planStartCfgWrite(cfgPrevA, cfgPrevV, true);
    return;
  }
  if (t == T("table/apply")) {
    if (stagedMask == 0) { publishBusResult("rejected: no table edits staged"); return; }
    planStart(true, false);
    return;
  }
  if (t == T("table/restore")) {
    int32_t bk[hrv::kTableRegs]; bool ok = true;
    for (int i = 0; i < hrv::kTableRegs && ok; i++) {
      char k[5] = { 't', 'b', (char)('0' + i), 0, 0 };
      bk[i] = prefs.getInt(k, -1);
      if (bk[i] < TABLE_MIN || bk[i] > TABLE_MAX) ok = false;
    }
    if (!ok) { publishBusResult("rejected: no valid backup stored"); return; }
    if (planActive || cmd.active || awaiting) { publishBusResult("rejected: busy"); return; }
    for (int i = 0; i < hrv::kTableRegs; i++) staged[i] = bk[i];
    stagedMask = 0xFF;
    if (!planStart(true, true)) stagedMask = 0;
    return;
  }
  if (t == T("eeprom/set")) {
    bool on = (strcasecmp(msg, "ON") == 0);
    eepromEn = on;
    eepromEnUntil = millis() + EEPROM_ENABLE_MS;
    eepromDirty = true; statePending = true;
    return;
  }
  if (t.startsWith(T("func_set/")) && t.endsWith("/set")) {
    int ch = t.substring(T("func_set/").length()).toInt() - 1;
    int code = -1;
    for (int i = 0; i < N_FUNC_OPTIONS; i++) if (strcmp(msg, FUNC_OPTIONS[i].name) == 0) code = FUNC_OPTIONS[i].code;
    if (ch < 0 || ch > 2 || code < 0) { publishBusResult("rejected: unknown switch or function"); return; }
    char cm[32];
    snprintf(cm, sizeof cm, "A W %d %d", 330 + ch, code);
    queueCmd(cm);
    return;
  }
  String chPrefix = T("ch/"), fnPrefix = T("func/");
  if (t.startsWith(chPrefix) && t.endsWith("/set")) {
    int ch = t.substring(chPrefix.length()).toInt() - 1;
    if (ch < 0 || ch >= NCH) return;
    for (char* p = msg; *p; ++p) if (*p >= 'a' && *p <= 'z') *p -= 32;
    bool on;
    uint32_t dur = 0;                       // 0 = channel default
    if (strncmp(msg, "ON", 2) == 0) {
      on = true;
      const char* c = strchr(msg, ',');     // "ON,<minutes>"
      if (c) { long m = atol(c + 1); if (m > 0) dur = (uint32_t)m * 60000UL; }
    } else if (strcmp(msg, "OFF") == 0) {
      on = false;
    } else {
      publishResult(ch, "rejected: payload must be ON, ON,<minutes> or OFF");
      return;
    }
    hrv::Result res = guard.request(ch, on, dur, millis());
    applyRelays();
    publishResult(ch, hrv::resultName(res));
    publishRelayState(ch);
    statePending = true;
  } else if (t.startsWith(fnPrefix) && t.endsWith("/set")) {
    // Manual override of the remembered switch function (1..10), -1 = forget.
    int ch = t.substring(fnPrefix.length()).toInt() - 1;
    setFunc(ch, atoi(msg), true);
  }
}

static void mqttLoop() {
  uint32_t now = millis();
  if (mqtt.connected()) {
    lastMqttOk = now;
    mqtt.loop();
    return;
  }
  if (now - lastMqttTry < 5000UL) return;
  lastMqttTry = now;
  if (WiFi.status() != WL_CONNECTED) return;
  String avail = T("status");
  bool ok = (MQTT_USER[0])
      ? mqtt.connect(HOSTNAME, MQTT_USER, MQTT_PASS, avail.c_str(), 0, true, "offline")
      : mqtt.connect(HOSTNAME, nullptr, nullptr, avail.c_str(), 0, true, "offline");
  if (!ok) return;
  mqtt.publish(avail.c_str(), "online", true);
  mqtt.subscribe(T("ch/+/set").c_str());
  mqtt.subscribe(T("func/+/set").c_str());
  mqtt.subscribe(T("bus/cmd").c_str());
  mqtt.subscribe(T("speed/set").c_str());
  mqtt.subscribe(T("eeprom/set").c_str());
  mqtt.subscribe(T("func_set/+/set").c_str());
  mqtt.subscribe(T("table/set/+").c_str());
  mqtt.subscribe(T("table/apply").c_str());
  mqtt.subscribe(T("table/discard").c_str());
  mqtt.subscribe(T("table/restore").c_str());
  mqtt.subscribe(T("table/read").c_str());
  mqtt.subscribe(T("settings/read").c_str());
  mqtt.subscribe(T("cfg/set/+").c_str());
  mqtt.subscribe(T("cfg/undo").c_str());
  mqtt.subscribe(T("scan/run").c_str());
  mqtt.publish(T("eeprom/state").c_str(), eepromEn ? "ON" : "OFF", true);
  publishDiscovery();
  for (int i = 0; i < NCH; i++) publishRelayState(i);
  statePending = true;
}

// ---- setup / loop ---------------------------------------------------------
void setup() {
  // Bus driver disabled before anything else (module pull-down also holds it low).
  if (BUS_DE_PIN >= 0) { pinMode(BUS_DE_PIN, OUTPUT); digitalWrite(BUS_DE_PIN, LOW); }
  // Relays next, and open: set the level, then make the pin an output.
  for (int i = 0; i < NCH; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);               // must come first on ESP32 core 3.x
    digitalWrite(RELAY_PINS[i], RELAY_INACTIVE);
  }
#if LED_PIN >= 0
  pinMode(LED_PIN, OUTPUT);
#endif
  Serial.begin(115200);
  Serial.println("\nHRV relay controller - all relays open");

  prefs.begin("hrv", false);
  for (int i = 0; i < 3; i++) {
    char k[4] = { 'f', (char)('0' + i), 0 };
    funcCode[i] = prefs.getInt(k, -1);
  }
  cfgPrevA = prefs.getInt("cpa", -1);
  cfgPrevV = prefs.getInt("cpv", -1);
  guard.begin(millis());
  updateBlocks();

  Serial2.begin(BUS_BAUD, SERIAL_8N1, BUS_RX_PIN, BUS_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(3300);
  mqtt.setSocketTimeout(2);

  if (OTA_PASS[0]) {
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASS);
    ArduinoOTA.onStart([]() { guard.allOff(millis()); applyRelays(); });
    // ArduinoOTA.begin() is called from loop() once WiFi is connected.
  }
}

void loop() {
  uint32_t now = millis();

  pollBus();
  mqttLoop();
  if (OTA_PASS[0]) {
    if (!otaStarted && WiFi.status() == WL_CONNECTED) {   // OTA must start after WiFi is up
      ArduinoOTA.begin();
      otaStarted = true;
      Serial.printf("WiFi connected, IP %s, OTA ready (%s.local)\n", WiFi.localIP().toString().c_str(), HOSTNAME);
      statePending = true;
    }
    if (otaStarted) ArduinoOTA.handle();
  }
  now = millis();   // refresh: MQTT callbacks above may have used a later time

  // Fail open if the network or broker has been gone too long.
  if (now - lastMqttOk > FAILSAFE_MS && guard.closedSwitches() + guard.isOn(3) > 0) {
    guard.allOff(now);
    Serial.println("failsafe: no MQTT, all relays opened");
    statePending = true;
  }

  serviceCmd(now);

  if (eepromEn && (int32_t)(now - eepromEnUntil) >= 0) { eepromEn = false; eepromDirty = true; }
  if (mqtt.connected()) {
    if (eepromDirty) {
      mqtt.publish(T("eeprom/state").c_str(), eepromEn ? "ON" : "OFF", true);
      eepromDirty = false; statePending = true;
    }
    if (relayDirty) {
      for (int i = 0; i < NCH; i++) publishRelayState(i);
      relayDirty = false;
    }
  }

  uint8_t opened = guard.tick(now);
  applyRelays();
  if (opened) {
    statePending = true;
    if (mqtt.connected())
      for (int i = 0; i < NCH; i++) if (opened & (1u << i)) publishRelayState(i);
  }

  if (mqtt.connected() && (statePending || now - lastPublish > 10000UL)) {
    publishState();
    lastPublish = now;
    statePending = false;
  }
  if (mqtt.connected() && now - lastDisc > 3600000UL) {   // refresh retained discovery hourly
    lastDisc = now;
    publishDiscovery();
  }
#if LED_PIN >= 0
  digitalWrite(LED_PIN, mqtt.connected() ? HIGH : LOW);
#endif
}
