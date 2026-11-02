#pragma once
// ============================================================================
//  HRV relay board - configuration
//  Board: ESP32-WROOM-32E N4, 4 relays, AC/DC powered (pins below come from the
//  ESPHome device entry for this board family. CONFIRM THEM ON YOUR BOARD).
// ============================================================================
#include <stdint.h>
#include "relay_guard.h"

// ---- relay outputs ---------------------------------------------------------
// Channels 1-3 = unit switch inputs S1..S3 (interlocked).
// Channel 4 = WRITE RELAY: energised = Aura-T power CUT. Wire the Aura-T supply
// through this relay's NC and COM contacts, so a de-energised relay (boot, crash,
// power loss, timeout) leaves the Aura-T powered. Only the supply line goes through it.
#define RELAY_PIN_1 32
#define RELAY_PIN_2 33
#define RELAY_PIN_3 25
#define RELAY_PIN_4 26
// HIGH = coil energised (contact closed). VERIFY on the bench with nothing wired
// to the relay contacts: all four relays must be OPEN (silent) after boot.
#define RELAY_ACTIVE_LEVEL HIGH
#define LED_PIN 23

// ---- RS-485 ----------------------------------------------------------------
// RO -> BUS_RX_PIN, DI -> BUS_TX_PIN, DE and /RE tied together -> BUS_DE_PIN.
// FIT A 10k PULL-DOWN from DE//RE to GND at the module, so the transceiver is in
// receive at reset and while the ESP32 pins float. The firmware only raises
// BUS_DE_PIN while sending a frame, and only if the transmit interlock below passes.
#define BUS_RX_PIN 16
#define BUS_TX_PIN 17
#define BUS_DE_PIN 27   // GPIO that drives DE//RE. Use -1 for an auto-direction module (TXD/RXD/EN type) with no DE pin.
#define BUS_BAUD 1200

// ---- TRANSMIT INTERLOCK ----------------------------------------------------
// Nothing is ever transmitted unless relay 4 (Aura-T power cut) is energised AND:
#define TX_SETTLE_MS 3000UL       // relay 4 has been energised at least this long
#define TX_QUIET_MS 1500UL        // no byte received on the bus for this long
#define TX_MIN_REMAIN_MS 3000UL   // relay 4 has at least this long left on its timer
#define TX_REPLY_MS 800UL         // wait this long for the unit's reply/echo
#define READ_RETRIES 3            // a read with no reply is re-sent this many times (never writes)
#define TX_WAIT_MAX_MS 15000UL    // a queued command is dropped after this long
// Compile-time ceiling. 0 = only 160/161/300-307/385 can ever be written. 1 also allows
// 330-332, 231/232, 241, 250, 253, 290, 345 (EEPROM-backed settings) - but ONLY while
// the runtime "EEPROM write enable" switch is on (it switches itself off after
// EEPROM_ENABLE_MS). Speed selection (385) never needs the enable.
#define ALLOW_WRITE_SETTINGS 1
#define EEPROM_ENABLE_MS (5UL * 60UL * 1000UL)
// When Home Assistant sends an "A ..." command the firmware energises relay 4 itself,
// sends, then releases it. This is the longest it will hold relay 4 for that.
#define AUTO_CUT_MS 30000UL

// Speed-table editing from Home Assistant (registers 300-307, units of 0.1 %).
// Limits for any value you can stage. The unit's own range is 14-100 %.
#define TABLE_MIN 140
#define TABLE_MAX 1000
// How long relay 4 is held for a register scan (24 reads, absent ones can take ~1 s each).
#define SCAN_CUT_MS 180000UL
// 1 = a setting write is followed by W161=0, exactly as the Aura-T does when it leaves a menu.
// 0 = write the setting only. Not proven either way for persistence: test with a power cycle.
#define CFG_SEND_MENU_EXIT 1
// How long relay 4 is held (Aura-T off) for a table read/save/restore sequence.
#define TABLE_CUT_MS 60000UL

// Dropdown choices shown in Home Assistant for the switch inputs (writes 330-332).
// Only functions proven on the unit. Off (NO)/Off (NC) are deliberately NOT offered:
// an Off (NC) input stops the fans whenever its relay is open.
struct FuncOpt { int code; const char* name; };
static const FuncOpt FUNC_OPTIONS[] = {
  { 1, "Wet room boost" }, { 2, "Kitchen boost" }, { 3, "Speed 1 / Setback" },
  { 5, "SUMMERboost disable" }, { 9, "Manual summer bypass" },
};
static const int N_FUNC_OPTIONS = sizeof(FUNC_OPTIONS) / sizeof(FUNC_OPTIONS[0]);

// ---- network ---------------------------------------------------------------
#define MQTT_BASE "hrv_relay"
#define HOSTNAME "hrv-relay"

// ---- safety limits ---------------------------------------------------------
// At most this many of S1..S3 closed at once. Hard-capped at 2 in code: all
// three closed at once is never allowed (factory-reset condition on the unit).
#define MAX_CLOSED_SWITCHES 1
#define BOOT_HOLD_MS 10000UL      // no relay may close for this long after boot
#define MIN_OFF_MS 2000UL         // a channel must stay open this long before re-closing
#define MIN_GAP_MS 1000UL         // minimum time between any two relay changes
#define FAILSAFE_MS (15UL * 60UL * 1000UL)  // MQTT lost this long -> open every relay

// Per channel: default hold time and hard maximum (ms). The relay always opens
// when the time is up; Home Assistant must re-send to keep it closed.
//   S2 is capped at 90 min: the Aura-T flashes a Boost Alert after 2 h in boost.
//   Note the unit adds its own overrun after S2 is released.
static const hrv::ChannelLimits CH_LIMITS[4] = {
  { 4UL * 3600000UL, 24UL * 3600000UL },   // S1
  { 15UL * 60000UL,  90UL * 60000UL   },   // S2
  { 4UL * 3600000UL, 24UL * 3600000UL },   // S3
  { 2UL * 60000UL,   5UL * 60000UL    },   // Aura-T power cut (write window)
};

static const char* const CH_NAMES[4] = {
  "HRV switch S1", "HRV switch S2", "HRV switch S3", "HRV Aura-T power cut"
};

// ---- function guard --------------------------------------------------------
// The firmware watches the Aura-T's switch-function settings on the bus
// (registers 330/331/332 = S1/S2/S3) and refuses to close a relay whose input
// is set to anything outside its allowed list. Set these to match how you have
// programmed S1..S3. Function codes (from the decode in hmem):
//   1 Wet room boost   2 Kitchen boost   3 Speed 1 / Setback
//   5 SUMMERboost disable   6 Speed 4 (unproven)   9 Manual summer bypass
//   10 Boost inhibit (unproven)
//   7 Off (NO), 8 Off (NC): NEVER allowed. Closing an Off input stops the fans,
//   and an Off (NC) input would stop the unit whenever the relay is open.
#define FN_BIT(code) (1u << (code))
static const uint16_t CH_ALLOWED_FUNCS[3] = {
  (uint16_t)(FN_BIT(5)),                  // S1: SUMMERboost disable
  (uint16_t)(FN_BIT(1) | FN_BIT(2)),      // S2: wet room or kitchen boost (speed 3 + overrun)
  (uint16_t)(FN_BIT(3)),                  // S3: Speed 1 / Setback
};
// 1 = refuse to close S1..S3 until the function code has been seen on the bus
// (it is then remembered across reboots). Power-cycle the Aura-T once, or
// re-save the switch function on it, to make it appear. 0 = allow when unknown.
#define REQUIRE_KNOWN_FUNCTION 1
