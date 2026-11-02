# titon-mvhr

ESP32 firmware that monitors and controls a **Titon HRV1.6 Q Plus HMB** MVHR unit (Aura-T controller) from
Home Assistant over MQTT. It listens passively to the unit's RS-485 bus, can switch the unit's three
volt-free inputs (S1-S3) with relays, and, behind several hardware and software interlocks, can read and
write bus registers (fan speed, speed table, settings).

> **Status:** bench-tested on a unit that is not yet installed. Use at your own risk. The Aura-T bus
> protocol was reverse-engineered, not documented by Titon. **Never write register 068 (factory reset).**

This repository replaces an earlier ESPHome-style implementation (still in the git history). That
protocol is a related but different version: registers 326 and 290 match, 036 (humidity), 341 (filter) and 060 (hours) do not.

## What it does
- Decodes the Aura-T <-> unit bus (1200 baud 8N1 ASCII, `AAAWsDDDDD` requests, `AAAsDDDDD` replies): supply/extract fan %, temperatures, running hours, status bits.
- Publishes everything to MQTT with Home Assistant discovery (sensors, binary sensors, switches, selects, numbers, buttons).
- Four relays: S1, S2, S3 (one closed at a time, each with a timer) and relay 4, which cuts power to the Aura-T so the ESP32 can talk on the bus.
- Commands from HA: speed 1-4, read any register, read all known settings, edit and save the 8-value speed table, change switch functions and a few settings (guarded).
- Home Assistant dashboard objects are described in the project notes; entity ids follow the discovery names.

## Confirmed register map (Aura-T HMB, 2026-11)
| Reg | Meaning |
|---|---|
| 380 / 381 | supply / extract fan, 0.1 % steps |
| 382 / 383 | outside / extract air temp, x10 C |
| 384 | status bits (below) |
| 385 | speed command 1-4 (speed 1 reverts to 2 after about 1 unit-hour) |
| 070 | running hours (ticks about 63.7 min) |
| 050 | room RH, x10 % (updates when humidity boost is active) |
| 326 | boost inhibit (0/1, the padlock) |
| 290 | SUMMERboost, inverted (0 = enabled) |
| 231 / 232 | bypass thresholds, x10 C |
| 241 | humidity set point, x10 % |
| 250 / 253 | boost overrun timers |
| 345 / 346 | filter interval (months) / hours remaining (= 345 x 730 - running hours) |
| 300-307 | speed table (supply/extract for speeds 1-4) |
| 330-332 | S1-S3 function codes |

Status 384 bits: 0-3 speed 1-4, 5 switch closed, 6 humidity control active, 8 boost inhibit, 9 overrun, 12 bypass, 15 SUMMERboost, 11/13/14 faults.

## Hardware
ESP32 4-relay board (relay GPIO 32/33/25/26, LED 23) plus a 3.3 V RS-485 transceiver (MAX3485 Module TTL to RS485): 
RO to GPIO16, DI to GPIO17, DE and /RE tied together to GPIO27 with a 10k pull-down. Pins and limits are in `config.h`.
Relay 4 should switch only the Aura-T supply through NC/COM so every failure leaves it powered.
Fasizi ESP32 4-Channel Relay Module with WiFi/Bluetooth IoT Development Board AC220V/DC5-30V ESP32-WROOM-32E

## Build and flash
1. Install Arduino or PlatformIO with the ESP32 core and the **PubSubClient** library. Board: ESP32 Dev Module.
2. `cp secrets.h.example secrets.h` and fill in Wi-Fi, MQTT and OTA password. `secrets.h` is git-ignored.
3. Build `hrv_relay.ino` and flash (USB first, then OTA).

## Host tests
```
g++ -std=c++17 -Itest/stubs -I. test/test_host.cpp -o /tmp/t && /tmp/t
```
`test/stubs/` holds Arduino stand-ins and a placeholder `secrets.h`.

## Safety summary
Relays are open at boot; at most one of S1-S3 closed; every close has a timer and needs re-sending; 15 min without MQTT opens everything. Nothing is transmitted unless relay 4 has been energised for 3 s or more, at least 3 s remain on its timer and the bus has been quiet for 1.5 s. EEPROM-backed settings need `ALLOW_WRITE_SETTINGS` in `config.h` and the HA "EEPROM write enable" switch.

Licence: GPL v2 (see LICENSE).

---

# Detailed notes

Files: hrv_relay.ino (sketch), config.h (pins, limits, allowed switch functions), relay_guard.h (interlock/timers),
hrv_bus.h (RS-485 parser), mini_json.h, secrets.h.example (copy to secrets.h). test/ = host tests + Arduino stubs.
Library needed: PubSubClient. Board: ESP32 Dev Module.

## Bring-up checklist (in order)
1. DC supply first (7-30 V), nothing wired to relay contacts, no RS-485 module. Mains only after the isolation check.
2. Confirm pins in config.h against your board (ESPHome entry said relays GPIO32/33/25/26, LED 23).
3. Flash. Boot: all four relays must stay OPEN (no click) through reset and power-up. If any click at boot, change RELAY_ACTIVE_LEVEL / stop.
4. Home Assistant: four switches appear. Closing S1-S3 is refused at first ("function not allowed or unknown") - this is the guard.
5. Tell the guard what S1-S3 are programmed as: publish an integer to hrv_relay/func/1/set (/2, /3), e.g. 5, 1, 3 - or let it learn
   from the bus (power-cycle the Aura-T / open its switch menu so it reads 330-332). Codes outside the allowed list in config.h stay blocked.
6. Test relays with a meter across the contacts: only one of S1-S3 closes at a time; S2 drops after 15 min by default (max 90).
   Send "ON,5" for a 5 minute hold. Send OFF to open.
7. Only then wire contacts to the HRV switch terminals (volt-free, S1/S2/S3 to their common as per the unit's terminal block).
8. RS-485 (optional): module powered 3.3 V, RO -> GPIO16, DE and /RE tied to GND, DI unconnected, A/B/0V only to J9. No termination or bias.
   Sensors appear once the Aura-T is awake and polling. bus_age_s shows time since last frame.

## Relay 4 = write relay (Aura-T power cut)
Energised = Aura-T unpowered, so it stops polling and the bus is quiet. Wire only the Aura-T supply through NC/COM, so every
failure (boot, crash, timeout) leaves it powered. Default cut 2 min, hard max 5 min, then it re-powers on its own. It is not
interlocked with S1-S3.

## Transmit (only with relay 4 energised)
Wiring: DI -> GPIO17, DE and /RE tied together -> GPIO27 with a 10k pull-down to GND at the module (receive at reset).
Nothing is sent unless ALL are true, re-checked immediately before DE goes high: relay 4 energised for 3 s or more, at least 3 s
left on its timer, no bus byte received for 1.5 s. If relay 4 opens, a queued command is dropped.
Use: 1) publish ON,5 to hrv_relay/ch/4/set   2) wait ~4 s   3) publish "R 385" or "W 385 2" to hrv_relay/bus/cmd
   4) read the outcome on hrv_relay/bus/result ("R 385 = 2", "W 385 2 echoed ok", "ECHO MISMATCH", "no reply", or a rejection).
One command at a time. Writable: 160, 161, 300-307, 385. Reads: any address. 330-332 and the other EEPROM-backed settings are
refused unless ALLOW_WRITE_SETTINGS is set to 1 in config.h (leave at 0 until you decide).
Bench test first with the unit NOT connected: put a scope/USB-RS485 on A/B and confirm DE is low at boot and only pulses during a send.

## Home Assistant controls (dashboard "HRV Control")
- hrv_relay/speed/set  "Speed 1".."Speed 4" -> writes 385. Firmware cuts the Aura-T (relay 4) itself, sends, releases (a few seconds).
- hrv_relay/func_set/N/set  function name -> writes 330/331/332. Needs "EEPROM write enable" ON (self-clears after 5 min).
- hrv_relay/eeprom/set ON|OFF. Settings writes are refused while it is OFF. Offered functions are in config.h (FUNC_OPTIONS); Off NO/NC are not offered.
- Manual commands: prefix "A " to let the firmware drive relay 4, e.g. "A R 385".
- Results appear on hrv_relay/bus/result ("HRV last bus result").

## Behaviour
- Never more than MAX_CLOSED_SWITCHES of S1-S3 closed (hard-capped below 3). 10 s boot hold-off. 2 s minimum open time.
- Every close has a timer; Home Assistant must re-send ON to extend. 15 min without MQTT opens everything. OTA start opens everything.
- Topics: hrv_relay/ch/N/set (ON | ON,<minutes> | OFF), /state, /result; hrv_relay/state (JSON); hrv_relay/func/N/set.
- Writes by the Aura-T are logged (last_write).

## Commissioning the speed table from Home Assistant

Needs the **EEPROM write enable** switch on (it self-clears after 5 min). The HRV Control
dashboard then shows a Commissioning card: supply/extract % for Speed 1-4 (registers 300-307,
0.1 % steps, limited to TABLE_MIN/TABLE_MAX in config.h = 14-100 %).

1. Press **Read from unit** (firmware cuts Aura-T power with relay 4, reads 300-307, releases). Shows the live values.
2. Edit the numbers. Edits are only *staged* in the ESP32 ("Edits staged, not saved" = on). Nothing is sent yet.
3. Press **SAVE to unit**. Sequence: read all 8 -> back them up (flash) -> W160=1 -> W300..W307 (0.5 s apart) -> W161=0 -> read all 8 back and compare.
   Any missing reply, echo mismatch, or relay 4 timing out aborts and releases relay 4. The result is in "Table status".
4. **Restore previous** rewrites the table backed up just before the last save.
5. **Discard edits** drops staged values.

Caveats: the minimal save sequence (no W160=0 commissioning session first) is UNPROVEN on the unit; do a
tiny change first (e.g. Speed 1 supply by 0.5 %) and confirm read-back and restore. RS-485 transmit hardware
must work first (`W 385` test). If the check shows VERIFY FAILED, press Restore previous.
Entity ids for the new number/button entities are predicted; if a dashboard row shows "not found", fix its id.

## Transmit test (no unit needed)
Send `A T` on hrv_relay/bus/cmd. Relay 4 is energised by the firmware and 16 bytes of 0x55 ('U') go out at 1200 8N1.
A PC terminal at 1200 8N1 should show `UUUUUUUUUUUUUUUU`. Anything else is wiring/baud/polarity, not the unit.
Use `A T 8` to keep sending for 8 s (max 10) so a meter can measure DE (3.3 V) and DI (about 1.6 V average).

## Finding out which register holds each Aura-T setting

The Aura-T manual lists what is configurable (humidity set point, the two boost overrun
timers, the summer-bypass thresholds, SUMMERboost on/off, filter interval, switch
functions). Which bus register holds each one has NOT been proven. The candidates in
`settings_map.h` come from matching the values the Aura-T reads at boot against the
manual's defaults (231=220, 232=150, 240=1, 241=700, 250=10, 253=30, 345=6). Treat them
as guesses until proven.

1. Press **HRV read unit settings**. The firmware cuts the Aura-T for a few seconds (relay 4),
   reads each candidate register plus 330-332, then restores it. Values appear as
   `HRV setting NNN ...` sensors. Read only, nothing is written.
2. With the Aura-T powered and the relay released, change ONE setting on the Aura-T
   (Setup Mode, see the manual) and save it. The register it writes shows up in
   **HRV Aura-T writes seen** as `address=value (N s ago)`. (124/125 are left out.)
3. Press **HRV read unit settings** again to confirm the value moved. Note the scaling
   (for example 700 = 70.0 %).
4. Change the setting back, and repeat for the next one.

Only after a register is proven should writes from Home Assistant be switched on for it.
Registers 028 and 290 are never writable from this firmware.
