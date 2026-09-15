/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2025  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
 *                          Ben Bament
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <MGgen2V2Lcharger.h>

/* Control of the MG Gen 2 V2L charger.
 *
 * MOD_RUN (V2L) sequencing rewritten against the OEM captures
 * MG_Hybrid_PTcan_{Start,Shutdown,Discharge}_20240311. Every value below
 * is quoted from those logs; comments give the log time it was observed.
 * Hybrid CAN (CCU side) = "bus 3" in the logs, PT CAN = "bus 1".
 *
 * Only file-static state was added so MGgen2V2Lcharger.h needs no change.
 */

uint8_t MGgen2V2Lcharger::chgStatus;
uint8_t MGgen2V2Lcharger::evseDuty;
float MGgen2V2Lcharger::dcBusV;
float MGgen2V2Lcharger::temp_1;
float MGgen2V2Lcharger::temp_2;
float MGgen2V2Lcharger::ACVolts;
float MGgen2V2Lcharger::ACAmps;
float MGgen2V2Lcharger::DCAmps;
float MGgen2V2Lcharger::LV_Volts;
float MGgen2V2Lcharger::LV_Amps;
uint16_t MGgen2V2Lcharger::batteryVolts;
uint8_t MGgen2V2Lcharger::dcDcTimer = 0;
uint8_t MGgen2V2Lcharger::dcDcCounter = 0;
uint8_t MGgen2V2Lcharger::v2lCounter = 0;   // rolling 4-bit counter for 0x08E
uint8_t MGgen2V2Lcharger::v2lHeartbeat = 0; // rolling counter for 0x297 full frame
static uint8_t PlugStat = 0;
static bool PPStat = false;

// ---------------------------------------------------------------------------
// V2L (MOD_RUN) sequencing state — all in 100 ms ticks unless noted.
// ---------------------------------------------------------------------------

// Timeline after entering RUN (HV is already up when RUN starts, so this
// compresses the OEM "contactors just closed" phase):
//   0x396 D5 : 0x40 for the first few ticks, then 0xC0  (OEM 11.07s -> 11.37s)
//   0x297 D2 : 0x02 (precharge) briefly, then 0x03 and it STAYS 0x03
//              (OEM 10.97s -> 11.87s; never returns to 0x01 with HV up)
//   0x08E/099: "not available" pattern, then live pattern ~1s after HV
//              (OEM 11.85s / 11.94s)
//   V2L request at V2L_REQ_TICK: 0x297 D2 |= 0x20 for 5 ticks, 0x29C cleared
//              on the 4th tick, then D1=0x01 latched + 0x33F=0x46
//              (OEM 24.87s -> 25.26s -> 25.37s -> 25.47s)
//   Then WAIT for the CCU: 0x33D D3 0x10 -> 0x30 (OEM took 5.1s).
//   Enable, timed from the 0x30 tick:
//     +3  0x322 D4=0x20            (OEM 30.87s)
//     +4  0x29C = discharge limits (OEM 30.96s)
//     +6  0x33F = 0x48             (OEM 31.17s) -> CCU answers 0x33D D3=0x60
//   After 0x33D D3=0x60 is seen:
//     +1  0x29B D4 0x02 -> 0x00    (OEM 31.28s)
//     +3  0x391 D5 0xB0 -> 0xD0    (OEM 31.45s)
#define V2L_T_HVFLAG      3    // 0x396 D5 0x40 -> 0xC0
#define V2L_T_D2_RUN      5    // 0x297 D2 0x02 -> 0x03 (OEM ~0.9 s)
#define V2L_T_BMS_VALID   10   // 0x08E / 0x099 switch to live pattern
#define V2L_T_D7_A        50   // 0x297 D7 high nibble 0xB -> 0xA (OEM ~4.8 s)
#define V2L_REQ_TICK      100  // when we raise the V2L request (OEM: 14 s after HV, user-driven)
#define V2L_REQ_LEN       5    // D2=0x23 window (OEM 500 ms)
// 0 = wait indefinitely for 0x33D D3==0x30 before enabling the output.
// Set to N to fall back to a timed enable N ticks after the request
// (the old behaviour) — NOT recommended, kept for experiment only.
#define V2L_READY_TIMEOUT 0

static uint8_t ccu33D_D3 = 0;       // last 0x33D D3 from the CCU (0x10/0x30/0x60)
static bool v2lRequested = false;   // D1=0x01 latched, 0x46 announced
static bool v2lReady = false;       // CCU showed 0x33D D3==0x30
static bool v2lOutputOn = false;    // CCU showed 0x33D D3==0x60
static uint8_t readyTick = 0;       // ticks since 0x30 seen (saturating)
static uint8_t outputTick = 0;      // ticks since 0x60 seen (saturating)
static uint8_t ccu099Counter = 0;   // 0x099 D2 counter
static uint8_t chg394Counter = 0;   // 0x394 / 0x39B counter

static inline uint8_t xor7(const uint8_t *b) {
  return b[0] ^ b[1] ^ b[2] ^ b[3] ^ b[4] ^ b[5] ^ b[6];
}

static void ResetV2LState() {
  ccu33D_D3 = 0;
  v2lRequested = false;
  v2lReady = false;
  v2lOutputOn = false;
  readyTick = 0;
  outputTick = 0;
}

bool MGgen2V2Lcharger::ControlCharge(bool RunCh, bool ACReq) {
  int chgmode = Param::GetInt(Param::interface);
  switch (chgmode) {
  case Unused:
    if (PlugStat == 1 && ACReq) {
      clearToStart = true;
      return true;
    } else {
      clearToStart = false;
      return false;
    }

    break;

  case i3LIM:
    if (RunCh &&
        ACReq) // we have a startup request to AC charge from a charge interface
    {
      clearToStart = true;
      return true;
    } else {
      clearToStart = false;
      return false;
    }
    break;

  case CPC:
    if (RunCh &&
        ACReq) // we have a startup request to AC charge from a charge interface
    {
      clearToStart = true;
      return true;
    } else {
      clearToStart = false;
      return false;
    }
    break;

  case Foccci:
    if (RunCh &&
        ACReq) // we have a startup request to AC charge from a charge interface
    {
      clearToStart = true;
      return true;
    } else {
      clearToStart = false;
      return false;
    }
    break;

  case Chademo:
    if (RunCh && ACReq) {
      clearToStart = true;
      return true;
    } else {
      clearToStart = false;
      return false;
    }

    break;
  }
  return false;
}

void MGgen2V2Lcharger::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x324);
  can->RegisterUserMessage(0x39F);
  can->RegisterUserMessage(0x323);
  can->RegisterUserMessage(0x33B);
  can->RegisterUserMessage(0x33D); // V2L handshake: D3 0x10 -> 0x30 -> 0x60
}

// 0x33D is CCU_2_Car. D3 = 0x10 idle, 0x30 "ready for output enable",
// 0x60 output live. D1/D2 carry output current/voltage once live.
static void handle33D(uint32_t data[2]) {
  uint8_t *bytes = (uint8_t *)data;
  ccu33D_D3 = bytes[2];
}

void MGgen2V2Lcharger::DecodeCAN(int id, uint32_t data[2]) {
  switch (id) {
  case 0x324:
    MGgen2V2Lcharger::handle324(data);
    break;
  case 0x39F:
    MGgen2V2Lcharger::handle39F(data);
    break;
  case 0x323:
    MGgen2V2Lcharger::handle323(data);
    break;
  case 0x33B:
    MGgen2V2Lcharger::handle33B(data);
    break;
  case 0x33D:
    handle33D(data);
    break;
  }
}

void MGgen2V2Lcharger::Task100Ms() {
  int opmode = Param::GetInt(Param::opmode);

  // set max voltage on charger
  setVolts = Param::GetInt(Param::Voltspnt);
  if (setVolts < 353.0f)
    setVolts = 353.0f; // minimum voltage
  if (setVolts > 453.0f)
    setVolts = 450; // maxiumum voltage

  // Convert voltage to 16-bit value (multiply by 50). Add a 0.5v offset,
  // otherwise it ramps down so slowly it will never hit vltspnt
  uint16_t voltage_encoded = static_cast<uint16_t>((setVolts + 0.5) * 50.0f);

  uint8_t bytes[8];

  // =========================================================================
  // MOD_RUN: V2L + DC-DC
  // =========================================================================
  if (opmode == MOD_RUN) {
    if (V2Ltimer < 200)
      V2Ltimer++;

    // ---- V2L handshake tracking (from 0x33D) ----------------------------
    if (v2lRequested && !v2lReady && ccu33D_D3 == 0x30)
      v2lReady = true;
#if V2L_READY_TIMEOUT > 0
    if (v2lRequested && !v2lReady &&
        V2Ltimer >= (V2L_REQ_TICK + V2L_REQ_LEN + V2L_READY_TIMEOUT))
      v2lReady = true; // timed fallback (old behaviour)
#endif
    if (v2lReady && readyTick < 250)
      readyTick++;
    if (v2lReady && !v2lOutputOn && ccu33D_D3 == 0x60)
      v2lOutputOn = true;
    if (v2lOutputOn && outputTick < 250)
      outputTick++;

    bool inReqWindow =
        (V2Ltimer >= V2L_REQ_TICK) && (V2Ltimer < V2L_REQ_TICK + V2L_REQ_LEN);
    if (V2Ltimer >= V2L_REQ_TICK + V2L_REQ_LEN)
      v2lRequested = true;

    bool send322Enable = v2lReady && readyTick >= 3;
    bool send29CLimits = v2lReady && readyTick >= 4;
    bool send33F_48 = v2lReady && readyTick >= 6;

    // ---- 0x19C: DC-DC keepalive (PT CAN in the OEM) -----------------------
    // Left as-is: DC-DC works with it. Note OEM keeps D3=0x06 permanently
    // and only shows 0x26 in D1 for one frame at wake; D4 alternates 9F/A0.
    if (dcDcTimer < 23)
      dcDcTimer++;
    bytes[0] = 0x06;
    bytes[1] = 0xA0;
    bytes[2] = (dcDcTimer >= 23) ? 0x26 : 0x06;
    bytes[3] = 0xA0;
    bytes[4] = 0x7F;
    bytes[5] = 0xFF;
    bytes[6] = dcDcCounter & 0x0F;
    bytes[7] = 0x7F;
    dcDcCounter++;
    can->Send(0x19C, (uint32_t *)bytes, 8);

    // ---- 0x08E: BMS fast frame ---------------------------------------------
    // OEM: 80 04 00 00 13 88 until ~1 s after contactors close, then
    // 5D 05 04 00 13 88 for the rest of the session (0x80 looks like a
    // not-available sentinel; 0x5D=93 is plausibly SOC%). XOR in D8.
    bool bmsValid = V2Ltimer >= V2L_T_BMS_VALID;
    bytes[0] = bmsValid ? 0x5D : 0x80;
    bytes[1] = bmsValid ? 0x05 : 0x04;
    bytes[2] = bmsValid ? 0x04 : 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x13;
    bytes[5] = 0x88;
    bytes[6] = v2lCounter & 0x0F;
    bytes[7] = xor7(bytes);
    v2lCounter++;
    can->Send(0x08E, (uint32_t *)bytes, 8);

    // ---- 0x099: BMS fast frame (new — was never sent) ----------------------
    // OEM: 80 nn 80 04 00 80 00  ->  A0 (80|nn) 80 04 00 5D 00. XOR in D8.
    bytes[0] = bmsValid ? 0xA0 : 0x80;
    bytes[1] = (ccu099Counter & 0x0F) | (bmsValid ? 0x80 : 0x00);
    bytes[2] = 0x80;
    bytes[3] = 0x04;
    bytes[4] = 0x00;
    bytes[5] = bmsValid ? 0x5D : 0x80;
    bytes[6] = 0x00;
    bytes[7] = xor7(bytes);
    ccu099Counter++;
    can->Send(0x099, (uint32_t *)bytes, 8);

    // ---- 0x297: BMS state — both frames still required --------------------
    // D2: 0x02 briefly, then 0x03 and it stays 0x03 (HV is up).
    //     V2L request = D2 | 0x20 for 5 ticks. 0x21/0x04/0x0C do not occur
    //     in this car.
    // D1: 0x00 until the request window closes, then 0x01 latched.
    // D5/D6: live value (C3 0B) — 0xFFFF only while D2==0x02 (OEM).
    // D7 hi nibble: 0x9 in 02, 0xB in 03 for ~5 s, then 0xA.
    uint8_t d2 = (V2Ltimer < V2L_T_D2_RUN) ? 0x02 : 0x03;
    if (inReqWindow)
      d2 |= 0x20;
    uint8_t d1 = v2lRequested ? 0x01 : 0x00;
    uint8_t d7hi;
    if (V2Ltimer < V2L_T_D2_RUN)
      d7hi = 0x90;
    else if (V2Ltimer < V2L_T_D7_A)
      d7hi = 0xB0;
    else
      d7hi = 0xA0;

    // Null frame (OEM sends this on PT CAN; both buses are common here).
    // D1 stays 0x00 on this frame in the OEM, D7=0x20 once running.
    bytes[0] = 0x00;
    bytes[1] = d2 & 0x0F;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = (V2Ltimer < V2L_T_D2_RUN) ? 0x00 : 0x20;
    bytes[7] = 0x00; // never XOR on the null frame
    can->Send(0x297, (uint32_t *)bytes, 8);

    // Full frame
    bytes[0] = d1;
    bytes[1] = d2;
    bytes[2] = 0xE0;
    bytes[3] = 0x00;
    bytes[4] = (V2Ltimer < V2L_T_D2_RUN) ? 0xFF : 0xC3;
    bytes[5] = (V2Ltimer < V2L_T_D2_RUN) ? 0xFF : 0x0B;
    bytes[6] = d7hi | (v2lHeartbeat & 0x0F);
    bytes[7] = xor7(bytes);
    v2lHeartbeat++;
    can->Send(0x297, (uint32_t *)bytes, 8);

    // ---- 0x29C: discharge limits ------------------------------------------
    // idle -> cleared on the 4th tick of the request window -> limits at
    // readyTick 4 (OEM 25.26s / 30.96s).
    bytes[0] = 0x28;
    if (send29CLimits) { // 22.0 A, 445.8 V
      bytes[1] = 0x89; bytes[2] = 0x04; bytes[3] = 0x00;
      bytes[4] = 0x00; bytes[5] = 0xDC; bytes[6] = 0x57; bytes[7] = 0x12;
    } else if (V2Ltimer >= V2L_REQ_TICK + V2L_REQ_LEN - 1) { // cleared
      bytes[1] = 0x00; bytes[2] = 0x00; bytes[3] = 0x00;
      bytes[4] = 0x00; bytes[5] = 0x00; bytes[6] = 0x00; bytes[7] = 0x00;
    } else { // idle
      bytes[1] = 0xFF; bytes[2] = 0x83; bytes[3] = 0xFF;
      bytes[4] = 0x00; bytes[5] = 0xFF; bytes[6] = 0x7F; bytes[7] = 0xFF;
    }
    can->Send(0x29C, (uint32_t *)bytes, 8);

    // ---- 0x1F1: wakeup keepalive -----------------------------------------
    // OEM Hybrid CAN: 0E 00 00 00 00 00 00 00 with HV up (D7 was 0x20 here).
    bytes[0] = 0x0E;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x1F1, (uint32_t *)bytes, 8);

    // ---- 0x33F: V2L output relay sequence --------------------------------
    // 0x00 idle -> 0x46 announce (with D1=0x01 latch) -> 0x48 output enable
    // ONLY after the CCU has shown 0x33D D3=0x30. No 0x06 step in this car.
    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x28;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    if (send33F_48)
      bytes[7] = 0x48;
    else if (v2lRequested)
      bytes[7] = 0x46;
    else
      bytes[7] = 0x00;
    can->Send(0x33F, (uint32_t *)bytes, 8);

    // ---- 0x322 (new in RUN) -----------------------------------------------
    // OEM: all zero for the whole session until 300 ms after 0x33D=0x30,
    // then D4=0x20 (sent just before the 0x29C limits and 0x33F=0x48).
    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = send322Enable ? 0x20 : 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x322, (uint32_t *)bytes, 8);

    // ---- 0x396: Car_2_CCU — HV bus status ---------------------------------
    // OEM D5: 0x00 with HV open, 0x40 for ~300 ms as contactors close, then
    // 0xC0 for the whole drive / V2L session (Start AND Discharge logs).
    bytes[0] = 0x44;
    bytes[1] = 0x6E;
    bytes[2] = 0xB4;
    bytes[3] = 0x28;
    bytes[4] = (V2Ltimer < V2L_T_HVFLAG) ? 0x40 : 0xC0;
    bytes[5] = 0x4E;
    bytes[6] = 0x4D;
    bytes[7] = 0x4D;
    can->Send(0x396, (uint32_t *)bytes, 8);

    // ---- 0x29B (new in RUN) -----------------------------------------------
    // OEM V2L: 3B CA 86 02 FD 06 00 00 from wake; D4 -> 0x00 ~100 ms after
    // the CCU reports 0x33D=0x60. Normal drive is 7B CA 86 03 ....
    bytes[0] = 0x3B;
    bytes[1] = 0xCA;
    bytes[2] = 0x86;
    bytes[3] = (v2lOutputOn && outputTick >= 1) ? 0x00 : 0x02;
    bytes[4] = 0xFD;
    bytes[5] = 0x06;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x29B, (uint32_t *)bytes, 8);

    // ---- 0x391 (new) — V2L mode flag present from wake --------------------
    // OEM: 10 01 A0 00 B0 00 48 00 in V2L (D5=0x00 in normal drive),
    // D5 -> 0xD0 ~300 ms after 0x33D=0x60. Sender unconfirmed — check DBC.
    bytes[0] = 0x10;
    bytes[1] = 0x01;
    bytes[2] = 0xA0;
    bytes[3] = 0x00;
    bytes[4] = (v2lOutputOn && outputTick >= 3) ? 0xD0 : 0xB0;
    bytes[5] = 0x00;
    bytes[6] = 0x48;
    bytes[7] = 0x00;
    can->Send(0x391, (uint32_t *)bytes, 8);

    // ---- 0x2A2 (new) — V2L mode flag present from wake --------------------
    // OEM Hybrid CAN: 86 00 20 C8 00 84 00 AC (D3 bit5 = V2L; 0x00 normally).
    // Sender unconfirmed — check DBC.
    bytes[0] = 0x86;
    bytes[1] = 0x00;
    bytes[2] = 0x20;
    bytes[3] = 0xC8;
    bytes[4] = 0x00;
    bytes[5] = 0x84;
    bytes[6] = 0x00;
    bytes[7] = 0xAC;
    can->Send(0x2A2, (uint32_t *)bytes, 8);

    // ---- 0x394 / 0x39B / 0x39A — present throughout OEM V2L -------------
    // (previously CHARGE-only). 0x39B D3 is 0x9D once HV is up.
    bytes[0] = 0x00;
    bytes[1] = 0x28;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x10;
    bytes[5] = 0x43;
    bytes[6] = chg394Counter & 0x0F;
    bytes[7] = xor7(bytes);
    can->Send(0x394, (uint32_t *)bytes, 8);

    bytes[0] = 0x44;
    bytes[1] = 0x43;
    bytes[2] = 0x9D;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = (chg394Counter & 0x0F) << 2; // OEM counter sits in bits 2..5
    bytes[7] = xor7(bytes);
    can->Send(0x39B, (uint32_t *)bytes, 8);
    chg394Counter++;

    bytes[0] = 0x00;
    bytes[1] = 0x43;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00; // OEM D5/D6 is a slowly ramping value; zero at idle
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x39A, (uint32_t *)bytes, 8);
  }

  // =========================================================================
  // MOD_CHARGE: unchanged (charging is reliable with this)
  // =========================================================================
  if (opmode == MOD_CHARGE) {

    bytes[0] = 0x0E; // 0E to wake up
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x20;
    bytes[7] = 0x00;
    can->Send(0x1F1, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x06; // 01 is stand by, 03 is driving, 06 is AC charging, 07
                     // is CCS charging
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x20;
    bytes[6] = 0x20; // 20 to wake up charger.
    bytes[7] = 0x00;
    can->Send(0x297, (uint32_t *)bytes, 8);

    bytes[0] = 0x3B;
    bytes[1] = 0xCA;
    bytes[2] = 0x86;
    bytes[3] = 0x00;
    bytes[4] = 0xFD;
    bytes[5] = 0x60;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x29B, (uint32_t *)bytes, 8);

    bytes[0] = 0x44;
    bytes[1] = 0x43;
    bytes[2] = 0x9D;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = dcDcCounter & 0x0F;
    bytes[7] = xor7(bytes);
    can->Send(0x39B, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x28;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x41;
    can->Send(0x33F, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x20;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x322, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x28;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x10;
    bytes[5] = 0x43;
    bytes[6] = dcDcCounter & 0x0F;
    bytes[7] = xor7(bytes);
    can->Send(0x394, (uint32_t *)bytes, 8);

    // DC-DC enable is gated: real MG5 holds D3=0x06 for ~2.3s after power-on
    // before flipping D3 to 0x26 to enable the DC-DC. Replicating that here
    // prevents the charger getting confused on a run->off->run cycle.
    if (dcDcTimer < 23)
      dcDcTimer++;
    bytes[0] = 0x06;
    bytes[1] = 0xA0;
    bytes[2] =
        (dcDcTimer >= 23) ? 0x26 : 0x06; // 0x26 enables DC-DC, 0x06 holds off
    bytes[3] = 0xA0;                     // real MG5 value (was 0x00)
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = dcDcCounter & 0x0F; // rolling 4-bit counter (0x00-0x0F)
    bytes[7] = 0x7E;
    dcDcCounter++;
    can->Send(0x19C, (uint32_t *)bytes, 8);
  }

  // 0x29C is handled inside MOD_RUN with the correct V2L sequencing.
  // Only send the charge/idle value when NOT in RUN mode to avoid
  // fighting the V2L 0x29C send and confusing the charger.
  if (opmode != MOD_RUN) {
    if (clearToStart) {
      bytes[0] = 0x28;
      bytes[1] = 0x89;
      bytes[2] = 0x07;
      bytes[3] = 0xFE;
      bytes[4] = 0x00;
      bytes[5] = 0xDC;
      bytes[6] = (voltage_encoded >> 8) & 0xFF;
      bytes[7] = voltage_encoded & 0xFF;
      can->Send(0x29C, (uint32_t *)bytes, 8);
    } else {
      bytes[0] = 0x00;
      bytes[1] = 0x00;
      bytes[2] = 0x00;
      bytes[3] = 0x00;
      bytes[4] = 0x00;
      bytes[5] = 0x00;
      bytes[6] = 0x00;
      bytes[7] = 0x12;
      can->Send(0x29C, (uint32_t *)bytes, 8);
    }
  }
}

void MGgen2V2Lcharger::Off() {

  V2Ltimer = 0;      // reset V2L timer
  dcDcTimer = 0;     // reset DC-DC startup gate — forces re-sequencing on next RUN
  dcDcCounter = 0;   // reset rolling counter
  v2lCounter = 0;    // reset 0x08E counter
  v2lHeartbeat = 0;  // reset 0x297 counter
  ResetV2LState();   // forget the 0x33D handshake

  // Everything below is the OEM *idle / HV-open* value (Shutdown log) so a
  // power cycle never leaves the CCU holding a run-mode flag. Changed vs the
  // previous version: 0x396 D5 0x80->0x00, 0x322 D4 0x20->0x00,
  // 0x1F1 D7 0x20->0x00, 0x29B -> drive-idle 7B/03 pattern.
  // NB the OEM sends 0x297 D2=0x08 exactly once and then goes silent within
  // 20 ms; we still stream for the rlyDly period.
  uint8_t bytes[8];
  bytes[0] = 0x46; // TRUE off state
  bytes[1] = 0xA0;
  bytes[2] = 0x06; // DC-DC off
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x19C, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x01;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x297, (uint32_t *)bytes, 8);

  bytes[0] = 0x28;
  bytes[1] = 0xFF;
  bytes[2] = 0x83;
  bytes[3] = 0xFF;
  bytes[4] = 0x00;
  bytes[5] = 0xFF;
  bytes[6] = 0x7F;
  bytes[7] = 0xFF;
  can->Send(0x29C, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00; // OEM idle 0x1F1 is all zero
  bytes[7] = 0x00;
  can->Send(0x1F1, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x28;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00; // idle
  can->Send(0x33F, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00; // OEM idle 0x322 D4 is 0x00 (0x20 is the output-enable flag)
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x322, (uint32_t *)bytes, 8);

  bytes[0] = 0x44;
  bytes[1] = 0x43;
  bytes[2] = 0x00; // OEM: 0x00 at wake / HV open
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x07; // XOR
  can->Send(0x39B, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x43;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x39A, (uint32_t *)bytes, 8);

  bytes[0] = 0x44;
  bytes[1] = 0x6E;
  bytes[2] = 0xB4;
  bytes[3] = 0x28;
  bytes[4] = 0x00; // HV open (was 0x80 — never seen in the OEM logs)
  bytes[5] = 0x4E;
  bytes[6] = 0x4D;
  bytes[7] = 0x4D;
  can->Send(0x396, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x343, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x32E, (uint32_t *)bytes, 8);

  bytes[0] = 0x7B; // OEM key-off idle: 7B CA 86 03 FD 06 00 00
  bytes[1] = 0xCA;
  bytes[2] = 0x86;
  bytes[3] = 0x03;
  bytes[4] = 0xFD;
  bytes[5] = 0x06;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x29B, (uint32_t *)bytes, 8);

  // 0x391 / 0x2A2 idle (no V2L flag) so the mode does not stick
  bytes[0] = 0x10;
  bytes[1] = 0x01;
  bytes[2] = 0xA0;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x48;
  bytes[7] = 0x00;
  can->Send(0x391, (uint32_t *)bytes, 8);

  bytes[0] = 0x86;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0xC8;
  bytes[4] = 0x00;
  bytes[5] = 0x84;
  bytes[6] = 0x00;
  bytes[7] = 0xAC;
  can->Send(0x2A2, (uint32_t *)bytes, 8);
}

void MGgen2V2Lcharger::handle324(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into
                       // bytes. See comments are useful:)
  batteryVolts = ((bytes[1] << 8) | (bytes[2])) * 0.02;

  if (Param::GetInt(Param::ShuntType) == 0 &&
      Param::GetInt(Param::Inverter) !=
          1) // Only populate if no shunt is used and not using Leaf inverter
             // !!!look to clean up
  {
    // Param::SetFloat(Param::udc, batteryVolts);
  }
}

void MGgen2V2Lcharger::handle39F(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into
                       // bytes. See comments are useful:)
  LV_Volts = bytes[1] / 8;
  LV_Amps = bytes[4];
  Param::SetFloat(Param::U12V, LV_Volts);
  Param::SetFloat(Param::I12V, LV_Amps);
}

void MGgen2V2Lcharger::handle323(uint32_t data[2]) {

  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into
                       // bytes. See comments are useful:
  PlugStat = bytes[5];
  if (PlugStat == 1)
    PPStat = true; // plug inserted
  else
    PPStat = false; // plug not inserted
  Param::SetInt(Param::PlugDet, PPStat);
}

void MGgen2V2Lcharger::handle33B(uint32_t data[2]) {

  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into
                       // bytes. See comments are useful:
  temp_1 = bytes[3] - 40;
  Param::SetInt(Param::ChgTemp, temp_1);
}
