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

/* Control of the MG Gen 2 V2L charger. */

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
uint8_t MGgen2V2Lcharger::v2lCounter = 0;    // rolling 4-bit counter for 0x08E
uint8_t MGgen2V2Lcharger::v2lHeartbeat = 0;  // free-running counter for 0x099
static uint8_t PlugStat = 0;
static bool PPStat = false;

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
  // can->RegisterUserMessage(0x326);//test
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
    // case 0x38A:
    //    MGgen2V2Lcharger::handle38A(data);
    //    break;
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
  if (opmode == MOD_RUN) // V2L and DC-DC
  {
    if (V2Ltimer < 200)
      V2Ltimer++;

    // --- 0x19C: DC-DC keepalive ---
    // D5=0x7F, D6=0xFF, D8=0x7F confirmed from working V2L log.
    if (dcDcTimer < 23)
      dcDcTimer++;
    bytes[0] = 0x06;
    bytes[1] = 0xA0;
    bytes[2] = (dcDcTimer >= 23) ? 0x26 : 0x06; // 0x26 enables DC-DC, 0x06 holds off
    bytes[3] = 0xA0;
    bytes[4] = 0x7F;
    bytes[5] = 0xFF;
    bytes[6] = dcDcCounter & 0x0F;
    bytes[7] = 0x7F;
    dcDcCounter++;
    can->Send(0x19C, (uint32_t *)bytes, 8);

    // --- 0x08E: V2L enable keepalive ---
    // D1..D6 constant across all 3091 frames of the MG ZS V2L log.
    // D7 = rolling 4-bit counter, D8 = XOR(D1..D7).
    bytes[0] = 0x80;
    bytes[1] = 0x04;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x13;
    bytes[5] = 0x88;
    bytes[6] = v2lCounter & 0x0F;
    bytes[7] = bytes[0]^bytes[1]^bytes[2]^bytes[3]^bytes[4]^bytes[5]^bytes[6];
    v2lCounter++;
    can->Send(0x08E, (uint32_t *)bytes, 8);

    // --- V2L phase sequencing (V2Ltimer ticks at 100ms) ---
    // Mirrors the MG ZS log ordering, compressed. Tune the boundaries below.
    //   A  0-19   D2=0x01 standby      33F=0x00   29C=idle
    //   B 20-24   D2=0x21 V2L REQUEST  33F=0x06   29C=cleared
    //   C 25-29   D2=0x01              33F=0x46
    //   D 30-34   D2=0x04              33F=0x46
    //   E 35-39   D2=0x0C              33F=0x46
    //   F 40+     D2=0x03 running      33F=0x48   29C=V2L active
    uint8_t v2lState;   // 0x297 D2
    uint8_t v2lD7hi;    // 0x297 D7 high nibble (state-linked in the real car)
    if (V2Ltimer < 20)      { v2lState = 0x01; v2lD7hi = 0x90; }
    else if (V2Ltimer < 25) { v2lState = 0x21; v2lD7hi = 0x80; }
    else if (V2Ltimer < 30) { v2lState = 0x01; v2lD7hi = 0x90; }
    else if (V2Ltimer < 35) { v2lState = 0x04; v2lD7hi = 0x80; }
    else if (V2Ltimer < 40) { v2lState = 0x0C; v2lD7hi = 0x80; }
    else                    { v2lState = 0x03; v2lD7hi = 0xB0; }

    // --- 0x297: BMS state — charger needs BOTH frames ---
    // Null frame: D2 mirrors the state, everything else zero, D8 = 0x00.
    // Confirmed: null frames never carry the XOR checksum.
    bytes[0] = 0x00;
    bytes[1] = v2lState;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = (v2lState == 0x03) ? 0x20 : 0x00;
    bytes[7] = 0x00; // NOT the XOR — real null frames are always 0x00 here
    can->Send(0x297, (uint32_t *)bytes, 8);

    // Full frame: D3=0xE0 data-valid, D5/D6=0xFF/0xFF in every state,
    // D7 = state-linked high nibble + rolling counter, D8 = XOR(D1..D7).
    bytes[0] = 0x01;
    bytes[1] = v2lState;
    bytes[2] = 0xE0;
    bytes[3] = 0x00;
    bytes[4] = 0xFF;
    bytes[5] = 0xFF;
    bytes[6] = v2lD7hi | (v2lHeartbeat & 0x0F);
    bytes[7] = bytes[0]^bytes[1]^bytes[2]^bytes[3]^bytes[4]^bytes[5]^bytes[6];
    v2lHeartbeat++;
    can->Send(0x297, (uint32_t *)bytes, 8);

    // --- 0x29C: discharge limits — steps in sync with 0x33F ---
    bytes[0] = 0x28;
    if (V2Ltimer < 20) {          // idle
      bytes[1] = 0xFF; bytes[2] = 0x83; bytes[3] = 0xFF;
      bytes[4] = 0x00; bytes[5] = 0xFF; bytes[6] = 0x7F; bytes[7] = 0xFF;
    } else if (V2Ltimer < 40) {   // cleared during arming
      bytes[1] = 0x00; bytes[2] = 0x00; bytes[3] = 0x00;
      bytes[4] = 0x00; bytes[5] = 0x00; bytes[6] = 0x00; bytes[7] = 0x00;
    } else {                      // V2L active: 22.0A, 445.8V
      bytes[1] = 0x89; bytes[2] = 0x04; bytes[3] = 0x00;
      bytes[4] = 0x00; bytes[5] = 0xDC; bytes[6] = 0x57; bytes[7] = 0x12;
    }
    can->Send(0x29C, (uint32_t *)bytes, 8);

    // --- 0x1F1: wakeup keepalive ---
    bytes[0] = 0x0E;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x20;
    bytes[7] = 0x00;
    can->Send(0x1F1, (uint32_t *)bytes, 8);

    // --- 0x33F: V2L output relay sequence ---
    // 0x00 idle -> 0x06 -> 0x46 announce -> 0x48 output enable.
    // The 0x06 step appears in the MG ZS log during the 0x297 D2=0x21
    // request window; 0x46 follows immediately after that window closes.
    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x28;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    if (V2Ltimer < 22)
      bytes[7] = 0x00;
    else if (V2Ltimer < 25)
      bytes[7] = 0x06;
    else if (V2Ltimer < 40)
      bytes[7] = 0x46;
    else
      bytes[7] = 0x48;
    can->Send(0x33F, (uint32_t *)bytes, 8);

    // --- 0x396: Car_2_CCU — direct car/HCU to charger authorisation ---
    // DBC confirms this is a Car_2_CCU message (car sends to charger, not BMS).
    // D5 was 0xC0, but the MG ZS V2L log only ever shows 0x00 or 0x80 here,
    // so 0xC0 is not a value the charger expects. Using 0x00.
    bytes[0] = 0x44;
    bytes[1] = 0x6E;
    bytes[2] = 0xB4;
    bytes[3] = 0x28;
    bytes[4] = 0x00;
    bytes[5] = 0x4E;
    bytes[6] = 0x4D;
    bytes[7] = 0x4D;
    can->Send(0x396, (uint32_t *)bytes, 8);
  }

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
    bytes[7] = bytes[0]^bytes[1]^bytes[2]^bytes[3]^bytes[4]^bytes[5]^bytes[6];
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
    bytes[7] = bytes[0]^bytes[1]^bytes[2]^bytes[3]^bytes[4]^bytes[5]^bytes[6];
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

  V2Ltimer = 0;  // reset V2L timer
  dcDcTimer = 0; // reset DC-DC startup gate — forces re-sequencing on next RUN
  dcDcCounter = 0; // reset rolling counter
  v2lCounter = 0;    // reset 0x08E counter
  v2lHeartbeat = 0;  // reset 0x099 counter
  uint8_t bytes[8];
  bytes[0] = 0x46; // TRUE off state
  bytes[1] = 0xA0;
  bytes[2] = 0x06; // DC-DC off
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00; // real MG5 sends 0x00 here during true off (was 0x7F)
  can->Send(0x19C, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x01;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00; // 20 to wake up charger.
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

  bytes[0] = 0x00; // 0E to wake up
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x20;
  bytes[7] = 0x00;
  can->Send(0x1F1, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x28;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;                        // idle — was incorrectly 0x46 (V2L announce)
  can->Send(0x33F, (uint32_t *)bytes, 8); // V2L

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x20;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x322, (uint32_t *)bytes, 8);

  bytes[0] = 0x44;
  bytes[1] = 0x43;
  bytes[2] = 0x9D;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x39B, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x43;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0xCD;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x39A, (uint32_t *)bytes, 8);

  bytes[0] = 0x44;
  bytes[1] = 0x6E;
  bytes[2] = 0xB4;
  bytes[3] = 0x28;
  bytes[4] = 0x80;
  bytes[5] = 0x4E;
  bytes[6] = 0x4E;
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

  bytes[0] = 0x3B;
  bytes[1] = 0xCA;
  bytes[2] = 0x86;
  bytes[3] = 0x00;
  bytes[4] = 0xFD;
  bytes[5] = 0x60;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x29B, (uint32_t *)bytes, 8);
}

void MGgen2V2Lcharger::handle324(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into
                       // bytes. See comments are useful:)
  batteryVolts = ((bytes[1] << 8) | (bytes[2])) * 0.02;
  ;

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
