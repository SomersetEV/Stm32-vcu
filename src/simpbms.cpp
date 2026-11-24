/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2022 Charlie Smurthwaite
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

#include "simpbms.h"
#include "my_math.h"

/*
 * This module receives messages from SimpBMS and updates the
 * BMS_MinV, BMS_MaxV, BMS_MinT and BMS_MaxT parameters with the
 * received values. It also implements a timeout to indicate whether
 * the BMS is actively sending data or not. This data can be
 * used to safely stop any charging process if the BMS is not
 * working correctly.
 *
 * The charge current limit (CCL) is not taken from the BMS. It is derived
 * here from the min/max cell temperature using nominal NMC Li-ion charge
 * C-rates for a 156Ah cell. Voltage derating is left to udclim and the
 * BMS_VmaxLimit gate. The discharge limit (DCL) still comes from the BMS.
 *
 * Orion DTC screening: the Orion sends its DTC Flags #1/#2 on 0x35B and this
 * module re-sends them on 0x35A for the screen, with P0A06 (Charge Limit
 * Enforcement) and P0A08 (Charger Safety Relay) masked out. In charge mode, if
 * those are the only codes set, an OBD2 "clear DTCs" request is sent so they
 * do not block charging. Clearing wipes every code, so it is never sent while
 * any other fault is present.
 */

static const float CELL_CAPACITY_AH = 156.0f;

// OBD2 physical request ID of the Orion BMS 2 (factory default).
static const uint32_t ORION_OBD2_REQ_ID = 0x7E3;
// DTC Flags #1 (bytes 0-1) and #2 (bytes 2-3), little endian, from the Orion.
static const uint32_t ORION_DTC_IN_ID = 0x35B;
// The same message with the ignored codes removed, for the screen.
static const uint32_t ORION_DTC_OUT_ID = 0x35A;
static const uint16_t DTC1_P0A08 = 0x0002; // Charger Safety Relay Fault
static const uint16_t DTC2_P0A06 = 0x8000; // Charge Limit Enforcement Fault
// Minimum gap between clear requests, in 100ms ticks.
static const int DTC_CLEAR_INTERVAL = 50;

// Nominal continuous charge C-rate against cell temperature. Linearly
// interpolated between points, end values held beyond either end.
static const struct {
  float tempC;
  float cRate;
} chargeDerate[] = {
    {-20.0f, 0.0f}, {0.0f, 0.0f},  {5.0f, 0.1f},  {10.0f, 0.3f}, {15.0f, 0.5f},
    {25.0f, 1.0f},  {40.0f, 1.0f}, {45.0f, 0.5f}, {50.0f, 0.2f}, {55.0f, 0.0f},
};
static const int chargeDeratePoints =
    sizeof(chargeDerate) / sizeof(chargeDerate[0]);

// Return the allowed charge current in amps for a given cell temperature.
static float TempChargeLimit(float tempC) {
  if (tempC <= chargeDerate[0].tempC)
    return chargeDerate[0].cRate * CELL_CAPACITY_AH;

  for (int i = 1; i < chargeDeratePoints; i++) {
    if (tempC <= chargeDerate[i].tempC) {
      float t0 = chargeDerate[i - 1].tempC;
      float t1 = chargeDerate[i].tempC;
      float c0 = chargeDerate[i - 1].cRate;
      float c1 = chargeDerate[i].cRate;
      return (c0 + (c1 - c0) * (tempC - t0) / (t1 - t0)) * CELL_CAPACITY_AH;
    }
  }
  return chargeDerate[chargeDeratePoints - 1].cRate * CELL_CAPACITY_AH;
}

void SimpBMS::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x373);
  can->RegisterUserMessage(0x351);
  can->RegisterUserMessage(0x356);
  can->RegisterUserMessage(0x355);
  can->RegisterUserMessage(ORION_DTC_IN_ID);
}

bool SimpBMS::BMSDataValid() {
  // Return false if primary BMS is not sending data.
  if (timeoutCounter < 1)
    return false;
  return true;
}

// Return whether charging is currently permitted.
bool SimpBMS::ChargeAllowed() {
  // Refuse to charge if the BMS is not sending data.
  if (!BMSDataValid())
    return false;

  // Refuse to charge if the voltage or temperature is out of range.
  if (maxCellV > Param::GetFloat(Param::BMS_VmaxLimit))
    return false;
  if (minCellV < Param::GetFloat(Param::BMS_VminLimit))
    return false;
  if (maxTempC > Param::GetFloat(Param::BMS_TmaxLimit))
    return false;
  if (minTempC < Param::GetFloat(Param::BMS_TminLimit))
    return false;

  // Refuse to charge if the temperature derated limit is effectively zero.
  if (chargeCurrentLimit < 0.5f)
    return false;

  // Otherwise, charging is permitted.
  return true;
}

// Return the maximum charge current allowed by the BMS.
float SimpBMS::MaxChargeCurrent() {
  if (!ChargeAllowed())
    return 0;
  return chargeCurrentLimit;
}

// Send an OBD2 service 04 (clear DTCs) single frame request to the BMS.
void SimpBMS::ClearBmsFaults() {
  uint8_t bytes[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  can->Send(ORION_OBD2_REQ_ID, bytes, 8);
}

// Return the maximum discharge current allowed by the BMS.
float SimpBMS::MaxDischargeCurrent() {
  // Not gated on ChargeAllowed() - that gate is charge specific and the BMS
  // already applies its own derates to the discharge limit before sending it.
  // Zero simply means "no data" here, the caller then leaves the configured
  // limits alone rather than cutting drive power.
  if (!BMSDataValid())
    return 0;
  return dischargeCurrentLimit * 0.1;
}

// Process voltage and temperature message from SimpBMS.
void SimpBMS::DecodeCAN(int id, uint8_t *data) {
  if (id == 0x373) {
    int minCell = data[0] | (data[1] << 8);
    int maxCell = data[2] | (data[3] << 8);
    int minTemp = data[4] | (data[5] << 8);
    int maxTemp = data[6] | (data[7] << 8);

    minCellV = minCell / 1000.0;
    maxCellV = maxCell / 1000.0;
    minTempC = minTemp - 273;
    maxTempC = maxTemp - 273;

  } else if (id == 0x351) {
    // CCL in data[2..3] is ignored, it is derived from temperature instead.
    dischargeCurrentLimit = data[4] | (data[5] << 8); // DCL, 0.1A per digit
    // Reset timeout counter to the full timeout value.
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
  } else if (id == 0x356) {
    batteryVoltage = (data[0] | (data[1] << 8)) * 0.1; // comes in 0.1V scale

    int16_t rawCurrent = (int16_t)(data[2] | (data[3] << 8));
    // comes in 0.1A scale. Negated as the Orion reports current with the
    // opposite sign convention to the one idc uses.
    current = -rawCurrent * 0.1f;

  } else if (id == 0x355) {
    stateOfCharge = data[0] | (data[1] << 8); // comes in 1% scale
  } else if (id == (int)ORION_DTC_IN_ID) {
    // Only stored here, this runs in the CAN receive callback. Forwarded and
    // acted on in Task100Ms.
    dtcFlags1 = data[0] | (data[1] << 8);
    dtcFlags2 = data[2] | (data[3] << 8);
    dtcFresh = true;
  }
}

void SimpBMS::Task100Ms() {
  // Decrement timeout counter.
  if (timeoutCounter > 0)
    timeoutCounter--;

  if (dtcClearHold > 0)
    dtcClearHold--;

  if (dtcFresh) {
    dtcFresh = false;
    uint16_t raw1 = dtcFlags1;
    uint16_t raw2 = dtcFlags2;
    uint16_t out1 = raw1 & ~DTC1_P0A08;
    uint16_t out2 = raw2 & ~DTC2_P0A06;

    // Forward to the screen with the ignored codes removed.
    uint8_t bytes[8] = {(uint8_t)(out1 & 0xFF), (uint8_t)(out1 >> 8),
                        (uint8_t)(out2 & 0xFF), (uint8_t)(out2 >> 8),
                        0, 0, 0, 0};
    can->Send(ORION_DTC_OUT_ID, bytes, 4);

    // Clear only in charge mode, and only if the ignored codes are the only
    // ones set. In other modes they are just hidden from the screen.
    bool ignoredSet = (raw1 & DTC1_P0A08) || (raw2 & DTC2_P0A06);
    if (Param::GetInt(Param::opmode) == MOD_CHARGE && ignoredSet &&
        out1 == 0 && out2 == 0 && dtcClearHold == 0) {
      ClearBmsFaults();
      dtcClearHold = DTC_CLEAR_INTERVAL;
    }
  }

  // Derate the charge limit on temperature. The curve rises then falls, so its
  // lowest value across the pack's temperature spread is always at either the
  // min or the max cell temperature.
  chargeCurrentLimit =
      MIN(TempChargeLimit(minTempC), TempChargeLimit(maxTempC));

  // Update informational parameters.
  Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());
  Param::SetInt(Param::BMS_DischargeLim, MaxDischargeCurrent());

  if (BMSDataValid()) {
    Param::SetFloat(Param::BMS_Vmin, minCellV);
    Param::SetFloat(Param::BMS_Vmax, maxCellV);
    Param::SetFloat(Param::BMS_Tmin, minTempC);
    Param::SetFloat(Param::BMS_Tmax, maxTempC);

    // Apply the current limits to the drive current limits.
    // idcmax is the positive (discharge) limit and so takes the DCL from the
    // BMS, idcmin the negative (regen, which charges the pack) limit and so
    // takes the temperature derated CCL.
    // IdcLimitCommand is fed ABS(idc) and picks its branch from the sign of the
    // torque request, so this mapping holds regardless of the sign convention
    // used for idc itself. The DCL arrives at 0.1A per digit and needs scaling,
    // the CCL is already in amps. Only written while the BMS is actually
    // talking - on a timeout we do nothing here and the user configured limits
    // stay in force, so a CAN dropout cannot cut drive.
    Param::SetFloat(Param::idcmax, MIN(dischargeCurrentLimit * 0.1f, 5000.0f));
    Param::SetFloat(Param::idcmin, MAX(-chargeCurrentLimit, -5000.0f));
  } else {
    Param::SetFloat(Param::BMS_Vmin, 0);
    Param::SetFloat(Param::BMS_Vmax, 0);
    Param::SetFloat(Param::BMS_Tmin, 0);
    Param::SetFloat(Param::BMS_Tmax, 0);
  }

  if (Param::GetInt(Param::ShuntType) == 0) // No Shunt Used
  {
    Param::SetFloat(Param::udc2, batteryVoltage);
    Param::SetFloat(Param::udcsw, batteryVoltage - 30);

    if (BMSDataValid()) {
      Param::SetFloat(Param::idc, current);
      // Only safe to take SOC from the BMS here. With a shunt fitted
      // utils::CalcSOC() owns SOC and the two would fight each other.
      Param::SetFloat(Param::SOC, stateOfCharge);
    } else {
      Param::SetFloat(Param::idc, 0);
      Param::SetFloat(Param::udcsw, 1000);
    }
  }
}
