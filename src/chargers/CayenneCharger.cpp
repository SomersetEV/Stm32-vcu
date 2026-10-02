/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2023  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
 * 2024 Ben Bament - Somerset EV
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
 *
 *Control of the Cayenne OBC Charger. By Ben Bament
 *
 */

#include "chargers/CayenneCharger.h"
#include "errormessage.h"
#include "iomatrix.h"
#include "vag_crc.h"
#include "vag_hvk.h"

#define MAX_HV_CURRENT 32 // A, upper limit of the HV current request
#define MIN_VALID_HV 50   // V, below this the HV reading is not trusted
#define TAPER_VOLTS 10.0f // V below Voltspnt over which the current tapers to 0

bool CayenneCharger::ControlCharge(bool RunCh, bool ACReq) {
  switch (Param::GetInt(Param::interface)) {
  case Unused:
  case Chademo:
    // No AC charge interface, so start on the charger's own plug detection
    clearToStart = RunCh && HVLM_Plug_Status > 1;
    break;
  default:
    // i3LIM, CPC and Foccci report the AC charge request through ACReq
    clearToStart = RunCh && ACReq;
    break;
  }
  return clearToStart;
}

void CayenneCharger::Off() {
  clearToStart = false;
  HVEM_SollStrom_HV = 0;
  BMS_MaxCharge_Curr = 0;
}

void CayenneCharger::DeInit() {
  Off();
  stopcharge = 0;
}

void CayenneCharger::Task10Ms() {
  VagHvk::Request10Ms(can); // BMS_01   0x191
}

void CayenneCharger::Task100Ms()

{
  CalcValues100ms();
  VagHvk::Request100Ms(can); // Klemmen_Status_01 0x3C0, HVK_01 0x503
  msg1A1();                  // BMS_02   0x1A1
  msg184();                  // ZV_01    0x184
  msg17B();                  // FCU_02   0x17B
  msg39D();                  // BMS_03   0x39D
  msg552();                  // HVEM_05
}
void CayenneCharger::Task200Ms()

{
  msg583();
}
// messages with no CRC counter

void CayenneCharger::msg552() // HVEM_05
{
  uint8_t bytes[8];
  bytes[0] = HVEM_05[0];
  bytes[1] = HVEM_05[1];
  bytes[2] = HVEM_05[2];
  bytes[3] = HVEM_05[3];
  bytes[4] = HVEM_05[4];
  bytes[5] = HVEM_05[5];
  bytes[6] = HVEM_05[6];
  bytes[7] = HVEM_05[7];
  can->Send(0x552, (uint32_t *)bytes, 8);
}

void CayenneCharger::msg583() // ZV_02 200ms
{
  uint8_t bytes[8];
  bytes[0] = ZV_02[0];
  bytes[1] = ZV_02[1];
  bytes[2] = ZV_02[2];
  bytes[3] = ZV_02[3];
  bytes[4] = ZV_02[4];
  bytes[5] = ZV_02[5];
  bytes[6] = ZV_02[6];
  bytes[7] = ZV_02[7];
  can->Send(0x583, (uint32_t *)bytes, 8);
}

void CayenneCharger::msg17B() // FCU_02   0x17B
{
  uint8_t bytes[8];
  bytes[0] = FCU_02[0];
  bytes[1] = FCU_02[1];
  bytes[2] = FCU_02[2];
  bytes[3] = FCU_02[3];
  bytes[4] = FCU_02[4];
  bytes[5] = FCU_02[5];
  bytes[6] = FCU_02[6];
  bytes[7] = FCU_02[7];
  can->Send(0x17B, (uint32_t *)bytes, 8);
}

void CayenneCharger::msg184() // ZV_01   0x184
{
  uint8_t bytes[8];
  bytes[0] = 0x00;
  bytes[1] = (ZV_01[1] | vag_cnt184);
  bytes[2] = ZV_01[2];
  bytes[3] = ZV_01[3];
  bytes[4] = ZV_01[4];
  bytes[5] = ZV_01[5];
  bytes[6] = ZV_01[6];
  bytes[7] = ZV_01[7];
  VagCrc::Calc(bytes, 8, 0x184);
  can->Send(0x184, (uint32_t *)bytes, 8);
  vag_cnt184++;
  if (vag_cnt184 > 0x0f)
    vag_cnt184 = 0x00;
}

void CayenneCharger::msg1A1() // BMS_02   0x1A1
{
  uint8_t bytes[8];
  bytes[0] = BMS_02[0];
  bytes[1] = BMS_02[1];
  bytes[2] = BMS_02[2];
  bytes[3] = BMS_02[3];
  bytes[4] = BMS_02[4];
  bytes[5] = BMS_02[5];
  bytes[6] = BMS_02[6];
  bytes[7] = BMS_02[7];
  can->Send(0x1A1, (uint32_t *)bytes, 8);
}

void CayenneCharger::msg39D() // BMS_03   0x39D
{
  uint8_t bytes[8];
  bytes[0] = BMS_03[0];
  bytes[1] = BMS_03[1];
  bytes[2] = BMS_03[2];
  bytes[3] = BMS_03[3];
  bytes[4] = BMS_03[4];
  bytes[5] = BMS_03[5];
  bytes[6] = BMS_03[6];
  bytes[7] = BMS_03[7];
  can->Send(0x39D, (uint32_t *)bytes, 8);
}
void CayenneCharger::UnLockCP() {

  ZV_FT_entriegeln = 1;
  ZV_entriegeln_Anf = 1;
  FCU_TK_Freigabe_Tankklappe = 1;
  ZV_verriegelt_extern_ist = 0;
  ZV_verriegelt_soll = 1;
}

void CayenneCharger::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x488);
  can->RegisterUserMessage(0x53C);
  can->RegisterUserMessage(0x564);
  can->RegisterUserMessage(0x565);
  can->RegisterUserMessage(0x67E);
  can->RegisterUserMessage(0x415);
  can->RegisterUserMessage(0x12DD5472);
  can->RegisterUserMessage(0x1B000044);
}

void CayenneCharger::DecodeCAN(int id, uint32_t data[2]) {
  switch (id) {
  case 0x1B000044: // NMH_Ladegaeraet. Wake signal
    CayenneCharger::handle1B000044(data);
    break;
  case 0x488: // HVLM_06
    CayenneCharger::handle488(data);
    break;

  case 0x53C: // HVLM_04
    CayenneCharger::handle53C(data);
    break;

  case 0X564: // LAD_01
    CayenneCharger::handle564(data);
    break;

  case 0x565: // HVLM_03
    CayenneCharger::handle565(data);
    break;

  case 0x67E: // LAD_02
    CayenneCharger::handle67E(data);
    break;

  case 0x12DD5472: // HVLM_10
    CayenneCharger::handle12DD5472(data);
    break;

  case 0x415: // stop charge message
    CayenneCharger::handle415(data);
    break;
  }
}

void CayenneCharger::handle488(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
  HVLM_MaxDC_ChargePower =
      (((bytes[2] & (0x3FU)) << 4) | ((bytes[1] >> 4) & (0x0FU))) * 250;
  HVLM_Max_DC_Voltage_DCLS =
      ((bytes[3] & (0xFFU)) << 2) | ((bytes[2] >> 6) & (0x03U));
  HVLM_Actual_DC_Current_DCLS =
      ((bytes[5] & (0x01U)) << 8) | (bytes[4] & (0xFFU));
  HVLM_Max_DC_Current_DCLS =
      ((bytes[6] & (0x03U)) << 7) | ((bytes[5] >> 1) & (0x7FU));
  HVLM_Min_DC_Voltage_DCLS =
      ((bytes[7] & (0x07U)) << 6) | ((bytes[6] >> 2) & (0x3FU));
  HVLM_Min_DC_Current_DCLS = ((bytes[7] >> 3) & (0x1FU));
}

void CayenneCharger::handle53C(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
  // HVLM_ParkingHeater_Mode = ((HVLM_04[1] >> 4) & (0x07U));
  // HVLM_StationaryClimat_Timer_Stat = ((HVLM_04[1] >> 7) & (0x01U));
  // HVLM_HVEM_MaxPower = ((HVLM_04[3] & (0x01U)) << 8) | (HVLM_04[2] &
  // (0xFFU));
  HVLM_Status_Grid = ((bytes[3] >> 1) & (0x01U));
  // HVLM_BEV_LoadingScreen = ((HVLM_04[3] >> 2) & (0x01U));
  HVLM_EnergyFlowType = ((bytes[3] >> 3) & (0x03U));
  // HVLM_VK_ParkingHeaterStatus = ((HVLM_04[3] >> 5) & (0x07U));
  // HVLM_VK_ClimateConditioningStat = (HVLM_04[4] & (0x03U));
  HVLM_OperationalMode = ((bytes[4] >> 2) & (0x03U));
  HVLM_HV_ActivationRequest = ((bytes[4] >> 4) & (0x03U));
  HVLM_ChargerErrorStatus =
      ((bytes[5] & (0x01U)) << 2) | ((bytes[4] >> 6) & (0x03U));
  HVLM_Park_Request = ((bytes[5] >> 1) & (0x07U));
  HVLM_Park_Request_Maintain = ((bytes[5] >> 4) & (0x03U));
  // HVLM_AWC_Mode = (HVLM_04[6] & (0x07U));
  HVLM_Plug_Status = ((bytes[6] >> 3) & (0x03U));
  HVLM_LoadRequest = ((bytes[6] >> 5) & (0x07U));
  HVLM_MaxBattChargeCurrent = (bytes[7] & (0xFFU));
}

void CayenneCharger::handle564(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
  mode = ((bytes[1] >> 4) & (0x07U));
  ACvoltage = ((bytes[2] & (0xFFU)) << 1) | ((bytes[1] >> 7) & (0x01U));
  Param::SetFloat(Param::AC_Volts, ACvoltage);
  HVVoltage = (((bytes[4] & (0x03U)) << 8) | (bytes[3] & (0xFFU)));
  hvCurrent =
      ((((bytes[5] & (0x0FU)) << 6) | ((bytes[4] >> 2) & (0x3FU))) * 0.2f) -
      102;
  current = (int16_t)hvCurrent;
  LAD_Status_Voltage = ((bytes[5] >> 4) & (0x03U));
  temperature = bytes[6] - 40;
  Param::SetFloat(Param::ChgTemp, temperature);
  LAD_PowerLossVal = ((bytes[7] & (0xFFU))) * 20;

  if (Param::GetInt(Param::ShuntType) == 0 &&
      Param::GetInt(Param::Inverter) != InvModes::Leaf_Gen1) {
    // Only backfill HV voltage/current from the charger when no dedicated
    // shunt is configured and the Leaf inverter is not already providing it.
    Param::SetFloat(Param::udc, HVVoltage);
    Param::SetFloat(Param::idc, hvCurrent > 0 ? hvCurrent : 0);
  }
}

void CayenneCharger::handle565(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
  HVLM_HV_StaleTime = ((bytes[0] & (0xFFU))) * 4;
  HVLM_ChargeSystemState = (bytes[1] & (0x03U));
  // HVLM_KESSY_KeySearch = ((HVLM_03[1] >> 2) & (0x03U));
  HVLM_Status_LED = ((bytes[1] >> 4) & (0x0FU));
  MaxACAmps = ((bytes[3] & (0x7FU))) / 2; // Max amps from EVSE
  HVLM_LG_ChargerTargetMode = ((bytes[3] >> 7) & (0x01U));
  HVLM_TankCapReleaseRequest = (bytes[4] & (0x03U));
  HVLM_RequestConnectorLock = ((bytes[4] >> 2) & (0x03U));
  HVLM_Start_VoltageMeasure_DCLS = ((bytes[4] >> 4) & (0x03U));
  // PnC_Trigger_OBC_cGW = ((HVLM_03[5] & (0x03U)) << 2) | ((HVLM_03[4] >> 6) &
  // (0x03U)); HVLM_ReleaseAirConditioning = ((HVLM_03[5] >> 2) & (0x03U));
  HVLM_ChargeReadyStatus = ((bytes[6] >> 1) & (0x07U));
  // HVLM_IsolationRequest = ((HVLM_03[6] >> 5) & (0x01U));
  HVLM_Output_Voltage_HV =
      ((bytes[7] & (0xFFU)) << 2) | ((bytes[6] >> 6) & (0x03U));
}

void CayenneCharger::handle67E(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
  LAD_Reduction_ChargerTemp = ((bytes[1] >> 4) & (0x01U));
  LAD_Reduction_Current = ((bytes[1] >> 5) & (0x01U));
  LAD_Reduction_SocketTemp = ((bytes[1] >> 6) & (0x01U));
  LAD_MaxChargerPower_HV =
      (((bytes[3] & (0x01U)) << 8) | (bytes[2] & (0xFFU))) * 100;
  PPLim = (bytes[4] & (0x07U));
  LAD_ControlPilotStatus = ((bytes[4] >> 3) & (0x01U));
  LAD_LockFeedback = ((bytes[4] >> 4) & (0x01U));
  LAD_ChargerCoolingDemand = ((bytes[4] >> 6) & (0x03U));
  // LAD_MaxLadLeistung_HV_Offset = ((LAD_02[7] >> 1) & (0x03U));
  LAD_ChargerWarning = ((bytes[7] >> 6) & (0x01U));
  LAD_ChargerFault = ((bytes[7] >> 7) & (0x01U));
}

void CayenneCharger::handle12DD5472(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
  HVLM_RtmWarnLadeverbindung =
      ((bytes[4] & (0x03U)) << 1) | ((bytes[3] >> 7) & (0x01U));
  HVLM_RtmWarnLadesystem = ((bytes[4] >> 2) & (0x07U));
  HVLM_RtmWarnLadestatus = ((bytes[4] >> 5) & (0x07U));
  HVLM_RtmWarnLadeKommunikation = (bytes[5] & (0x07U));
}

void CayenneCharger::handle1B000044(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
  carwakeup = bytes[6];
}

void CayenneCharger::handle415(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
  stopcharge = bytes[0];
}

void CayenneCharger::CalcValues100ms() // Run to calculate values every 100 ms
{
  // Use the measured HV voltage, falling back to the charger's own reading
  float hvVolts = Param::GetFloat(Param::udc);
  if (hvVolts < MIN_VALID_HV)
    hvVolts = HVVoltage;
  int voltSetpoint = Param::GetInt(Param::Voltspnt);

  // Charge current limit, as for the Elcon charger: the lower of the power
  // setpoint and the BMS current limit, clamped to MAX_HV_CURRENT
  float limitAmps = 0;
  if (hvVolts >= MIN_VALID_HV) {
    float power = MIN(Param::GetFloat(Param::Pwrspnt),
                      Param::GetFloat(Param::BMS_ChargeLim) * hvVolts);
    limitAmps = MIN(power / hvVolts, MAX_HV_CURRENT);
  }

  // Taper the limit linearly to 0 over the last TAPER_VOLTS below the voltage
  // setpoint, so the current settles where the battery sits just under
  // Voltspnt instead of stepping up and down around it
  float headroom = voltSetpoint - hvVolts;
  int targetAmps = limitAmps + 0.5f;
  if (headroom < TAPER_VOLTS) {
    targetAmps = limitAmps * MAX(headroom, 0.0f) / TAPER_VOLTS + 0.5f;
    // Keep 1A until Voltspnt is reached so the VCU's end of charge check
    // (udc >= Voltspnt and idc <= IdcTerm) can trigger
    if (headroom > 0 && limitAmps >= 1)
      targetAmps = MAX(targetAmps, 1);
  }

  if (!clearToStart || stopcharge == 1)
    targetAmps = 0;

  // stop charging
  if (stopcharge == 1)
    UnLockCP();

  // Ramp the HV current request towards the target, 1A per 100ms
  if (HVEM_SollStrom_HV > targetAmps)
    HVEM_SollStrom_HV--;
  else if (HVEM_SollStrom_HV < targetAmps)
    HVEM_SollStrom_HV++;

  BMS_MaxCharge_Curr = targetAmps;
  HVEM_MaxSpannung_HV = voltSetpoint;
  BMS_Batt_Max_Volt = voltSetpoint;

  // Runtime Values:
  VagHvk::bms01.BMS_Batt_Curr = (current + 2047);

  // BMS Limits Discharge:
  BMS_MaxDischarge_Curr = 1500;
  BMS_Min_Batt_Volt = 0;
  BMS_Min_Batt_Volt_Discharge = 0;
  BMS_MaxCharge_Curr_Offset = 0;
  BMS_Min_Batt_Volt_Charge = 0;
  BMS_OpenCircuit_Volts = 0;

  HVEM_Nachladen_Anf = false; // Request for HV charging with plugged in
                              // connector and deactivated charging request

  // The charger only charges while it sees the HV system up and the BMS in AC
  // charge mode. Earlier versions always sent these values (the if statements
  // that were meant to switch them used = instead of ==), so they are kept
  // constant here. HVK_DCDC_Sollmodus is left to the DC-DC converter module.
  VagHvk::hvk01.HV_Bordnetz_aktiv = true;
  VagHvk::hvk01.BMS_Sollmodus = 4;        // AC_Charging
  VagHvk::hvk01.MO_EmSollzustand = 50;    // HvAcCh
  VagHvk::hvk01.Gesamtst_Spgfreiheit = 2; // Voltage
  VagHvk::hvk01.HVLM_Sollmodus = clearToStart;
  VagHvk::bms01.packVoltage = 400;
  VagHvk::bms01.linkVoltage = 400;
  if (HVVoltage > 250) {
    VagHvk::bms01.packVoltage = HVVoltage;
    VagHvk::bms01.linkVoltage = HVVoltage;
  }

  // Plug detection and EVSE current limit, when no charge interface or PP
  // input reports them
  if (Param::GetInt(Param::interface) == Unused) {
    bool ppInput = Param::GetInt(Param::GPA1Func) == IOMatrix::PILOT_PROX ||
                   Param::GetInt(Param::GPA2Func) == IOMatrix::PILOT_PROX;
    if (!ppInput)
      Param::SetInt(Param::PlugDet, HVLM_Plug_Status > 1);
    Param::SetInt(Param::CableLim, MaxACAmps);
  }

  // Charger faults (1=DC-NotOK, 2=AC-NotOK, 3=Interlock) and warnings
  if (LAD_ChargerFault ||
      (HVLM_ChargerErrorStatus >= 1 && HVLM_ChargerErrorStatus <= 3))
    ErrorMessage::Post(ERR_CHGFAULT);
  if (LAD_ChargerWarning)
    ErrorMessage::Post(ERR_CHGWARN);

  //  BMS_02
  BMS_02[0] = 0x00;
  BMS_02[1] = (BMS_MaxCharge_Curr_Offset & (0x0FU)) |
              ((BMS_MaxDischarge_Curr & (0x0FU)) << 4);
  BMS_02[2] = ((BMS_MaxDischarge_Curr >> 4) & (0x7FU)) |
              ((BMS_MaxCharge_Curr & (0x01U)) << 7);
  BMS_02[3] = ((BMS_MaxCharge_Curr >> 1) & (0xFFU));
  BMS_02[4] = ((BMS_MaxCharge_Curr >> 9) & (0x03U)) |
              ((BMS_Min_Batt_Volt & (0x3FU)) << 2);
  BMS_02[5] = ((BMS_Min_Batt_Volt >> 6) & (0x0FU)) |
              ((BMS_Min_Batt_Volt_Discharge & (0x0FU)) << 4);
  BMS_02[6] = ((BMS_Min_Batt_Volt_Discharge >> 4) & (0x3FU)) |
              ((BMS_Min_Batt_Volt_Charge & (0x03U)) << 6);
  BMS_02[7] = ((BMS_Min_Batt_Volt_Charge >> 2) & (0xFFU));

  //  BMS_03
  BMS_03[0] = (BMS_OpenCircuit_Volts & (0xFFU));
  BMS_03[1] = ((BMS_OpenCircuit_Volts >> 8) & (0x03U)) |
              ((BMS_Batt_Max_Volt & (0x0FU)) << 4);
  BMS_03[2] = ((BMS_Batt_Max_Volt >> 4) & (0x3FU)) |
              ((BMS_MaxDischarge_Curr & (0x03U)) << 6);
  BMS_03[3] = ((BMS_MaxDischarge_Curr >> 2) & (0xFFU));
  BMS_03[4] = ((BMS_MaxDischarge_Curr >> 10) & (0x01U)) |
              ((BMS_MaxCharge_Curr & (0x7FU)) << 1);
  BMS_03[5] = ((BMS_MaxCharge_Curr >> 7) & (0x0FU)) |
              ((BMS_Min_Batt_Volt_Discharge & (0x0FU)) << 4);
  BMS_03[6] = ((BMS_Min_Batt_Volt_Discharge >> 4) & (0x3FU)) |
              ((BMS_Min_Batt_Volt_Charge & (0x03U)) << 6);
  BMS_03[7] = ((BMS_Min_Batt_Volt_Charge >> 2) & (0xFFU));

  //  HVEM_05
  //  HVEM_Nachladen_Anf - Request for HV charging with plugged in connector and
  //  deactivated charging request HVEM_SollStrom_HV - Target current charging
  //  on the HV side HVEM_MaxSpannung_HV - Maximum charging voltage to the
  //  charger or DC charging station HVEM_Abschaltstatus - Climate Reduction -
  //  Set to 0 for 100% (No Reduction)
  HVEM_05[1] = 0x00;
  HVEM_05[2] = 0x00;
  HVEM_05[3] = 0x00;
  HVEM_05[4] =
      (HVEM_Nachladen_Anf & (0x01U)) | ((HVEM_SollStrom_HV & (0x7FU)) << 1);
  HVEM_05[5] = ((HVEM_SollStrom_HV >> 7) & (0x0FU)) |
               ((HVEM_MaxSpannung_HV & (0x0FU)) << 4);
  HVEM_05[6] = ((HVEM_MaxSpannung_HV >> 4) & (0x3FU)) | ((0x00 & (0x03U)) << 6);
  HVEM_05[7] = 0x00;

  ZV_01[1] = (0x00 & (0x0FU)) | ((ZV_FT_verriegeln & (0x01U)) << 4) |
             ((ZV_FT_entriegeln & (0x01U)) << 5) |
             ((ZV_BT_verriegeln & (0x01U)) << 6) |
             ((ZV_BT_entriegeln & (0x01U)) << 7);
  ZV_01[7] = ((0x00 >> 5) & (0x3FU)) | ((ZV_entriegeln_Anf & (0x01U)) << 6) |
             ((0x00 & (0x01U)) << 7);

  ZV_02[2] = (ZV_verriegelt_intern_ist & (0x01U)) |
             ((ZV_verriegelt_extern_ist & (0x01U)) << 1) |
             ((ZV_verriegelt_intern_soll & (0x01U)) << 2) |
             ((ZV_verriegelt_extern_soll & (0x01U)) << 3) |
             ((0x00 & (0x01U)) << 4) | ((0x00 & (0x01U)) << 5) |
             ((0x00 & (0x01U)) << 6) | ((0x00 & (0x01U)) << 7);
  ZV_02[7] = (ZV_Tankklappe_offen & (0x01U)) | ((ZV_Rollo_auf & (0x01U)) << 1) |
             ((ZV_Rollo_zu & (0x01U)) << 2) | ((ZV_SAD_auf & (0x01U)) << 3) |
             ((ZV_SAD_zu & (0x01U)) << 4) |
             ((BCM_Tankklappensteller_Fehler & (0x01U)) << 5) |
             ((ZV_verriegelt_soll & (0x03U)) << 6);

  FCU_02[5] = (0x00 & (0x01U)) | ((0x00 & (0x03U)) << 1) |
              ((FCU_TK_Betankung_Anforderung & (0x01U)) << 3) |
              ((0x00 & (0x0FU)) << 4);
  FCU_02[7] = (FCU_TK_Freigabe_Tankklappe & (0x03U));
}
