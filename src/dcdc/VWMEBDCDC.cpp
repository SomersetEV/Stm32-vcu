/*
 * This file is part of the ZombieVerter project.
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

#include "dcdc/VWMEBDCDC.h"
#include "digio.h"
#include "my_math.h"
#include "params.h"
#include "vag_crc.h"
#include "vag_hvk.h"

namespace {
// HVK_DCDC_Sollmodus and the mode reported in DCDC_04
constexpr uint8_t MODE_STANDBY = 0;
constexpr uint8_t MODE_PRECHARGE = 1;
constexpr uint8_t MODE_BUCK = 2;

constexpr uint8_t COMMS_TIMEOUT = 10;   // 100ms ticks without DC-DC frames
constexpr uint8_t CONTACTOR_SETTLE = 5; // 100ms ticks of main contactor on
                                        // before leaving precharge mode
constexpr float PRECHARGE_MATCH = 10;   // V, link to udcsw window
constexpr float MIN_VALID_HV = 50;      // V, below this HV is not trusted

// Load detection during precharge. Charging the link capacitance always
// raises the voltage, so HV current flowing while the voltage does not rise
// means something on the link is drawing it.
constexpr float LOAD_CURRENT = 1.0f; // A, HV current treated as flowing
constexpr uint8_t LOAD_WINDOW = 10;  // 100ms ticks per voltage rise check
constexpr float LOAD_MIN_RISE = 5;   // V, minimum rise per window with current
constexpr uint8_t QUIET_TICKS = 5;   // 100ms ticks under LOAD_CURRENT before
                                     // precharge counts as done

// NVEM_10 12V setpoint: 10.6V + 25mV per bit, up to 16.0V
constexpr int SETPOINT_MIN_MV = 10600;
constexpr int SETPOINT_STEP_MV = 25;
constexpr int SETPOINT_MAX_RAW = 216;
} // namespace

void VWMEBDCDC::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x2AE); // DCDC_01
  can->RegisterUserMessage(0x5CD); // DCDC_03
  can->RegisterUserMessage(0xF7);  // DCDC_04
}

void VWMEBDCDC::DeInit() {
  requestMode = MODE_STANDBY;
  actualMode = MODE_STANDBY;
  prechargeComplete = false;
  hvVoltage = 0;
  hvCurrent = 0;
  rxAge = 0xFF;
  contactorTicks = 0;
  loadFault = false;
  loadTicks = 0;
  quietTicks = 0;
  counter = 0;
  timer50Ms = 0;
  // Hand HVK_01 back to its defaults for the charger
  VagHvk::hvk01.DCDC_Sollmodus = VagHvk::Hvk01().DCDC_Sollmodus;
  VagHvk::hvk01.Vorladung_aktiv = false;
}

void VWMEBDCDC::Task10Ms() {
  timer50Ms++;
  if (timer50Ms >= 5) {
    SendNVEM10();
    timer50Ms = 0;
  }
  // BMS_01 carries the pack voltage the link has to be brought up to
  if (requestMode != MODE_STANDBY)
    VagHvk::Request10Ms(can);
}

void VWMEBDCDC::Task100Ms() {
  if (rxAge < 0xFF)
    rxAge++;

  switch (Param::GetInt(Param::opmode)) {
  case MOD_PRECHARGE:
    requestMode = MODE_PRECHARGE;
    contactorTicks = 0;
    break;
  case MOD_RUN:
  case MOD_CHARGE:
  case MOD_MAINTAIN:
  case MOD_PREHEAT:
    // Keep holding the link up until the main contactor has closed, then
    // supply 12V
    if (!DigIo::dcsw_out.Get())
      contactorTicks = 0;
    else if (contactorTicks < CONTACTOR_SETTLE)
      contactorTicks++;
    requestMode =
        contactorTicks >= CONTACTOR_SETTLE ? MODE_BUCK : MODE_PRECHARGE;
    break;
  default:
    requestMode = MODE_STANDBY;
    contactorTicks = 0;
    break;
  }

  CheckLoad();

  if (CommsOk() && hvVoltage > MIN_VALID_HV) {
    VagHvk::bms01.linkVoltage = hvVoltage;
    // With the main contactor closed the link is at pack voltage
    if (requestMode == MODE_BUCK)
      VagHvk::bms01.packVoltage = hvVoltage;
  }

  VagHvk::hvk01.DCDC_Sollmodus = requestMode;
  VagHvk::hvk01.Vorladung_aktiv = requestMode == MODE_PRECHARGE;
  VagHvk::Request100Ms(can);
}

void VWMEBDCDC::CheckLoad() {
  int opmode = Param::GetInt(Param::opmode);

  if (opmode == MOD_OFF) {
    loadFault = false; // cleared for the next precharge attempt
    loadTicks = 0;
    quietTicks = 0;
    return;
  }
  // Only the precharge itself is checked: once the main contactor closes the
  // pack supplies the loads
  if (opmode != MOD_PRECHARGE || !CommsOk()) {
    loadTicks = 0;
    quietTicks = 0;
    return;
  }

  if (ABS(hvCurrent) > LOAD_CURRENT) {
    quietTicks = 0;
    if (loadTicks == 0)
      windowStartVoltage = hvVoltage;
    loadTicks++;
    if (loadTicks >= LOAD_WINDOW) {
      if (hvVoltage - windowStartVoltage < LOAD_MIN_RISE)
        loadFault = true;
      loadTicks = 0;
    }
  } else {
    loadTicks = 0;
    if (quietTicks < QUIET_TICKS)
      quietTicks++;
  }
}

bool VWMEBDCDC::PrechargeDone() {
  if (requestMode != MODE_PRECHARGE || !CommsOk() || loadFault ||
      quietTicks < QUIET_TICKS)
    return false;
  return prechargeComplete ||
         ABS(hvVoltage - VagHvk::PackVoltage()) < PRECHARGE_MATCH;
}

bool VWMEBDCDC::CommsOk() { return rxAge < COMMS_TIMEOUT; }

void VWMEBDCDC::SendNVEM10() {
  uint8_t bytes[8] = {0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0xFD, 0x00};
  int setpoint = 0; // 10.6V floor while not supplying 12V

  if (requestMode == MODE_BUCK) {
    setpoint = (Param::GetFloat(Param::DCSetPnt) * 1000 - SETPOINT_MIN_MV) /
               SETPOINT_STEP_MV;
    setpoint = MIN(MAX(setpoint, 0), SETPOINT_MAX_RAW);
  }

  bytes[1] = counter;
  bytes[7] = setpoint;
  VagCrc::Calc(bytes, 8, 0x20F);
  can->Send(0x20F, bytes, 8);
  counter = (counter + 1) & 0x0F;
}

void VWMEBDCDC::DecodeCAN(int id, uint8_t *data) {
  switch (id) {
  case 0x2AE: // DCDC_01
    rxAge = 0;
    hvVoltage = (((uint16_t)data[2] << 4) | (data[1] >> 4)) * 0.25f;
    hvCurrent =
        ((int)((((uint16_t)data[4] & 0x03) << 8) | data[3]) - 511) * 0.1f;
    Param::SetFloat(Param::DCDCUhv, hvVoltage);
    Param::SetFloat(Param::DCDCIhv, hvCurrent);
    Param::SetFloat(Param::U12V, data[7] * 0.1f);
    Param::SetFloat(Param::I12V,
                    (float)((((data[5] & 0x0F) << 6) | (data[4] >> 2)) - 511));
    if (Param::GetInt(Param::ShuntType) == 0 &&
        Param::GetInt(Param::BMS_Mode) != BMSSTW &&
        Param::GetInt(Param::Inverter) != InvModes::Leaf_Gen1) {
      // No dedicated shunt, so precharge on the converter's link voltage
      Param::SetFloat(Param::udc, hvVoltage);
    }
    break;
  case 0x5CD: // DCDC_03
    rxAge = 0;
    Param::SetFloat(Param::DCDCTemp, data[7] - 40);
    break;
  case 0xF7: // DCDC_04
    rxAge = 0;
    actualMode = (data[5] >> 1) & 0x07;
    prechargeComplete = (data[4] >> 2) & 0x01;
    Param::SetInt(Param::DCDCMode, actualMode);
    break;
  default:
    break;
  }
}
