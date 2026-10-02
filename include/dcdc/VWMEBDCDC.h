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

#ifndef VWMEBDCDC_H
#define VWMEBDCDC_H

#include "dcdc/dcdc.h"

// VW MEB DC-DC converter 1EA907190x. It precharges the HV link by boosting
// from 12V, so no precharge resistor is used, then supplies 12V by bucking
// from HV. Message set from Battery-Emulator PR #3021: the converter is
// commanded by HVK_DCDC_Sollmodus in HVK_01 0x503 and its 12V output setpoint
// in NVEM_10 0x20F, and reports on DCDC_01 0x2AE, DCDC_03 0x5CD and
// DCDC_04 0xF7.
//
// Still to be confirmed on the bench: that the converter talks classic CAN
// (the VCU cannot receive CAN-FD frames), whether it needs KL15 in
// Klemmen_Status_01, and which message it takes the precharge target voltage
// from (BMS_01 0x191 is sent with udcsw as the target while precharging).
class VWMEBDCDC : public DCDC {
public:
  void DecodeCAN(int id, uint8_t *data) override;
  void DeInit() override;
  void Task10Ms() override;
  void Task100Ms() override;
  void SetCanInterface(CanHardware *c) override;
  bool DoesPrecharge() override { return true; }
  bool PrechargeDone() override;
  bool PrechargeFault() override { return loadFault; }

private:
  void SendNVEM10();
  void CheckLoad();
  bool CommsOk();

  uint8_t requestMode = 0;
  uint8_t actualMode = 0;
  bool prechargeComplete = false;
  float hvVoltage = 0;
  float hvCurrent = 0;
  uint8_t rxAge = 0xFF;       // 100ms ticks since the last DC-DC frame
  uint8_t contactorTicks = 0; // 100ms ticks the main contactor has been on
  bool loadFault = false;     // HV load seen during precharge, latched to OFF
  uint8_t loadTicks = 0;      // 100ms ticks of HV current in this window
  uint8_t quietTicks = 0;     // 100ms ticks without HV current
  float windowStartVoltage = 0;
  uint8_t counter = 0;
  uint8_t timer50Ms = 0;
};

#endif // VWMEBDCDC_H
