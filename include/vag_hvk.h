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
 *
 * Vehicle side "HV coordinator" frames that several VAG modules on the hybrid
 * CAN bus listen to (Cayenne charger, MEB DC-DC). Each module writes the
 * fields it owns and requests the frames on its CAN interface; Task10Ms() and
 * Task100Ms() then send each requested frame once per CAN interface, so two
 * modules on the same bus never put duplicate 0x503 / 0x3C0 / 0x191 frames on
 * it.
 */

#ifndef VAG_HVK_H
#define VAG_HVK_H

#include "canhardware.h"
#include <stdint.h>

namespace VagHvk {

// HVK_01 0x503
struct Hvk01 {
  // 0 "HvOff" 1 "HvStbyReq" 2 "HvStbyWait" 3 "HvBattOnReq" 4 "HvBattOnWait"
  // 10 "HvOnIdle" 20 "HvOnDrvRdy" 46 "HvAcChPreReq" 47 "HvAcChPreWait"
  // 48 "HvAcChReq" 49 "HvAcChWait" 50 "HvAcCh" 56 "HvDcChPreReq"
  // 57 "HvDcChPreWait" 58 "HvDcChReq" 59 "HvDcChWait" 60 "HvDcCh"
  // 67 "HvChOffReq" 68 "HvChOffWait" 69 "HvOnIdleReq" 70 "HvOnIdleWait"
  // 96 "HvCpntOffReq" 97 "HvCpntOffWait" 98 "HvBattOffReq" 99 "HvBattOffWait"
  // 119 "HvElmOffReq" 120 "HvElmOff"
  uint8_t MO_EmSollzustand = 50;
  // 0 "HV_Off" 1 "HV_On" 3 "AC_Charging_ext" 4 "AC_Charging" 6 "DC_Charging"
  // 7 "Init"
  uint8_t BMS_Sollmodus = 4;
  // 0 "Standby" 1 "HV_On_Precharging" 2 "Step down" 3 "Step up"
  // 4 "Test pulse_12V" 7 "Initialization"
  uint8_t DCDC_Sollmodus = 2;
  // Requested target mode of the charging manager: 0=Not Enabled, 1=Enabled
  uint8_t HVLM_Sollmodus = 0;
  // Active high-voltage vehicle electrical system: 0=Not Active, 1=Active
  bool HV_Bordnetz_aktiv = true;
  // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
  uint8_t Gesamtst_Spgfreiheit = 2;
  // HVK_Vorladung_aktiv, set while a DC-DC converter precharges the HV link.
  // While set, BMS_Sollmodus and HV_Bordnetz_aktiv are sent as HV_Off / Not
  // Active regardless of the values above.
  bool Vorladung_aktiv = false;
};

// Klemmen_Status_01 0x3C0
struct Klemmen {
  bool ZAS_Kl_S = false;                   // KeySwitch Inserted
  bool ZAS_Kl_15 = false;                  // Acc position
  bool ZAS_Kl_X = false;                   // Run position
  bool ZAS_Kl_50_Startanforderung = false; // Start
};

// BMS_01 0x191
struct Bms01 {
  uint16_t BMS_Batt_Curr = 2047; // 1A per bit, offset 2047
  float packVoltage = 400;       // V, used when no pack side reading exists
  float linkVoltage = 400;       // V, at the HV terminals
};

extern Hvk01 hvk01;
extern Klemmen klemmen;
extern Bms01 bms01;

// Pack voltage sent in BMS_01: udcsw while a DC-DC precharges, as that is the
// precharge target, else the pack side shunt / contactor box reading when
// there is one, else bms01.packVoltage.
float PackVoltage();

void Request10Ms(CanHardware *can);  // BMS_01
void Request100Ms(CanHardware *can); // HVK_01 and Klemmen_Status_01
void Task10Ms();
void Task100Ms();

} // namespace VagHvk

#endif // VAG_HVK_H
