/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2022  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
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

#ifndef FOSHANINVERTER_H
#define FOSHANINVERTER_H

#include <inverter.h>

/* Foshan motor controller, protocol TXSNCAN00120250619.
 * CAN 2.0B, 500kbps, Intel (little endian) byte order.
 * Command frame 0x501 every 20ms, feedback 0x504/0x505/0x506.
 * Driven here in torque control mode.
 */
class FoshanInverter : public Inverter {
public:
  FoshanInverter();
  void SetCanInterface(CanHardware *c);
  void DecodeCAN(int id, uint32_t data[2]);
  void Task1Ms();
  void SetTorque(float torquePercent);
  float GetMotorTemperature() { return motor_temp; }
  float GetInverterTemperature() { return inv_temp; }
  float GetInverterVoltage() { return dcVoltage; }
  float GetMotorSpeed() { return speed; }
  int GetInverterState() { return faultCode != 0; }

private:
  void SendControlFrame();

  uint8_t run1ms;      // divides the 1ms task down to the 20ms frame rate
  uint8_t fltRstCount; // remaining 20ms ticks to hold Fault_Reset asserted
  int16_t torqueRequest; // Nm, sent to the controller
  int16_t speed;         // rpm, signed
  int16_t motor_temp;    // °C
  int16_t inv_temp;      // °C
  uint8_t faultCode;
  uint16_t dtcCode;
  float dcVoltage; // V
  float dcCurrent; // A

  // Decoded for diagnostics, not currently used to control anything
  int16_t actualTorque; // Nm, reported by the controller
  int16_t torqueLimUp;  // Nm
  int16_t torqueLimDn;  // Nm
  float acCurrent;      // A
};

#endif // FOSHANINVERTER_H
