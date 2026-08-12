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

#include "foshaninverter.h"
#include "my_math.h"
#include "params.h"

#define FOSHAN_CTRL_MODE_TORQUE 2
#define FOSHAN_TORQUE_OFFSET 5000
#define FOSHAN_SPEED_OFFSET 15000
#define FOSHAN_SPEED_LIM_SCALE 4 // rpm per bit of the 12 bit limit fields
#define FOSHAN_LIM_MAX 4095      // 12 bit limit fields
#define FOSHAN_FLTRST_TICKS 25   // 25 * 20ms = 500ms fault reset pulse

FoshanInverter::FoshanInverter() {
  run1ms = 0;
  fltRstCount = 0;
  torqueRequest = 0;
  speed = 0;
  actualTorque = 0;
  motor_temp = 0;
  inv_temp = 0;
  torqueLimUp = 0;
  torqueLimDn = 0;
  faultCode = 0;
  dtcCode = 0;
  acCurrent = 0;
  dcCurrent = 0;
  dcVoltage = 0;
}

void FoshanInverter::SetCanInterface(CanHardware *c) {
  can = c;

  can->RegisterUserMessage(0x504); // MCU_status1
  can->RegisterUserMessage(0x505); // MCU_status2
  can->RegisterUserMessage(0x506); // MCU_status3
}

void FoshanInverter::DecodeCAN(int id, uint32_t data[2]) {
  uint8_t *bytes = (uint8_t *)data; // Intel byte order, so byte 0 comes first

  switch (id) {
  case 0x504: // MCU_status1
    speed = (bytes[0] | (bytes[1] << 8)) - FOSHAN_SPEED_OFFSET;
    actualTorque = (bytes[2] | (bytes[3] << 8)) - FOSHAN_TORQUE_OFFSET;
    acCurrent = (bytes[4] | (bytes[5] << 8)) * 0.1f;
    break;
  case 0x505: // MCU_status2
    motor_temp = bytes[0] - 110;
    inv_temp = bytes[1] - 110;
    torqueLimUp = bytes[2] | (bytes[3] << 8);
    torqueLimDn = (bytes[4] | (bytes[5] << 8)) - FOSHAN_TORQUE_OFFSET;
    faultCode = bytes[6] & 0x0F;
    Param::SetInt(Param::FoshanFault, faultCode);
    break;
  case 0x506: // MCU_status3
    dtcCode = bytes[0] | (bytes[1] << 8);
    dcCurrent = (bytes[2] | (bytes[3] << 8)) - 1000;
    dcVoltage = (bytes[4] | (bytes[5] << 8)) * 0.1f;
    Param::SetInt(Param::FoshanDTC, dtcCode);

    if (Param::GetInt(Param::ShuntType) == 0) // no shunt, use inverter values
    {
      Param::SetFloat(Param::udc, dcVoltage);
      Param::SetFloat(Param::idc, dcCurrent);
    }
    break;
  }
}

void FoshanInverter::SetTorque(float torquePercent) {
  float maxTorque = Param::GetFloat(Param::FoshanMaxTrq);
  float nm = (torquePercent * maxTorque) / 100.0f;

  if (Param::GetInt(Param::reversemotor) != 0) {
    nm = -nm; // reverses the direction the motor turns in
  }

  nm = MAX(-maxTorque, MIN(maxTorque, nm));
  torqueRequest = (int16_t)nm;

  Param::SetInt(Param::torque, torqueRequest); // Nm sent to inv, to web interface
}

void FoshanInverter::Task1Ms() {
  run1ms++;

  // The control frame runs on the 1ms task rather than the 10ms one so that it
  // keeps going in every opmode. The controller expects a continuous stream of
  // commands and needs to see a stop command to power down cleanly.
  if (run1ms == 20) {
    run1ms = 0;
    SendControlFrame();
  }
}

void FoshanInverter::SendControlFrame() {
  uint8_t bytes[8];

  uint8_t enable = Param::GetInt(Param::opmode) == MOD_RUN ? 1 : 0;

  // Fault reset is a one shot from the web interface. Latch it into a countdown
  // and clear the parameter so it can never stick on.
  if (Param::GetInt(Param::FoshanFltRst) != 0) {
    fltRstCount = FOSHAN_FLTRST_TICKS;
    Param::SetInt(Param::FoshanFltRst, 0);
  }

  uint8_t fltRst = 0;
  if (fltRstCount > 0) {
    fltRstCount--;
    fltRst = 1;
  }

  // In torque mode these two fields are the speed limits, 4rpm per bit. The
  // negative limit is sent as a positive value and applied as negative.
  uint16_t limit = Param::GetInt(Param::revlim) / FOSHAN_SPEED_LIM_SCALE;
  limit = MIN(limit, FOSHAN_LIM_MAX);

  uint16_t trqRaw = torqueRequest + FOSHAN_TORQUE_OFFSET;
  uint16_t spdRaw = FOSHAN_SPEED_OFFSET; // 0 rpm, unused in torque mode

  bytes[0] = enable | (fltRst << 1) | (FOSHAN_CTRL_MODE_TORQUE << 2);
  bytes[1] = limit & 0xFF;                              // DeLimitHighPosVal 7:0
  bytes[2] = ((limit >> 8) & 0x0F) | ((limit & 0x0F) << 4); // HighPos 11:8, LowNeg 3:0
  bytes[3] = (limit >> 4) & 0xFF;                       // DeLimitLowNegVal 11:4
  bytes[4] = trqRaw & 0xFF;
  bytes[5] = trqRaw >> 8;
  bytes[6] = spdRaw & 0xFF;
  bytes[7] = spdRaw >> 8;

  can->Send(0x501, bytes, 8);
}
