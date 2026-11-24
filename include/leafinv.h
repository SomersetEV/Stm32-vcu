/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
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

#ifndef LEAFINV_H
#define LEAFINV_H

#include "inverter.h"
#include "my_fp.h"
#include <stdint.h>

class LeafINV : public Inverter {
public:
  void DecodeCAN(int id, uint32_t data[2]);
  void Task10Ms();
  void Task100Ms();
  static bool ControlCharge(bool RunCh);
  void SetTorque(float torque);
  float GetMotorTemperature() { return motor_temp; }
  float GetInverterTemperature() { return inv_temp; }
  float GetInverterVoltage() { return voltage; }
  float GetMotorSpeed() { return speed; }
  int GetInverterState() { return error; }
  void SetCanInterface(CanHardware *c);

  // Startup supervisor. Call every 10ms regardless of opmode.
  // Holds torque at zero and masks motor speed after each inverter power-up,
  // and power cycles the inverter (via inv_out) if it latches its startup
  // fault while stationary.
  void StartupTask10Ms(int opmode, bool throttleReleased);
  bool TorqueHold() const {
    return startState != STARTUP_OFF && startState != STARTUP_READY;
  }
  bool SpeedMasked() const { return TorqueHold(); }
  bool PowerInhibit() const { return startState == STARTUP_RESTARTING; }
  bool HeaterAllowed() const { return startState == STARTUP_READY; }
  bool RestartInProgress() const { return restartInProgress; }

private:
  enum StartupState {
    STARTUP_OFF = 0,
    STARTUP_HOLD,
    STARTUP_READY,
    STARTUP_RESTARTING,
    STARTUP_FAULT,
    STARTUP_GAVEUP
  };

  uint8_t nissan_crc(uint8_t *data);
  static int8_t fahrenheit_to_celsius(uint16_t fahrenheit);
  void StartupArmHold();
  uint32_t lastRecv;
  int16_t speed;
  int16_t lastValidSpeed;
  bool speedValid;
  int16_t inv_temp;
  int16_t motor_temp;
  bool error;
  uint16_t voltage;
  int16_t final_torque_request;

  // 0x1DA receive counter, used to detect the inverter going silent
  volatile uint32_t rxCount;

  // Startup supervisor state
  StartupState startState;
  bool restartInProgress;
  uint16_t holdTicks;
  uint16_t winTicks;
  uint16_t offTicks;
  uint16_t faultTicks;
  uint16_t silentTicks;
  uint32_t lastRxCount;
  uint8_t restartCount;
};

#endif // LEAFINV_H
