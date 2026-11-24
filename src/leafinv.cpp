/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
 *               2024-     Tom de Bree <Tom@voltinflux.com>
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

#include "leafinv.h"
#include "NissLeafMng.h"
#include "digio.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include "stm32_can.h"
#include "utils.h"

// Startup supervisor timings, in 10ms ticks
#define INV_START_HOLD_TICKS 300   // 3s torque hold + rpm mask after power-up
#define INV_RESTART_WIN_TICKS 1000 // 10s window after power-up for auto restart
#define INV_RESTART_OFF_TICKS 300  // 3s inverter 12V off time
#define INV_SILENT_TICKS 50        // 0x1DA must be silent 500ms before power on
#define INV_FAULT_DEBOUNCE 20      // error flag must persist 200ms
#define INV_RESTART_MAX 5          // restart attempts per key cycle
#define INV_RESTART_MAX_RPM 100    // only restart below this motor speed
#define INV_NOT_SWITCHED_TICKS 500 // 0x1DA still present 5s after off time:
                                   // inv_out does not power the inverter

/*Info on running Leaf Gen 2 PDM
IDs required :
0x1D4 VCM (10ms)
0x1DB LBC (10ms)
0x1DC LBC (10ms)
0x1F2 VCM (10ms)
0x50B VCM (100ms)
0x55B LBC (100ms)
0x59E LBC (500ms)
0x5BC LBC (100ms)
PDM sends:
0x390 (100ms)
0x393 (100ms)
0x679 on evse plug insert

*/

void LeafINV::SetCanInterface(CanHardware *c) {
  NissLeafMng::SetCanInterface(
      c); // set Leaf VCM messages on same bus as Inverter
  can = c;

  can->RegisterUserMessage(0x1DA); // Leaf inv msg
  can->RegisterUserMessage(0x55A); // Leaf inv msg
}

void LeafINV::DecodeCAN(int id, uint32_t data[2]) {
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)

  if (id == 0x1DA) // THIS MSG CONTAINS INV VOLTAGE, MOTOR SPEED AND ERROR STATE
  {
    voltage = (bytes[0] * 2); // MEASURED VOLTAGE FROM LEAF INVERTER

    if (Param::GetInt(Param::ShuntType) == 0 &&
        voltage <
            420) // Only populate if no shunt is used and voltage is under 420
    {
      Param::SetFloat(Param::udc, voltage);
    }

    error = (bytes[6] & 0xb0) != 0x00; // INVERTER ERROR STATE

    // During its ~260ms power-up self test the inverter sends 0x7FFE/0x7FFF
    // in bytes 4-5, which would otherwise decode as 16383rpm.
    if (bytes[4] == 0x7F && bytes[5] >= 0xFE) {
      speed = 0;
      speedValid = false;
    } else {
      int16_t parsed_speed = (bytes[4] << 7) | bytes[5] >> 1;
      if (parsed_speed > 0x3fff)
        parsed_speed -= 0x7fff; // 15 bit signed conversion
      speed = parsed_speed;
      speedValid = true;
      if (!error)
        lastValidSpeed = speed; // faulted frames report 0rpm, ignore them
    }
    rxCount++;

  } else if (id == 0x55A) // THIS MSG CONTAINS INV TEMP AND MOTOR TEMP
  {
    inv_temp = fahrenheit_to_celsius(bytes[2]);   // INVERTER TEMP
    motor_temp = fahrenheit_to_celsius(bytes[1]); // MOTOR TEMP
  }
}

void LeafINV::SetTorque(float torquePercent) {
  final_torque_request = (torquePercent * 2047) / 100.0f;

  if (Param::GetInt(Param::reversemotor) == 1) {
    final_torque_request *= -1; // reverse torque request to flip motor rotation
  }

  Param::SetInt(Param::torque,
                final_torque_request); // post processed final torque value sent
                                       // to inv to web interface
}

void LeafINV::Task10Ms() // ONLY RAN IN RUNMODE
{

  NissLeafMng::Task10Ms(
      final_torque_request); // This is only called when in run mode, send
                             // torque request with it
}

void LeafINV::Task100Ms() // Always ran
{
  // Inverter calling of 100ms VCM task takes precedences over PDM
  NissLeafMng::Task100Ms();
}

void LeafINV::StartupArmHold() {
  startState = STARTUP_HOLD;
  holdTicks = INV_START_HOLD_TICKS;
  winTicks = INV_RESTART_WIN_TICKS;
  faultTicks = 0;
}

/*
 * Leaf inverter startup supervisor, called every 10ms.
 *
 * After each inverter power-up the torque request is held at 0 and the motor
 * speed is masked to 0 for INV_START_HOLD_TICKS, and until the inverter
 * reports a valid speed with no error and the throttle is released.
 *
 * At full charge the inverter can latch a fault ~0.3-1s after power-up which
 * only clears with a 12V power cycle. If that happens inside the startup
 * window while stationary, inv_out is switched off for INV_RESTART_OFF_TICKS
 * (and until 0x1DA stops), then the main state machine powers it back on.
 */
void LeafINV::StartupTask10Ms(int opmode, bool throttleReleased) {
  if (opmode != MOD_RUN) {
    startState = STARTUP_OFF;
    restartInProgress = false;
    restartCount = 0;
    faultTicks = 0;
    return;
  }

  if (startState == STARTUP_OFF)
    StartupArmHold(); // just entered RUN

  // Track whether the inverter is talking
  uint32_t rx = rxCount;
  if (rx != lastRxCount) {
    lastRxCount = rx;
    silentTicks = 0;
  } else if (silentTicks < 0xFFFF) {
    silentTicks++;
  }
  bool invAlive = silentTicks < INV_SILENT_TICKS;

  if (winTicks > 0)
    winTicks--;

  switch (startState) {
  case STARTUP_HOLD:
  case STARTUP_READY:
    if (error && invAlive) {
      if (faultTicks < 0xFFFF)
        faultTicks++;
    } else {
      faultTicks = 0;
    }

    if (faultTicks >= INV_FAULT_DEBOUNCE) {
      if (winTicks > 0 && ABS(lastValidSpeed) < INV_RESTART_MAX_RPM &&
          restartCount < INV_RESTART_MAX) {
        startState = STARTUP_RESTARTING;
        restartInProgress = true;
        offTicks = 0;
        DigIo::inv_out.Clear(); // inverter 12V off
      } else {
        startState =
            (restartCount >= INV_RESTART_MAX) ? STARTUP_GAVEUP : STARTUP_FAULT;
        restartInProgress = false;
        ErrorMessage::Post(ERR_INVFAULT);
      }
      break;
    }

    if (startState == STARTUP_HOLD) {
      if (holdTicks > 0) {
        holdTicks--;
      } else if (invAlive && speedValid && !error && throttleReleased) {
        startState = STARTUP_READY;
        restartInProgress = false;
      }
    }
    break;

  case STARTUP_RESTARTING:
    DigIo::inv_out.Clear(); // keep inverter 12V off
    if (offTicks < 0xFFFF)
      offTicks++;

    if (offTicks >= INV_RESTART_OFF_TICKS && !invAlive) {
      // Inverter is fully off. Forget its old state and let the main state
      // machine power it back on.
      restartCount++;
      error = false;
      speedValid = false;
      lastValidSpeed = 0;
      StartupArmHold();
    } else if (offTicks >= INV_RESTART_OFF_TICKS + INV_NOT_SWITCHED_TICKS) {
      // Still talking long after power was removed: inv_out isn't what
      // powers this inverter, so a restart can't work.
      startState = STARTUP_GAVEUP;
      restartInProgress = false;
      ErrorMessage::Post(ERR_INVFAULT);
    }
    break;

  case STARTUP_FAULT:
  case STARTUP_GAVEUP:
  case STARTUP_OFF:
  default:
    break; // torque stays held until the next key cycle
  }
}

int8_t LeafINV::fahrenheit_to_celsius(uint16_t fahrenheit) {
  int16_t result = ((int16_t)fahrenheit - 32) * 5 / 9;
  if (result < -128)
    return -128;
  if (result > 127)
    return 127;
  return result;
}
