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

#include "govlog.h"
#include "params.h"
#include "throttle.h"
#include <string.h>

namespace GovLog {

// Rounds half away from zero; NaN gives 0.
int16_t SatS16(float v) {
  if (!(v == v))
    return 0;
  if (v >= 32767.0f)
    return 32767;
  if (v <= -32768.0f)
    return -32768;
  return (int16_t)(v >= 0 ? v + 0.5f : v - 0.5f);
}

static uint16_t SatU16(int32_t v) {
  return v < 0 ? 0 : (v > 65535 ? 65535 : (uint16_t)v);
}

static uint8_t SatU8(int32_t v) {
  return v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v);
}

static int8_t SatS8(int32_t v) {
  return v < -128 ? -128 : (v > 127 ? 127 : (int8_t)v);
}

static void PutU16(uint8_t *d, uint16_t v) {
  d[0] = v & 0xFF;
  d[1] = v >> 8;
}

static void PutS16(uint8_t *d, int16_t v) { PutU16(d, (uint16_t)v); }

void Pack4A0(uint8_t *d, float potnom) {
  memset(d, 0, 8);
  PutU16(d + 0, SatU16(Throttle::govSpeedIn));
  PutS16(d + 2, SatS16(Throttle::govSpeedErr * 8.0f));
  PutS16(d + 4, SatS16(Throttle::potUser * 100.0f));
  PutS16(d + 6, SatS16(potnom * 100.0f));
}

void Pack4A1(uint8_t *d, uint8_t counter) {
  memset(d, 0, 8);
  PutS16(d + 0, SatS16(Throttle::govIntegral * 100.0f));
  PutS16(d + 2, SatS16(Throttle::govPTerm * 100.0f));
  PutS16(d + 4, SatS16(Throttle::govDTerm * 100.0f));
  d[6] = Throttle::govFlags;
  d[7] = counter;
}

void Pack4A2(uint8_t *d) {
  memset(d, 0, 8);
  PutU16(d + 0, SatU16(Param::GetInt(Param::revlim)));
  PutU16(d + 2, SatU16(Param::Get(Param::govKp)));
  PutU16(d + 4, SatU16(Param::Get(Param::govKi)));
  PutU16(d + 6, SatU16(Param::Get(Param::govKd)));
}

void Pack4A3(uint8_t *d) {
  memset(d, 0, 8);
  d[0] = SatU8(Param::GetInt(Param::govImax));
  d[1] = SatU8(Param::GetInt(Param::govFlt));
  d[2] = SatU8(Param::GetInt(Param::opmode));
  d[3] = (uint8_t)SatS8(Param::GetInt(Param::dir));
  d[4] = SatU8(Param::GetInt(Param::TorqDerate));
  d[5] = (uint8_t)SatS8(Param::GetInt(Param::regenmax));
  d[6] = SatU8(Param::GetInt(Param::throtramp));
}

} // namespace GovLog
