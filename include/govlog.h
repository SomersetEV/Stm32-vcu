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

#ifndef GOVLOG_H
#define GOVLOG_H

#include <stdint.h>

// Fast governor log frames, sent every 10 ms in run mode when GovLog = On.
// Logging only: nothing here feeds back into control. All multi-byte values
// are little-endian and saturate instead of wrapping.
//
// 0x4A0 10 ms  0-1 speed passed to the governor  uint16  1 rpm
//              2-3 speed error (limit - filtered) int16   0.125 rpm
//              4-5 user throttle before limits    int16   0.01 %
//              6-7 potnom after limits and ramp   int16   0.01 %
// 0x4A1 10 ms  0-1 integral term (govInt)        int16   0.01 %
//              2-3 P term                         int16   0.01 %
//              4-5 D term                         int16   0.01 %
//              6   flags, Throttle::GOV_FLAG_*    uint8
//              7   rolling counter                uint8
// 0x4A2 100 ms 0-1 revlim                         uint16  1 rpm
//              2-3 govKp as entered               uint16  raw fixed point (x32)
//              4-5 govKi as entered               uint16  raw fixed point (x32)
//              6-7 govKd as entered               uint16  raw fixed point (x32)
// 0x4A3 100 ms 0 govImax uint8 1 %, 1 govFlt uint8, 2 opmode uint8,
//              3 dir int8, 4 TorqDerate uint8, 5 regenmax int8 1 %,
//              6 throtramp uint8, 7 zero
namespace GovLog {
const uint32_t ID_FAST_SPEED = 0x4A0;
const uint32_t ID_FAST_TERMS = 0x4A1;
const uint32_t ID_SLOW_GAINS = 0x4A2;
const uint32_t ID_SLOW_STATE = 0x4A3;
const int SLOW_DIVIDER = 10; // 0x4A2 and 0x4A3 every 10th call

void Pack4A0(uint8_t *d, float potnom);
void Pack4A1(uint8_t *d, uint8_t counter);
void Pack4A2(uint8_t *d);
void Pack4A3(uint8_t *d);

int16_t SatS16(float v);
} // namespace GovLog

#endif // GOVLOG_H
