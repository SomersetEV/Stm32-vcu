/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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

#include "my_fp.h"
#include "my_math.h"
#include "test_list.h"
#include "throttle.h"
#include <cstdio>
#include <cstring>
#include <stdint.h>

using namespace std;

void Param::Change(Param::PARAM_NUM p) {
  // Dummy function - we ignore parameter changes in these tests
}

// Include a dummy error list to allow the tests to link. It is unused.
const char *errorListString = "";

static void TestSetup() {
  // percentage of deadzone
  Throttle::throtdead = 5;
  Throttle::potmin[0] = 100;
  Throttle::potmax[0] = 4000;
  Throttle::throtmax = 100;
  Param::SetInt(Param::dir, 1);
}

// TEMPERATURE DERATING
static void TestThrottleTemperateOverMaxThrottleTo0() {
  float temp = 60;
  float tempMax = 50;
  float finalSpnt = 100;
  ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) &&
         finalSpnt == 0);
}

static void TestThrottleTemperateInDerateZoneThrottleTo50Percent() {
  float temp = 61;
  float tempMax = 60;
  float finalSpnt = 100;
  ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) &&
         finalSpnt == 50);
}

static void TestThrottleUnderTemperateNoDeRate() {
  float temp = 30;
  float tempMax = 60;
  float finalSpnt = 100;
  ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) == false &&
         finalSpnt == 100);
}

static void TestThrottleTemperateInDerateZoneThrottleButThrottleUnderLimit() {
  float temp = 61;
  float tempMax = 60;
  float finalSpnt = 49;
  ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) &&
         finalSpnt == 49);
}

// CALC THROTTLE
static void TestCalcThrottleIs0WhenThrottleAndBrakePressed() {

  ASSERT(Throttle::CalcThrottle(3000, 0, true) == 0);
}

static void TestCalcThrottleIs0WhenNoThrottleAndBrakePressed() {

  ASSERT(Throttle::CalcThrottle(0, 0, true) == 0);
}

static void TestCalcThrottleIs0WhenInDeadZone() {
  // deadzone is first 5% of travel between 100 and 4000
  ASSERT(Throttle::CalcThrottle(295, 0, false) == 0);
}

static void TestCalcThrottleIsAbove0WhenJustOutOfDeadZone() {
  // deadzone is first 5% of travel between 100 and 4000
  ASSERT(Throttle::CalcThrottle(496, 0, false) > 0);
}

static void TestCalcThrottleIs100WhenMax() {
  // deadzone is first 5% of travel between 100 and 4000
  float throtVal = Throttle::CalcThrottle(4000, 0, false);
  ASSERT(throtVal == 100);
}

static void TestCalcThrottleIs100WhenOverMax() {
  // deadzone is first 5% of travel between 100 and 4000
  float throtVal = Throttle::CalcThrottle(4001, 0, false);
  // ASSERT(throtVal == 100);
}

// GOVERNOR GOLDEN TEST
// A scripted sequence of speed and throttle inputs through SpeedLimitCommand,
// checked bit for bit against outputs recorded from the unmodified governor
// at commit d85e756. Changes that only add logging must leave every output
// identical. Must be the first call to SpeedLimitCommand in the process: at
// d85e756 the governor state lives in function statics and cannot be reset.
//
// To re-record (only ever against unmodified governor code):
//   make clean && make CPPFLAGS="-ggdb -I../include -I../libopeninv/include -DGOLDEN_RECORD"
//   ./test_vcu      (writes golden_governor.h in the current directory)
static const int GOLDEN_TICKS = 500;

#ifndef GOLDEN_RECORD
#include "golden_governor.h"
#endif

// Integer triangle wave, amplitude amp, period 4 * amp / step ticks.
static int Triangle(int t, int amp, int step) {
  int period = 4 * amp / step;
  int p = t % period;
  int q = period / 4;
  if (p < q) return p * step;
  if (p < 3 * q) return (2 * q - p) * step;
  return (p - 4 * q) * step;
}

static void GoldenInput(int t, int &speed, float &throttle) {
  if (t == 0) {
    Throttle::speedLimit = 2000;
    Throttle::govKp = 30 * 0.01f;
    Throttle::govKi = 2 * 0.001f;
    Throttle::govKd = 2;
    Throttle::govImax = 100;
    Throttle::regenmax = -10;
  }
  if (t == 300) {
    Throttle::govKi = 50 * 0.001f;
    Throttle::govImax = 20;
  }

  if (t < 100) {              // full throttle acceleration through the limit
    speed = t * 24;
    throttle = 100;
  } else if (t < 200) {       // ripple around the limit, baler-like
    speed = 2000 + Triangle(t, 60, 6);
    throttle = 100;
  } else if (t < 250) {       // pedal is the binding limit
    speed = 1950 + t % 5;
    throttle = 10;
  } else if (t < 300) {       // drop below half the limit: integral reset
    speed = MAX(0, 1950 - (t - 250) * 40);
    throttle = 100;
  } else if (t < 400) {       // just under the limit: integral clamps at govImax
    speed = 1990;
    throttle = 100;
  } else if (t < 450) {       // over the limit: regen, negative clamp
    speed = 2050;
    throttle = 100;
  } else {                    // driver regen request below the limit
    speed = 1500;
    throttle = -5;
  }
}

static void TestGovernorGolden() {
  int mismatches = 0;
  int first = -1;
#ifdef GOLDEN_RECORD
  static uint32_t spntBits[GOLDEN_TICKS];
  static int32_t intRaw[GOLDEN_TICKS];
#endif
  for (int t = 0; t < GOLDEN_TICKS; t++) {
    int speed;
    float finalSpnt;
    GoldenInput(t, speed, finalSpnt);
    Throttle::SpeedLimitCommand(finalSpnt, speed);
    uint32_t bits;
    memcpy(&bits, &finalSpnt, sizeof(bits));
    int32_t govInt = Param::Get(Param::govInt);
#ifdef GOLDEN_RECORD
    spntBits[t] = bits;
    intRaw[t] = govInt;
#else
    if (bits != goldenSpntBits[t] || govInt != goldenGovIntRaw[t]) {
      if (first < 0) first = t;
      mismatches++;
    }
#endif
  }
#ifdef GOLDEN_RECORD
  FILE *f = fopen("golden_governor.h", "w");
  fprintf(f, "// Recorded by TestGovernorGolden from unmodified SpeedLimitCommand.\n");
  fprintf(f, "static const uint32_t goldenSpntBits[GOLDEN_TICKS] = {\n");
  for (int t = 0; t < GOLDEN_TICKS; t++)
    fprintf(f, "%s0x%08X,%s", t % 6 == 0 ? "    " : " ", (unsigned)spntBits[t],
           t % 6 == 5 || t == GOLDEN_TICKS - 1 ? "\n" : "");
  fprintf(f, "};\nstatic const int32_t goldenGovIntRaw[GOLDEN_TICKS] = {\n");
  for (int t = 0; t < GOLDEN_TICKS; t++)
    fprintf(f, "%s%d,%s", t % 10 == 0 ? "    " : " ", (int)intRaw[t],
           t % 10 == 9 || t == GOLDEN_TICKS - 1 ? "\n" : "");
  fprintf(f, "};\n");
  fclose(f);
#else
  if (mismatches)
    cout << "Golden mismatch: " << mismatches << " ticks, first at tick " << first
         << endl;
#endif
  ASSERT(mismatches == 0);
}

void ThrottleTest::RunTest() {
  TestSetup();
  TestThrottleTemperateOverMaxThrottleTo0();
  TestThrottleTemperateInDerateZoneThrottleTo50Percent();
  TestThrottleUnderTemperateNoDeRate();
  TestThrottleTemperateInDerateZoneThrottleButThrottleUnderLimit();
  TestCalcThrottleIs0WhenThrottleAndBrakePressed();
  TestCalcThrottleIs0WhenNoThrottleAndBrakePressed();
  TestCalcThrottleIs0WhenInDeadZone();
  TestCalcThrottleIsAbove0WhenJustOutOfDeadZone();
  TestCalcThrottleIs100WhenMax();
  TestCalcThrottleIs100WhenOverMax();
  TestGovernorGolden(); // keep first of the governor tests, see above
}
