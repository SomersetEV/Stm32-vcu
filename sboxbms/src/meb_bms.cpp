/*
 * VW MEB battery module (CMU) interface.
 *
 * C port of mebbms.cpp from stm32_mebtoroadster,
 * Copyright (C) 2018-2021 Johannes Huebner, GPL v3 or later
 * (itself based on Tom de Bree's VW-bms). Modifications for STM32-canbridge,
 * then ported to libopeninv for the ZombieVerter hardware.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "meb_bms.h"
#include <string.h>

#define IIRFILTERF(l, n, c)                                                    \
  (((n) + ((l) * ((1 << (c)) - 1))) / (float)(1 << (c)))

#define CELL_FILTER 3
#define TEMP_FILTER 4

#define SOC_CURVE_MIN_MV 2850
#define SOC_CURVE_MAX_MV 4200
#define SOC_CURVE_STEP_MV 50

#define CELL_ID_FIRST 0x1C0
#define CELL_ID_LAST 0x1DF
#define TEMP_ID_FIRST 0x1A5555F4u
#define TEMP_ID_LAST 0x1A5555FBu

/* 2.85V .. 4.20V in 50 mV steps, SoC in 0.01 % */
static const uint16_t vtgToSoc[] = {0,    12,   31,   55,   80,   116,  153,
                                    202,  239,  325,  496,  701,  1231, 1794,
                                    2307, 3368, 4388, 5368, 5949, 6326, 6742,
                                    7232, 7708, 8104, 8575, 8996, 9446, 10000};
#define SOC_TABLE_ITEMS (sizeof(vtgToSoc) / sizeof(vtgToSoc[0]))

static CanHardware *mebCan;
static float cellMv[NUM_CELLS];
static uint8_t cellSeen[NUM_CELLS];
static float tempsC[NUM_CMUS];
static uint8_t tempSeen[NUM_CMUS];
static uint32_t lastRx[NUM_CMUS];     /* last temperature frame */
static uint32_t lastCellRx[NUM_CMUS]; /* last cell voltage frame */
static uint8_t cellRxSeen[NUM_CMUS];
static uint8_t balancerRunning[NUM_CMUS];
static uint8_t balCmu;
static uint8_t balCounter;
static MEB_STATS stats;

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void meb_reset(void) {
  memset(cellMv, 0, sizeof(cellMv));
  memset(cellSeen, 0, sizeof(cellSeen));
  memset(tempsC, 0, sizeof(tempsC));
  memset(tempSeen, 0, sizeof(tempSeen));
  memset(lastRx, 0, sizeof(lastRx));
  memset(lastCellRx, 0, sizeof(lastCellRx));
  memset(cellRxSeen, 0, sizeof(cellRxSeen));
  memset(balancerRunning, 0, sizeof(balancerRunning));
  memset(&stats, 0, sizeof(stats));
  balCmu = 0;
  balCounter = 0;
}

void meb_init(CanHardware *can) {
  mebCan = can;
  meb_reset();
}

void meb_register_filters(CanHardware *can) {
  can->RegisterUserMessage(CELL_ID_FIRST, 0x7E0); /* 0x1C0..0x1DF */
  for (uint32_t id = TEMP_ID_FIRST; id <= TEMP_ID_LAST; id++)
    can->RegisterUserMessage(id);
}

bool meb_is_id(uint32_t id) {
  return (id >= CELL_ID_FIRST && id <= CELL_ID_LAST) ||
         (id >= TEMP_ID_FIRST && id <= TEMP_ID_LAST);
}

static void set_cell(int idx, uint32_t raw) {
  uint32_t mv = raw + 1000;

  /* raw 0 is what a CMU reports before it has measured; it is not 1000 mV */
  if (idx >= NUM_CELLS || raw == 0 || mv > 4400)
    return;

  if (!cellSeen[idx]) {
    cellMv[idx] = (float)mv;
    cellSeen[idx] = 1;
  } else {
    cellMv[idx] = IIRFILTERF(cellMv[idx], (float)mv, CELL_FILTER);
  }
}

void meb_handle_rx(uint32_t id, const uint8_t *data, uint8_t dlc,
                   uint32_t now) {
  uint32_t d0, d1;

  if (dlc < 8)
    return;

  d0 = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
       ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
  d1 = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
       ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);

  /* libopeninv does not pass the IDE bit: an id > 0x7FF is extended */
  if (id >= CELL_ID_FIRST && id <= CELL_ID_LAST && (id & 3) != 3) {
    int group = (int)(((id - CELL_ID_FIRST + 1) * 3) / 4); /* 4-cell group */
    int cmu = group / 3;
    int base = group * 4;

    if (cmu >= NUM_CMUS)
      return;

    /* the CMU is talking even while its readings are skipped for balancing */
    lastCellRx[cmu] = now;
    cellRxSeen[cmu] = 1;

    if (balancerRunning[cmu])
      return;

    set_cell(base + 0, (d0 >> 12) & 0xFFF);
    set_cell(base + 1, (d0 >> 24) | ((d1 & 0xF) << 8));
    set_cell(base + 2, (d1 >> 4) & 0xFFF);
    set_cell(base + 3, (d1 >> 16) & 0xFFF);
  } else if (id >= TEMP_ID_FIRST && id <= TEMP_ID_LAST) {
    int cmu = (int)(id & 0xF) - 4;
    float t = (float)((d1 >> 4) & 0xFF) * 0.5f - 40.0f;

    if (cmu < 0 || cmu >= NUM_CMUS)
      return;

    if (!tempSeen[cmu]) {
      tempsC[cmu] = t;
      tempSeen[cmu] = 1;
    } else {
      tempsC[cmu] = IIRFILTERF(tempsC[cmu], t, TEMP_FILTER);
    }
    lastRx[cmu] = now;
  }
}

uint8_t meb_alive(uint32_t now) {
  int i;

  for (i = 0; i < NUM_CMUS; i++) {
    if (!tempSeen[i] || (now - lastRx[i]) > CMU_TIMEOUT_MS)
      return 0;
    if (!cellRxSeen[i] || (now - lastCellRx[i]) > CMU_TIMEOUT_MS)
      return 0;
  }
  for (i = 0; i < NUM_CELLS; i++) {
    if (!cellSeen[i])
      return 0;
  }
  return 1;
}

static float low_temp_derating(float lowTemp) {
  const float drt1Temp = 25.0f;
  const float drt2Temp = 0.0f;
  const float drt3Temp = -20.0f;
  const float factorAtDrt2 = 0.3f;

  if (lowTemp > drt1Temp)
    return 1.0f;
  if (lowTemp > drt2Temp)
    return factorAtDrt2 +
           (1.0f - factorAtDrt2) * (lowTemp - drt2Temp) / (drt1Temp - drt2Temp);
  if (lowTemp > drt3Temp)
    return factorAtDrt2 * (lowTemp - drt3Temp) / (drt2Temp - drt3Temp);
  return 0.0f;
}

static float high_temp_derating(float maxTemp, float highTemp) {
  return clampf((maxTemp - highTemp) * 0.15f, 0.0f, 1.0f);
}

static float charge_stage(float setpointMv, float maxA, float vmax) {
  return clampf((setpointMv - vmax) * CHG_KP_A_PER_MV, 0.0f, maxA);
}

void meb_task(uint32_t now) {
  int i;
  float mn = 5000.0f, mx = 0.0f, sum = 0.0f;
  float lo = 200.0f, hi = -100.0f;
  float target;

  if (!meb_alive(now)) {
    stats.chargeLimitA = 0.0f;
    stats.dischargeLimitA = 0.0f;
    return;
  }

  for (i = 0; i < NUM_CELLS; i++) {
    if (cellMv[i] < mn)
      mn = cellMv[i];
    if (cellMv[i] > mx)
      mx = cellMv[i];
    sum += cellMv[i];
  }
  for (i = 0; i < NUM_CMUS; i++) {
    if (tempsC[i] < lo)
      lo = tempsC[i];
    if (tempsC[i] > hi)
      hi = tempsC[i];
  }

  stats.minCellMv = mn;
  stats.maxCellMv = mx;
  stats.packMv = sum;
  stats.lowTempC = lo;
  stats.highTempC = hi;

  /* Charge: three-stage taper on the highest cell, then temperature derating */
  target = charge_stage(CHG_SP1_MV, CHG_I1_A, mx);
  {
    float s2 = charge_stage(CHG_SP2_MV, CHG_I2_A, mx);
    float s3 = charge_stage(CELL_VMAX_MV, CHG_I3_A, mx);
    if (s2 > target)
      target = s2;
    if (s3 > target)
      target = s3;
  }
  target *= low_temp_derating(lo) * high_temp_derating(CHG_TMAX_C, hi);

  if (target < stats.chargeLimitA)
    stats.chargeLimitA = target; /* drop immediately */
  else if (target > stats.chargeLimitA + CHG_RAMP_A_PER_TICK)
    stats.chargeLimitA += CHG_RAMP_A_PER_TICK; /* ramp up slowly   */
  else
    stats.chargeLimitA = target;

  /* Discharge: linear to zero at the cutoff, then high temperature derating */
  stats.dischargeLimitA =
      clampf((mn - CUTOFF_MV) * DCHG_A_PER_MV, 0.0f, DCHG_MAX_A) *
      high_temp_derating(DCHG_TMAX_C, hi);
}

static void send_bal_frames(uint8_t cmu, const uint8_t *cmds) {
  uint32_t d[2];

  if (!mebCan)
    return;

  memcpy(d, &cmds[0], 8);
  mebCan->Send((cmu < 5 ? 0x1A555412u : 0x1A5554A1u) + cmu * 2u, d, 8);

  memcpy(d, &cmds[8], 8);
  mebCan->Send((cmu < 5 ? 0x1A555413u : 0x1A5554A2u) + cmu * 2u, d, 8);
}

void meb_balance(uint8_t enable) {
  uint8_t cmds[16] = {0, 0, 0, 0, 0,    0,    0,    0,
                      0, 0, 0, 0, 0xFE, 0xFE, 0xFE, 0xFE};
  uint8_t balance, running = 0;
  int c;

  if (balCmu == 0) {
    balCounter = (uint8_t)((balCounter + 1) & 0x3);
  }

  /* balance 3 out of 4 rounds; the off round lets voltages settle and update */
  balance = enable && balCounter < 3 && stats.minCellMv > 0.0f;

  for (c = 0; c < CELLS_PER_CMU; c++) {
    float v = cellMv[balCmu * CELLS_PER_CMU + c];
    uint8_t flag = (v > stats.minCellMv + BAL_HYST_MV) && (v > BAL_MIN_MV);

    if (flag && balance) {
      cmds[c] = 0x08;
      running = 1;
    }
  }

  balancerRunning[balCmu] = running;
  send_bal_frames(balCmu, cmds);

  balCmu = (uint8_t)((balCmu + 1) % NUM_CMUS);
}

void meb_balance_all_off(void) {
  uint8_t cmds[16] = {0, 0, 0, 0, 0,    0,    0,    0,
                      0, 0, 0, 0, 0xFE, 0xFE, 0xFE, 0xFE};
  uint8_t cmu;

  for (cmu = 0; cmu < NUM_CMUS; cmu++) {
    balancerRunning[cmu] = 0;
    send_bal_frames(cmu, cmds);
  }
}

uint8_t meb_balancing_count(void) {
  uint8_t n = 0;

  for (int i = 0; i < NUM_CMUS; i++)
    n += balancerRunning[i] ? 1 : 0;
  return n;
}

const MEB_STATS *meb_stats(void) { return &stats; }

float meb_soc_from_voltage(float cellMvIn) {
  float v = clampf(cellMvIn, (float)SOC_CURVE_MIN_MV, (float)SOC_CURVE_MAX_MV) -
            SOC_CURVE_MIN_MV;
  int idx = (int)(v / SOC_CURVE_STEP_MV);
  float frac;

  if (idx >= (int)SOC_TABLE_ITEMS - 1)
    return vtgToSoc[SOC_TABLE_ITEMS - 1] / 100.0f;

  frac = (v - idx * SOC_CURVE_STEP_MV) / (float)SOC_CURVE_STEP_MV;
  return (vtgToSoc[idx] + (vtgToSoc[idx + 1] - vtgToSoc[idx]) * frac) / 100.0f;
}
