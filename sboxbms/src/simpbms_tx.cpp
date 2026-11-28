/*
 * SimpBMS output, as decoded by ZombieVerter's simpbms.cpp.
 * All multi-byte fields little-endian.
 *
 *  0x351  [0-1] charge voltage 0.1V  [2-3] charge current limit 0.1A
 *         [4-5] discharge current limit 0.1A  [6-7] discharge voltage 0.1V
 *  0x355  [0-1] SoC %  [2-3] SoH %  [4-5] SoC 0.01%
 *  0x356  [0-1] pack voltage 0.01V  [2-3] current 0.1A (int16, + = discharge)
 *         [4-5] avg module temp 0.1C
 *  0x373  [0-1] min cell mV  [2-3] max cell mV  [4-5] min temp K  [6-7] max K
 *
 * Ported from the STM32-canbridge meb-simpbms branch.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "simpbms_tx.h"
#include "bms_config.h"
#include "meb_bms.h"
#include "soc.h"
#include <string.h>

static CanHardware *vcuCan;

static void put16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static uint16_t u16_round(float v) {
  if (v <= 0.0f)
    return 0;
  if (v >= 65535.0f)
    return 65535;
  return (uint16_t)(v + 0.5f);
}

static int16_t s16_round(float v) {
  if (v >= 32767.0f)
    return 32767;
  if (v <= -32768.0f)
    return -32768;
  return (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static void send(uint32_t id, const uint8_t *data) {
  uint32_t d[2];

  memcpy(d, data, 8);
  vcuCan->Send(id, d, 8);
}

void simpbms_init(CanHardware *can) { vcuCan = can; }

void simpbms_send(uint32_t now) {
  const MEB_STATS *st = meb_stats();
  uint8_t d[8];
  float soc;

  if (!vcuCan || !meb_alive(now))
    return; /* let the VCU time out and fail safe */

  soc = soc_get();

  /* 0x373: cell and temperature extremes (also resets the VCU's BMS timeout) */
  put16(&d[0], u16_round(st->minCellMv));
  put16(&d[2], u16_round(st->maxCellMv));
  put16(&d[4], u16_round(st->lowTempC + 273.0f));
  put16(&d[6], u16_round(st->highTempC + 273.0f));
  send(0x373, d);

  /* 0x351: limits */
  put16(&d[0], u16_round((float)NUM_CELLS * CELL_VMAX_MV / 100.0f));
  put16(&d[2], u16_round(st->chargeLimitA * 10.0f));
  put16(&d[4], u16_round(st->dischargeLimitA * 10.0f));
  put16(&d[6], u16_round((float)NUM_CELLS * CUTOFF_MV / 100.0f));
  send(0x351, d);

  /* 0x356: pack voltage, current, temperature */
  put16(&d[0], u16_round(soc_pack_mv(now, st->packMv) / 10.0f));
  put16(&d[2], (uint16_t)s16_round(soc_current_a(now) * 10.0f));
  put16(&d[4], (uint16_t)s16_round((st->lowTempC + st->highTempC) * 5.0f));
  d[6] = 0;
  d[7] = 0;
  send(0x356, d);

  /* 0x355: state of charge */
  put16(&d[0], u16_round(soc));
  put16(&d[2], 100);
  put16(&d[4], u16_round(soc * 100.0f));
  d[6] = 0;
  d[7] = 0;
  send(0x355, d);
}
