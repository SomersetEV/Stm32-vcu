/*
 * VW MEB battery module (CMU) interface.
 * Port of STM32-canbridge meb_bms.h to libopeninv.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef MEB_BMS_H
#define MEB_BMS_H

#include "bms_config.h"
#include "canhardware.h"
#include <stdint.h>

typedef struct {
  float minCellMv;
  float maxCellMv;
  float packMv; /* sum of all cells */
  float lowTempC;
  float highTempC;
  float chargeLimitA;
  float dischargeLimitA;
} MEB_STATS;

void meb_init(CanHardware *can);
void meb_reset(void); /* forget all data (after wake) */
void meb_register_filters(CanHardware *can);
bool meb_is_id(uint32_t id);
void meb_handle_rx(uint32_t id, const uint8_t *data, uint8_t dlc, uint32_t now);
void meb_task(uint32_t now);      /* every TASK_PERIOD_MS */
void meb_balance(uint8_t enable); /* every TASK_PERIOD_MS */
void meb_balance_all_off(void);   /* before sleep */
uint8_t meb_alive(uint32_t now);
uint8_t meb_balancing_count(void); /* CMUs with a bleed resistor on */
const MEB_STATS *meb_stats(void);
float meb_soc_from_voltage(float cellMv); /* 0..100 % */

#endif
