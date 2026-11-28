/*
 * State of charge and ISA shunt readings.
 * Port of STM32-canbridge soc.h to libopeninv.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef SOC_H
#define SOC_H

#include "meb_bms.h"
#include <stdint.h>

void soc_init(void);
void soc_register_filters(CanHardware *can);
uint8_t soc_is_shunt_id(uint32_t id);
void soc_handle_rx(uint32_t id, const uint8_t *data, uint8_t dlc, uint32_t now);
void soc_task(uint32_t now, uint8_t bmsAlive,
              const MEB_STATS *st);    /* every TASK_PERIOD_MS */
void soc_request_reanchor(void);       /* re-seed from voltage on next task */
void soc_note_wake(uint8_t longSleep); /* after sleep */
float soc_get(void);                   /* 0..100 % */
uint8_t soc_shunt_ok(uint32_t now);
float soc_current_a(uint32_t now); /* positive = discharge, 0 if no shunt */
int32_t soc_raw_ma(uint32_t now);  /* raw ISA mA, 0 if no shunt          */
float soc_pack_mv(uint32_t now, float cellSumMv);
/* ISA voltage inputs; return 0 when stale */
int32_t soc_batt_mv(uint32_t now); /* battery side of the contactors  */
int32_t soc_out_mv(uint32_t now);  /* inverter side of the contactors */

#endif
