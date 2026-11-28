/*
 * Clocks, 1 ms SysTick time base and LSI RTC for the S-Box/BMS firmware.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef SYSCLK_H
#define SYSCLK_H

#include <stdint.h>

void sysclk_setup(void);   /* 72 MHz from HSE + peripheral clocks */
void sysclk_restore(void); /* after STOP: back to 72 MHz */
void systick_setup(void);  /* 1 kHz */
uint32_t millis(void);
void rtc_lsi_setup(void); /* RTC on LSI, ~1 s per count, alarm on EXTI17 */
void nvic_setup(void);

#endif
