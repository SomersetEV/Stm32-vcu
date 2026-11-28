/*
 * STOP-mode sleep with CAN2 wake for the S-Box/BMS firmware.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef POWER_H
#define POWER_H

#include <stdint.h>

void power_init(uint32_t now);
void power_note_zombie_rx(uint32_t now); /* 0x100 / 0x300 from the Zombie */
void power_note_other_rx(uint32_t now);  /* any other CAN2 frame */
bool power_zombie_active(uint32_t now);  /* Zombie heard recently */
void power_task(uint32_t now); /* main loop only: sleeps when CAN2 is quiet */

#endif
