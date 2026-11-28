/*
 * SimpBMS output frames for the real ZombieVerter.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef SIMPBMS_TX_H
#define SIMPBMS_TX_H

#include "canhardware.h"
#include <stdint.h>

void simpbms_init(CanHardware *can);
/* Send 0x373, 0x351, 0x356 and 0x355. Call every TASK_PERIOD_MS.
 * Sends nothing while the MEB modules are not alive. */
void simpbms_send(uint32_t now);

#endif
