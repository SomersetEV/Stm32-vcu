/*
 * BMW PHEV "S-Box" emulation: takes the ZombieVerter's contactor commands
 * (0x100, 0x300) and reports current and voltages (0x200, 0x210, 0x220)
 * in the format ZombieVerter's bmw_sbox.cpp decodes.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef SBOX_EMU_H
#define SBOX_EMU_H

#include "canhardware.h"
#include <stdint.h>

enum SboxState { SBOX_OFF = 0, SBOX_PRECHARGE = 1, SBOX_RUN = 2 };

typedef void (*SboxOutputFn)(bool precharge, bool positive, bool negative);

void sbox_init(CanHardware *can, SboxOutputFn outputs);
void sbox_register_filters(CanHardware *can);
bool sbox_is_id(uint32_t id);
void sbox_handle_rx(uint32_t id, const uint8_t *data, uint8_t dlc,
                    uint32_t now);
/* Contactor state machine, every 10 ms. Voltages in mV, 0 = unknown. */
void sbox_task(uint32_t now, int32_t uBattMv, int32_t uOutMv);
void sbox_send(int32_t currentMa, int32_t uBattMv, int32_t uOutMv);
void sbox_force_off(void); /* before sleep */

SboxState sbox_state(void);
uint8_t sbox_cmd_byte(void);
bool sbox_cmd_valid(uint32_t now);
uint32_t sbox_crc_errors(void);
bool sbox_out_precharge(void);
bool sbox_out_positive(void);
bool sbox_out_negative(void);

#endif
