/*
 * BMW PHEV "S-Box" emulation for the ZombieVerter.
 *
 * What the real Zombie sends (bmw_sbox.cpp, SBOX::ControlContactors, 10 ms):
 *  0x300 [FF FE FF FF]      byte 1 = 0xFE enables the contactors
 *  0x100 [cmd, ctr<<4|1, FF, crc]
 *        cmd 0x00 = all off, 0xA6 = precharge + negative, 0xAA = all on
 *        crc = BMW_crc8() over {cmd, ctr<<4|1, FF, 0, 0, 0, 0, 0}
 *
 * What it expects back (SBOX::handle200/210/220): bytes 0..2 int24 LE
 *  0x200 current mA, 0x210 battery voltage mV, 0x220 output voltage mV.
 *  The Zombie uses 0x220 as udc and 0x210 - 20 V as udcsw, so precharge
 *  only completes once the inverter side has charged up.
 *
 * The contactors follow the Zombie only; this box never trips them on a BMS
 * fault. It does open everything when the commands stop arriving, and it
 * will not close the positive contactor until precharge has done its job.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "sbox_emu.h"
#include "bms_config.h"
#include <string.h>

/* Defined in ../src/bmw_sbox.cpp: the exact CRC the Zombie uses */
uint8_t BMW_crc8(const uint8_t *data, const uint16_t size);

static CanHardware *zombieCan;
static SboxOutputFn setOutputs;

static volatile uint8_t cmdByte;
static volatile uint32_t lastCmdRx;
static volatile uint8_t cmdSeen;
static volatile uint8_t enableByte;
static volatile uint32_t lastEnableRx;
static volatile uint8_t enableSeen;
static volatile uint32_t crcErrors;

static SboxState state;
static uint32_t stateStart;
static bool outPre, outPos, outNeg;

static void apply(bool pre, bool pos, bool neg) {
  outPre = pre;
  outPos = pos;
  outNeg = neg;
  if (setOutputs)
    setOutputs(pre, pos, neg);
}

void sbox_init(CanHardware *can, SboxOutputFn outputs) {
  zombieCan = can;
  setOutputs = outputs;
  cmdSeen = enableSeen = 0;
  cmdByte = SBOX_CMD_OFF;
  crcErrors = 0;
  state = SBOX_OFF;
  stateStart = 0;
  apply(false, false, false);
}

void sbox_register_filters(CanHardware *can) {
  can->RegisterUserMessage(SBOX_ID_CMD);
  can->RegisterUserMessage(SBOX_ID_ENABLE);
}

bool sbox_is_id(uint32_t id) {
  return id == SBOX_ID_CMD || id == SBOX_ID_ENABLE;
}

void sbox_handle_rx(uint32_t id, const uint8_t *data, uint8_t dlc,
                    uint32_t now) {
  if (id == SBOX_ID_CMD) {
    if (dlc < 4)
      return;
#if SBOX_CHECK_CRC
    {
      uint8_t buf[8] = {data[0], data[1], data[2], 0, 0, 0, 0, 0};
      if (BMW_crc8(buf, 8) != data[3]) {
        crcErrors = crcErrors + 1;
        return;
      }
    }
#endif
    cmdByte = data[0];
    lastCmdRx = now;
    cmdSeen = 1;
  } else if (id == SBOX_ID_ENABLE) {
    if (dlc < 2)
      return;
    enableByte = data[1];
    lastEnableRx = now;
    enableSeen = 1;
  }
}

bool sbox_cmd_valid(uint32_t now) {
  return cmdSeen && (now - lastCmdRx) <= CMD_TIMEOUT_MS && enableSeen &&
         (now - lastEnableRx) <= CMD_TIMEOUT_MS &&
         enableByte == SBOX_ENABLE_BYTE;
}

static SboxState requested(uint32_t now) {
  if (!sbox_cmd_valid(now))
    return SBOX_OFF;

  switch (cmdByte) {
  case SBOX_CMD_PRECHARGE:
    return SBOX_PRECHARGE;
  case SBOX_CMD_RUN:
    return SBOX_RUN;
  default:
    return SBOX_OFF; /* 0x00 and anything unexpected */
  }
}

static bool precharged(int32_t uBattMv, int32_t uOutMv) {
  return uBattMv >= PRECHG_MIN_BATT_MV && uOutMv >= uBattMv - PRECHG_DV_MV;
}

static void enter(SboxState s, uint32_t now) {
  state = s;
  stateStart = now;
}

void sbox_task(uint32_t now, int32_t uBattMv, int32_t uOutMv) {
  SboxState req = requested(now);

  switch (state) {
  case SBOX_OFF:
    /* a RUN request straight from off still goes through precharge */
    if (req != SBOX_OFF)
      enter(SBOX_PRECHARGE, now);
    break;
  case SBOX_PRECHARGE:
    if (req == SBOX_OFF)
      enter(SBOX_OFF, now);
    else if (req == SBOX_RUN && (now - stateStart) >= PRECHG_MIN_MS &&
             precharged(uBattMv, uOutMv))
      enter(SBOX_RUN, now);
    break;
  case SBOX_RUN:
    if (req == SBOX_OFF)
      enter(SBOX_OFF, now);
    else if (req == SBOX_PRECHARGE)
      enter(SBOX_PRECHARGE, now);
    break;
  }

  switch (state) {
  case SBOX_OFF:
    apply(false, false, false);
    break;
  case SBOX_PRECHARGE:
    apply(true, false, true);
    break;
  case SBOX_RUN:
    apply((now - stateStart) < PRECHG_OVERLAP_MS, true, true);
    break;
  }
}

void sbox_force_off(void) {
  cmdSeen = enableSeen = 0;
  state = SBOX_OFF;
  apply(false, false, false);
}

static void send24(uint32_t id, int32_t v) {
  uint8_t b[8] = {0};
  uint32_t d[2];

  if (v > 0x7FFFFF)
    v = 0x7FFFFF;
  if (v < -0x800000)
    v = -0x800000;

  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
  b[2] = (uint8_t)((v >> 16) & 0xFF);
  memcpy(d, b, 8);
  zombieCan->Send(id, d, 8);
}

static int32_t clamp_mv(int32_t mv) {
  if (mv < 0)
    return 0;
  return mv > SBOX_MAX_MV ? SBOX_MAX_MV : mv;
}

void sbox_send(int32_t currentMa, int32_t uBattMv, int32_t uOutMv) {
  if (!zombieCan)
    return;
  send24(SBOX_ID_CURRENT, SBOX_CURRENT_SIGN * currentMa);
  send24(SBOX_ID_UBATT, clamp_mv(uBattMv));
  send24(SBOX_ID_UOUT, clamp_mv(uOutMv));
}

SboxState sbox_state(void) { return state; }
uint8_t sbox_cmd_byte(void) { return cmdByte; }
uint32_t sbox_crc_errors(void) { return crcErrors; }
bool sbox_out_precharge(void) { return outPre; }
bool sbox_out_positive(void) { return outPos; }
bool sbox_out_negative(void) { return outNeg; }
