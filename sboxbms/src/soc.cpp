/*
 * State of charge: coulomb counting from an ISA/IVT shunt on CAN1, anchored
 * to the MEB cell voltage curve at boot, after a long sleep and after rest.
 * The shunt is on permanent 12V, so its As counter keeps counting while we
 * sleep and a short sleep needs no re-anchor.
 *
 * Ported from the STM32-canbridge meb-simpbms branch. The ISA voltage inputs
 * are also kept here, because the S-Box emulation reports them to the Zombie.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "soc.h"
#include "bms_config.h"

static float soc;  /* % */
static float soc0; /* SoC at last anchor */
static uint8_t seeded;
static uint8_t reanchorPending;

static int32_t rawMa;
static float currentA; /* positive = discharge */
static uint32_t lastCurrentRx;
static uint8_t currentSeen;

static int32_t asRaw;
static int32_t asBase;
static uint32_t lastAsRx;
static uint8_t asSeen;
static uint8_t usingAs;

static float ahIntegrated; /* fallback integration since anchor */
static uint32_t restStart;
static uint32_t lastTaskMs;
static uint8_t taskSeen;

static int32_t uBattMv;
static uint32_t lastUBattRx;
static uint8_t uBattSeen;

static int32_t uOutMv;
static uint32_t lastUOutRx;
static uint8_t uOutSeen;

/*
 * ZombieVerter configures the ISA for little-endian output and decodes it
 * that way (isa_shunt.cpp): bytes 2..5, byte 2 least significant.
 */
static int32_t le32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static float clamp_soc(float v) {
  return v < 0.0f ? 0.0f : (v > 100.0f ? 100.0f : v);
}

static void forget_shunt(void) {
  currentSeen = asSeen = uBattSeen = uOutSeen = 0;
}

static void anchor(float value, uint32_t now) {
  soc = clamp_soc(value);
  soc0 = soc;
  asBase = asRaw;
  ahIntegrated = 0.0f;
  restStart = now;
  seeded = 1;
}

void soc_init(void) {
  soc = soc0 = 0.0f;
  seeded = 0;
  reanchorPending = 0;
  rawMa = 0;
  currentA = 0.0f;
  forget_shunt();
  usingAs = 0;
  ahIntegrated = 0.0f;
  taskSeen = 0;
}

void soc_register_filters(CanHardware *can) {
  can->RegisterUserMessage(0x520, 0x7F0); /* 0x520..0x52F */
}

void soc_request_reanchor(void) {
  reanchorPending = 1;
  /* shunt data from before sleep is stale */
  forget_shunt();
}

void soc_note_wake(uint8_t longSleep) {
  if (longSleep) {
    soc_request_reanchor(); /* pack has rested: trust the voltage curve */
  } else {
    /* keep the anchor and carry on counting; wait for fresh shunt frames */
    forget_shunt();
  }
  taskSeen = 0;
}

uint8_t soc_is_shunt_id(uint32_t id) {
  return id >= SHUNT_ID_FIRST && id <= SHUNT_ID_LAST;
}

void soc_handle_rx(uint32_t id, const uint8_t *data, uint8_t dlc,
                   uint32_t now) {
  if (dlc < 6)
    return;

  switch (id) {
  case SHUNT_ID_CURRENT:
    rawMa = le32(&data[2]);
    currentA = (float)(SHUNT_SIGN * rawMa) / 1000.0f;
    lastCurrentRx = now;
    currentSeen = 1;
    break;
  case SHUNT_ID_AS:
    asRaw = SHUNT_SIGN * le32(&data[2]);
    lastAsRx = now;
    asSeen = 1;
    break;
  case SHUNT_ID_UBATT:
    uBattMv = le32(&data[2]);
    lastUBattRx = now;
    uBattSeen = 1;
    break;
  case SHUNT_ID_UOUT:
    uOutMv = le32(&data[2]);
    lastUOutRx = now;
    uOutSeen = 1;
    break;
  default:
    break;
  }
}

uint8_t soc_shunt_ok(uint32_t now) {
  return currentSeen && (now - lastCurrentRx) <= SHUNT_TIMEOUT_MS;
}

static uint8_t as_ok(uint32_t now) {
  return asSeen && (now - lastAsRx) <= SHUNT_TIMEOUT_MS;
}

void soc_task(uint32_t now, uint8_t bmsAlive, const MEB_STATS *st) {
  float vSoc;
  uint8_t wantAs;
  float dtS;

  /* integrate over the real tick time; the task period is "at least" 100 ms */
  dtS =
      taskSeen ? (float)(now - lastTaskMs) / 1000.0f : TASK_PERIOD_MS / 1000.0f;
  if (dtS > 1.0f)
    dtS = 1.0f;
  lastTaskMs = now;
  taskSeen = 1;

  if (!bmsAlive)
    return; /* hold last value until cell data is valid */

  vSoc = meb_soc_from_voltage(st->minCellMv);

  if (!seeded || reanchorPending) {
    reanchorPending = 0;
    anchor(vSoc, now);
    usingAs = as_ok(now);
    return;
  }

  if (!soc_shunt_ok(now)) {
    /* no shunt: voltage only; re-anchor on the current value when it returns */
    soc = vSoc;
    anchor(soc, now);
    usingAs = as_ok(now);
    return;
  }

  /* switch between the shunt's As counter and our own integration without a
   * jump */
  wantAs = as_ok(now);
  if (wantAs != usingAs) {
    anchor(soc, now);
    usingAs = wantAs;
  }

  ahIntegrated += currentA * dtS / 3600.0f;

  {
    float deltaAh = usingAs ? (float)(asRaw - asBase) / 3600.0f : ahIntegrated;
    soc = clamp_soc(soc0 - deltaAh / MAX_AH * 100.0f);
  }

  /* end points */
  if (st->maxCellMv >= CELL_VMAX_MV)
    anchor(100.0f, now);
  else if (st->minCellMv <= CUTOFF_MV)
    anchor(0.0f, now);

  /* re-anchor to the voltage curve after a long rest */
  if (currentA > REST_CURRENT_A || currentA < -REST_CURRENT_A) {
    restStart = now;
  } else if ((now - restStart) >= REST_MS) {
    anchor(vSoc, now);
  }
}

float soc_get(void) { return soc; }

float soc_current_a(uint32_t now) {
  return soc_shunt_ok(now) ? currentA : 0.0f;
}

int32_t soc_raw_ma(uint32_t now) { return soc_shunt_ok(now) ? rawMa : 0; }

int32_t soc_batt_mv(uint32_t now) {
  return (uBattSeen && (now - lastUBattRx) <= SHUNT_TIMEOUT_MS) ? uBattMv : 0;
}

int32_t soc_out_mv(uint32_t now) {
  return (uOutSeen && (now - lastUOutRx) <= SHUNT_TIMEOUT_MS) ? uOutMv : 0;
}

float soc_pack_mv(uint32_t now, float cellSumMv) {
#if SHUNT_USE_UBATT
  int32_t mv = soc_batt_mv(now);
  if (mv > 0)
    return (float)mv;
#endif
  (void)now;
  return cellSumMv;
}
