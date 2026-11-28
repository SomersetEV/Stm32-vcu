/*
 * Host unit tests for the S-Box + SimpBMS firmware logic.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "bms_config.h"
#include "bmw_sbox.h"
#include "meb_bms.h"
#include "sbox_emu.h"
#include "simpbms_tx.h"
#include "soc.h"
#include <stdio.h>
#include <string.h>

static int failures = 0;
static int checks = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    checks++;                                                                  \
    if (!(cond)) {                                                             \
      failures++;                                                              \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
    }                                                                          \
  } while (0)

#define CHECK_EQ(a, b)                                                         \
  do {                                                                         \
    checks++;                                                                  \
    long long va_ = (long long)(a), vb_ = (long long)(b);                      \
    if (va_ != vb_) {                                                          \
      failures++;                                                              \
      printf("FAIL %s:%d: %s == %lld, expected %lld\n", __FILE__, __LINE__,    \
             #a, va_, vb_);                                                    \
    }                                                                          \
  } while (0)

class FakeCan : public CanHardware {
public:
  struct Frame {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
  };

  Frame frames[64];
  int count = 0;

  void SetBaudrate(enum baudrates) override {}
  void Send(uint32_t canId, uint32_t data[2], uint8_t len, bool) override {
    if (count < 64) {
      frames[count].id = canId;
      frames[count].len = len;
      memset(frames[count].data, 0, 8);
      memcpy(frames[count].data, data, len);
      count++;
    }
  }
  const Frame *Find(uint32_t id) const {
    for (int i = count - 1; i >= 0; i--)
      if (frames[i].id == id)
        return &frames[i];
    return 0;
  }
  void Reset() { count = 0; }

private:
  void ConfigureFilters() override {}
};

/* ----------------------------------------------------------- outputs --- */

static bool outPre, outPos, outNeg;

static void Outputs(bool pre, bool pos, bool neg) {
  outPre = pre;
  outPos = pos;
  outNeg = neg;
}

/* ------------------------------------------------ simulated Zombie link --- */

static FakeCan zombieSide; /* frames the real Zombie sends */
static FakeCan boxCan2;    /* frames this box sends on CAN2 */
static FakeCan boxCan1;    /* frames this box sends on CAN1 */
static uint32_t now;

/* one 10 ms tick: the Zombie sends its frames, then our task runs */
static void zombie_tick(int opmode, int32_t uBatt, int32_t uOut,
                        bool send0x300 = true, bool corruptCrc = false) {
  zombieSide.Reset();
  SBOX::ControlContactors(opmode, &zombieSide);
  for (int i = 0; i < zombieSide.count; i++) {
    FakeCan::Frame f = zombieSide.frames[i];
    if (f.id == SBOX_ID_ENABLE && !send0x300)
      continue;
    if (f.id == SBOX_ID_CMD && corruptCrc)
      f.data[3] ^= 0x5A;
    sbox_handle_rx(f.id, f.data, f.len, now);
  }
  sbox_task(now, uBatt, uOut);
  now += 10;
}

static void run(int opmode, uint32_t ms, int32_t uBatt, int32_t uOut) {
  for (uint32_t t = 0; t < ms; t += 10)
    zombie_tick(opmode, uBatt, uOut);
}

static void idle(uint32_t ms, int32_t uBatt, int32_t uOut) {
  for (uint32_t t = 0; t < ms; t += 10) {
    sbox_task(now, uBatt, uOut);
    now += 10;
  }
}

static void reset_box(void) {
  now = 1000;
  sbox_init(&boxCan2, Outputs);
  run(0, 50, 0, 0);
}

static void test_zombie_frames_are_what_we_expect(void) {
  zombieSide.Reset();
  SBOX::ControlContactors(2, &zombieSide);
  SBOX::ControlContactors(2, &zombieSide);
  const FakeCan::Frame *en = zombieSide.Find(SBOX_ID_ENABLE);
  const FakeCan::Frame *cmd = zombieSide.Find(SBOX_ID_CMD);
  CHECK(en != 0);
  CHECK(cmd != 0);
  if (en && cmd) {
    CHECK_EQ(en->data[1], SBOX_ENABLE_BYTE);
    CHECK_EQ(cmd->len, 4);
    CHECK_EQ(cmd->data[0], SBOX_CMD_PRECHARGE);
  }
}

static void test_normal_sequence(void) {
  reset_box();
  CHECK_EQ(sbox_state(), SBOX_OFF);
  CHECK(!outPre && !outPos && !outNeg);

  /* precharge: negative + precharge */
  run(2, 50, 350000, 0);
  CHECK_EQ(sbox_state(), SBOX_PRECHARGE);
  CHECK(outPre && !outPos && outNeg);

  /* RUN requested before the inverter side has charged: interlock holds */
  run(1, 300, 350000, 100000);
  CHECK_EQ(sbox_state(), SBOX_PRECHARGE);
  CHECK(outPre && !outPos && outNeg);

  /* charged up: positive closes, precharge overlaps briefly */
  run(1, 20, 350000, 340000);
  CHECK_EQ(sbox_state(), SBOX_RUN);
  CHECK(outPre && outPos && outNeg);
  run(1, PRECHG_OVERLAP_MS + 20, 350000, 349000);
  CHECK(!outPre && outPos && outNeg);

  /* charge mode uses the same command */
  run(4, 100, 350000, 349000);
  CHECK_EQ(sbox_state(), SBOX_RUN);

  /* off */
  run(0, 30, 350000, 349000);
  CHECK_EQ(sbox_state(), SBOX_OFF);
  CHECK(!outPre && !outPos && !outNeg);
}

static void test_precharge_fail(void) {
  reset_box();
  run(2, 100, 350000, 50000);
  CHECK_EQ(sbox_state(), SBOX_PRECHARGE);
  run(3, 30, 350000, 50000); /* MOD_PCHFAIL */
  CHECK_EQ(sbox_state(), SBOX_OFF);
  CHECK(!outPre && !outPos && !outNeg);
}

static void test_run_from_off_goes_through_precharge(void) {
  reset_box();
  run(1, 40, 350000, 350000);
  CHECK_EQ(sbox_state(), SBOX_PRECHARGE);
  CHECK(outPre && !outPos && outNeg);
  run(1, PRECHG_MIN_MS + 30, 350000, 350000);
  CHECK_EQ(sbox_state(), SBOX_RUN);
}

static void test_no_voltage_never_closes_positive(void) {
  reset_box();
  run(2, 50, 0, 0);
  run(1, 500, 0, 0);
  CHECK_EQ(sbox_state(), SBOX_PRECHARGE);
  CHECK(!outPos);
}

static void test_command_timeout(void) {
  reset_box();
  run(2, 50, 350000, 0);
  run(1, 200, 350000, 345000);
  CHECK_EQ(sbox_state(), SBOX_RUN);

  idle(CMD_TIMEOUT_MS - 50, 350000, 345000);
  CHECK_EQ(sbox_state(), SBOX_RUN);
  idle(100, 350000, 345000);
  CHECK_EQ(sbox_state(), SBOX_OFF);
  CHECK(!outPre && !outPos && !outNeg);
}

static void test_bad_crc_is_ignored(void) {
  reset_box();
  uint32_t errs = sbox_crc_errors();
  for (int i = 0; i < 30; i++)
    zombie_tick(2, 350000, 0, true, true);
  CHECK(sbox_crc_errors() >= errs + 30);
  CHECK_EQ(sbox_state(), SBOX_OFF);
}

static void test_missing_enable_frame(void) {
  reset_box();
  run(2, 50, 350000, 0);
  CHECK_EQ(sbox_state(), SBOX_PRECHARGE);
  for (int i = 0; i < (CMD_TIMEOUT_MS / 10) + 10; i++)
    zombie_tick(2, 350000, 0, false);
  CHECK_EQ(sbox_state(), SBOX_OFF);
}

static void decode_box_frames(void) {
  for (int i = 0; i < boxCan2.count; i++) {
    uint32_t d[2];
    memcpy(d, boxCan2.frames[i].data, 8);
    SBOX::DecodeCAN(boxCan2.frames[i].id, d);
  }
}

static void test_zombie_decodes_our_frames(void) {
  boxCan2.Reset();
  sbox_send(-123456, 355000, 354321);
  CHECK_EQ(boxCan2.count, 3);
  decode_box_frames();
  CHECK_EQ(SBOX::Amperes, -123456 * SBOX_CURRENT_SIGN);
  CHECK_EQ(SBOX::Voltage, 355000);
  CHECK_EQ(SBOX::Voltage2, 354321);

  boxCan2.Reset();
  sbox_send(98765, 700000, -5);
  decode_box_frames();
  CHECK_EQ(SBOX::Amperes, 98765 * SBOX_CURRENT_SIGN);
  CHECK_EQ(SBOX::Voltage, SBOX_MAX_MV);
  CHECK_EQ(SBOX::Voltage2, 0);
}

/* ------------------------------------------------------------- MEB --- */

static uint16_t cellMv(int i) { return (uint16_t)(3700 + i); }

static void feed_meb(uint32_t t) {
  for (uint32_t id = 0x1C0; id <= 0x1DF; id++) {
    if ((id & 3) == 3)
      continue;
    int group = (int)(((id - 0x1C0 + 1) * 3) / 4);
    uint32_t c0 = cellMv(group * 4 + 0) - 1000;
    uint32_t c1 = cellMv(group * 4 + 1) - 1000;
    uint32_t c2 = cellMv(group * 4 + 2) - 1000;
    uint32_t c3 = cellMv(group * 4 + 3) - 1000;
    uint32_t d0 = (c0 << 12) | ((c1 & 0xFF) << 24);
    uint32_t d1 = (c1 >> 8) | (c2 << 4) | (c3 << 16);
    uint8_t b[8];
    for (int k = 0; k < 4; k++) {
      b[k] = (uint8_t)(d0 >> (8 * k));
      b[4 + k] = (uint8_t)(d1 >> (8 * k));
    }
    meb_handle_rx(id, b, 8, t);
  }
  for (int cmu = 0; cmu < NUM_CMUS; cmu++) {
    uint32_t raw = (uint32_t)((20 + cmu + 40) * 2); /* 20..27 degC */
    uint32_t d1 = raw << 4;
    uint8_t b[8] = {0, 0, 0, 0, (uint8_t)d1, (uint8_t)(d1 >> 8), 0, 0};
    meb_handle_rx(0x1A5555F4u + cmu, b, 8, t);
  }
}

static void put_isa(uint32_t id, int32_t v, uint32_t t) {
  uint8_t b[8] = {0,
                  0,
                  (uint8_t)v,
                  (uint8_t)(v >> 8),
                  (uint8_t)(v >> 16),
                  (uint8_t)(v >> 24),
                  0,
                  0};
  soc_handle_rx(id, b, 8, t);
}

static void test_meb_and_simpbms(void) {
  meb_init(&boxCan1);
  soc_init();
  simpbms_init(&boxCan2);

  CHECK(!meb_alive(1000));
  feed_meb(1000);
  CHECK(meb_alive(1000));
  CHECK(!meb_alive(1000 + CMU_TIMEOUT_MS + 1));

  meb_task(1000);
  const MEB_STATS *st = meb_stats();
  CHECK_EQ((int)(st->minCellMv + 0.5f), 3700);
  CHECK_EQ((int)(st->maxCellMv + 0.5f), 3795);
  CHECK_EQ((int)(st->packMv + 0.5f), 96 * 3700 + 95 * 96 / 2);
  CHECK_EQ((int)st->lowTempC, 20);
  CHECK_EQ((int)st->highTempC, 27);
  CHECK(st->chargeLimitA > 0.0f);
  CHECK(st->dischargeLimitA > 0.0f);

  boxCan1.Reset();
  meb_balance(1);
  CHECK_EQ(boxCan1.count, 2);
  CHECK_EQ(boxCan1.frames[0].id, 0x1A555412u);
  CHECK_EQ(boxCan1.frames[1].id, 0x1A555413u);

  soc_task(1000, 1, st);
  CHECK(soc_get() > 0.0f && soc_get() < 100.0f);

  /* SimpBMS frames, decoded the way the Zombie's simpbms.cpp does */
  boxCan2.Reset();
  simpbms_send(1000);
  const FakeCan::Frame *f373 = boxCan2.Find(0x373);
  const FakeCan::Frame *f351 = boxCan2.Find(0x351);
  const FakeCan::Frame *f356 = boxCan2.Find(0x356);
  const FakeCan::Frame *f355 = boxCan2.Find(0x355);
  CHECK(f373 && f351 && f356 && f355);
  if (f373 && f351 && f356 && f355) {
    const uint8_t *d = f373->data;
    CHECK_EQ(d[0] | (d[1] << 8), 3700);
    CHECK_EQ(d[2] | (d[3] << 8), 3795);
    CHECK_EQ((d[4] | (d[5] << 8)) - 273, 20);
    CHECK_EQ((d[6] | (d[7] << 8)) - 273, 27);
    d = f351->data;
    CHECK_EQ(d[0] | (d[1] << 8), NUM_CELLS * CELL_VMAX_MV / 100);
    CHECK_EQ(d[2] | (d[3] << 8),
             (int)(meb_stats()->chargeLimitA * 10.0f + 0.5f));
    d = f356->data;
    CHECK_EQ(d[0] | (d[1] << 8), (96 * 3700 + 95 * 96 / 2 + 5) / 10);
    CHECK_EQ((int16_t)(d[2] | (d[3] << 8)), 0);
    d = f355->data;
    CHECK_EQ(d[0] | (d[1] << 8), (int)(soc_get() + 0.5f));
  }

  /* stale CMU data: SimpBMS goes quiet so the Zombie times out */
  boxCan2.Reset();
  simpbms_send(1000 + CMU_TIMEOUT_MS + 1);
  CHECK_EQ(boxCan2.count, 0);
}

static void test_isa_decode(void) {
  soc_init();
  put_isa(SHUNT_ID_CURRENT, -2500, 2000);
  put_isa(SHUNT_ID_UOUT, 351234, 2000);
  put_isa(SHUNT_ID_UBATT, 360000, 2000);
  CHECK(soc_shunt_ok(2000));
  CHECK_EQ(soc_raw_ma(2000), -2500);
  CHECK_EQ((int)(soc_current_a(2000) * 1000.0f), SHUNT_SIGN * -2500);
  CHECK_EQ(soc_out_mv(2000), 351234);
  CHECK_EQ(soc_batt_mv(2000), 360000);
  CHECK_EQ((int)soc_pack_mv(2000, 1.0f), SHUNT_USE_UBATT ? 360000 : 1);

  /* stale shunt data reads as zero */
  uint32_t late = 2000 + SHUNT_TIMEOUT_MS + 1;
  CHECK(!soc_shunt_ok(late));
  CHECK_EQ(soc_raw_ma(late), 0);
  CHECK_EQ(soc_out_mv(late), 0);
  CHECK_EQ(soc_batt_mv(late), 0);
}

int main(void) {
  test_zombie_frames_are_what_we_expect();
  test_normal_sequence();
  test_precharge_fail();
  test_run_from_off_goes_through_precharge();
  test_no_voltage_never_closes_positive();
  test_command_timeout();
  test_bad_crc_is_ignored();
  test_missing_enable_frame();
  test_zombie_decodes_our_frames();
  test_meb_and_simpbms();
  test_isa_decode();

  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
