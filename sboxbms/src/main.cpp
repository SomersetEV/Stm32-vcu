/*
 * This file is part of the ZombieVerter project.
 *
 * ZombieVerter hardware as a BMW S-Box + SimpBMS for another ZombieVerter.
 *
 *  CAN1 (PA11/PA12): VW MEB module CMUs (cells, temperatures, balancing)
 *                    and the ISA shunt (current, U1, U2, As).
 *  CAN2 (PB5/PB6):   the real ZombieVerter. It sends S-Box contactor
 *                    commands (0x100, 0x300); we answer with S-Box current
 *                    and voltages (0x200, 0x210, 0x220) and SimpBMS frames
 *                    (0x351, 0x355, 0x356, 0x373).
 *  Outputs:          precharge, main positive and main negative contactors.
 *
 * The BMS logic is ported from the STM32-canbridge meb-simpbms branch.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "anain.h"
#include "bms_config.h"
#include "canmap.h"
#include "cansdo.h"
#include "digio.h"
#include "errormessage.h"
#include "isa_shunt.h"
#include "meb_bms.h"
#include "param_save.h"
#include "params.h"
#include "power.h"
#include "sbox_emu.h"
#include "sdocommands.h"
#include "simpbms_tx.h"
#include "soc.h"
#include "stm32_can.h"
#include "stm32scheduler.h"
#include "sysclk.h"
#include "terminal.h"
#include "terminalcommands.h"
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#define PRINT_JSON 0
#define UAUX_GAIN 210 /* same hard-coded gain as the VCU firmware */

static Stm32Scheduler *scheduler;
static Stm32Can *can1;
static Stm32Can *can2;
static CanMap *canMap;
static volatile bool isaInitPending;

/* ------------------------------------------------------------- CAN RX --- */

/* libopeninv callbacks don't say which bus a frame came from, so each bus
 * gets its own callback object */
class BusRx : public CanCallback {
public:
  explicit BusRx(int b) : bus(b) {}

  void HandleRx(uint32_t id, uint32_t data[2], uint8_t dlc) override {
    uint32_t now = millis();
    const uint8_t *bytes = (const uint8_t *)data;

    if (bus == 0) {
      if (meb_is_id(id))
        meb_handle_rx(id, bytes, dlc, now);
      else if (soc_is_shunt_id(id))
        soc_handle_rx(id, bytes, dlc, now);
    } else {
      if (sbox_is_id(id)) {
        sbox_handle_rx(id, bytes, dlc, now);
        power_note_zombie_rx(now);
      } else {
        power_note_other_rx(now);
      }
    }
  }

  /* called whenever the bus's filter list is cleared (e.g. "can clear") */
  void HandleClear() override {
    if (bus == 0) {
      meb_register_filters(can1);
      soc_register_filters(can1);
    } else {
      sbox_register_filters(can2);
    }
  }

private:
  int bus;
};

/* ---------------------------------------------------------- contactors --- */

static void SetContactors(bool precharge, bool positive, bool negative) {
  if (precharge)
    DigIo::CONT_PRECHARGE.Set();
  else
    DigIo::CONT_PRECHARGE.Clear();

  if (positive)
    DigIo::CONT_POSITIVE.Set();
  else
    DigIo::CONT_POSITIVE.Clear();

  if (negative)
    DigIo::CONT_NEGATIVE.Set();
  else
    DigIo::CONT_NEGATIVE.Clear();
}

static void AllOutputsOff(void) {
  DigIo::dcsw_out.Clear();
  DigIo::prec_out.Clear();
  DigIo::inv_out.Clear();
  DigIo::gp_out1.Clear();
  DigIo::gp_out2.Clear();
  DigIo::gp_out3.Clear();
  DigIo::SL1_out.Clear();
  DigIo::SL2_out.Clear();
  DigIo::SP_out.Clear();
  DigIo::req_out.Clear();
  DigIo::PWM1.Clear();
  DigIo::PWM2.Clear();
  DigIo::PWM3.Clear();
  DigIo::lin_nslp.Clear(); /* LIN transceiver asleep */
  DigIo::mcp_sby.Set();    /* CAN3 (MCP25625) in standby: not used */
}

/* -------------------------------------------------------------- tasks --- */

/* Battery side: ISA if present, else the sum of the cells */
static int32_t BatteryMv(uint32_t now) {
  int32_t mv = soc_batt_mv(now);

  if (mv <= 0 && meb_alive(now))
    mv = (int32_t)meb_stats()->packMv;
  return mv;
}

static void Ms10Task(void) {
  static uint8_t txTick = 0;
  uint32_t now = millis();
  int32_t uBatt = BatteryMv(now);
  int32_t uOut = soc_out_mv(now); /* 0 without ISA: Zombie precharge fails */

  sbox_task(now, uBatt, uOut);

  if (++txTick >= SBOX_TX_PERIOD_MS / 10) {
    txTick = 0;
    /* only talk while the Zombie does, so we never keep it awake */
    if (power_zombie_active(now))
      sbox_send(soc_raw_ma(now), uBatt, uOut);
  }
}

static void PostErrors(uint32_t now, uint8_t alive) {
  static bool wasCmdValid = false, wasAlive = false, wasShunt = false;
  static uint32_t lastCrcErrors = 0;
  bool cmdValid = sbox_cmd_valid(now);
  bool shunt = soc_shunt_ok(now);

  ErrorMessage::SetTime(rtc_get_counter_val() + 1);

  if (wasCmdValid && !cmdValid)
    ErrorMessage::Post(ERR_CMDTIMEOUT);
  if (sbox_crc_errors() != lastCrcErrors)
    ErrorMessage::Post(ERR_CMDCRC);
  if (wasAlive && !alive)
    ErrorMessage::Post(ERR_BMSCOMM);
  if (wasShunt && !shunt)
    ErrorMessage::Post(ERR_SHUNTTIMEOUT);

  wasCmdValid = cmdValid;
  wasAlive = alive;
  wasShunt = shunt;
  lastCrcErrors = sbox_crc_errors();
}

static void UpdateSpotValues(uint32_t now, uint8_t alive) {
  const MEB_STATS *st = meb_stats();

  Param::SetInt(Param::boxstate, sbox_state());
  Param::SetInt(Param::cmdbyte, sbox_cmd_byte());
  Param::SetInt(Param::cmdvalid, sbox_cmd_valid(now));
  Param::SetInt(Param::crcerrors, sbox_crc_errors());
  Param::SetInt(Param::prechg_out, sbox_out_precharge());
  Param::SetInt(Param::pos_out, sbox_out_positive());
  Param::SetInt(Param::neg_out, sbox_out_negative());
  Param::SetFloat(Param::udc_out, soc_out_mv(now) / 1000.0f);
  Param::SetFloat(Param::udc_batt, BatteryMv(now) / 1000.0f);
  Param::SetFloat(Param::idc, soc_current_a(now));
  Param::SetInt(Param::shuntok, soc_shunt_ok(now));
  Param::SetFloat(Param::soc, soc_get());
  Param::SetFloat(Param::umin, st->minCellMv);
  Param::SetFloat(Param::umax, st->maxCellMv);
  Param::SetFloat(Param::udelta, st->maxCellMv - st->minCellMv);
  Param::SetFloat(Param::packv, st->packMv / 1000.0f);
  Param::SetFloat(Param::tmin, st->lowTempC);
  Param::SetFloat(Param::tmax, st->highTempC);
  Param::SetFloat(Param::chglim, st->chargeLimitA);
  Param::SetFloat(Param::dischglim, st->dischargeLimitA);
  Param::SetInt(Param::cmusok, alive);
  Param::SetInt(Param::balancing, meb_balancing_count());
  Param::SetInt(Param::zombieok, power_zombie_active(now));
  Param::SetFloat(Param::uaux, ((float)AnaIn::uaux.Get()) / UAUX_GAIN);
  Param::SetFloat(Param::cpuload, scheduler->GetCpuLoad() / 10.0f);
  Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());
}

static void Ms100Task(void) {
  static uint8_t ledTick = 0;
  uint32_t now = millis();
  uint8_t alive;

  iwdg_reset();

  meb_task(now);
  alive = meb_alive(now);
  soc_task(now, alive, meb_stats());
  meb_balance(BALANCE_ENABLE && alive);

  if (power_zombie_active(now)) {
    simpbms_send(now);
    canMap->SendAll();
  }

  PostErrors(now, alive);
  UpdateSpotValues(now, alive);

  /* slow blink: alive; fast blink: contactors closed */
  if (++ledTick >= (sbox_state() == SBOX_OFF ? 5 : 1)) {
    ledTick = 0;
    DigIo::led_out.Toggle();
  }
}

extern "C" void tim4_isr(void) { scheduler->Run(); }

/* ----------------------------------------------------------- params ----- */

void Param::Change(Param::PARAM_NUM paramNum) {
  /* may be called from the CAN ISR (SDO), so only flag work for the loop */
  if ((paramNum == Param::IsaInit || paramNum == Param::PARAM_LAST) &&
      Param::GetInt(Param::IsaInit) == 1)
    isaInitPending = true;
}

/* Configure the ISA exactly as the ZombieVerter does (little-endian, cyclic
 * 100 ms). Runs from the main loop; it blocks for a few seconds. */
static void RunIsaInit(void) {
  isaInitPending = false;
  iwdg_set_period_ms(IWDG_SLEEP_MS);
  iwdg_reset();
  ISA::initialize(can1);
  iwdg_set_period_ms(IWDG_RUN_MS);
  iwdg_reset();
  Param::SetInt(Param::IsaInit, 0);
}

/* --------------------------------------------------------------- main --- */

int main(void) {
  extern const TERM_CMD TermCmds[];
  char c = 0;

  sysclk_setup();
  rtc_lsi_setup();

  ANA_IN_CONFIGURE(ANA_IN_LIST);
  DIG_IO_CONFIGURE(DIG_IO_LIST);
  AllOutputsOff();
  AnaIn::Start();

  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN2_REMAP);
  nvic_setup();

  /* the bootloader normally starts it already; make the period explicit */
  iwdg_set_period_ms(IWDG_RUN_MS);
  iwdg_start();

  parm_load();
  Param::Change(Param::PARAM_LAST);
  systick_setup();

  Terminal t(USART3, TermCmds, false, true, true);
  Stm32Can c1(CAN1, CanHardware::Baud500);
  Stm32Can c2(CAN2, CanHardware::Baud500, true);
  can1 = &c1;
  can2 = &c2;

  /* don't load a CAN map left in flash by the VCU firmware */
  CanMap cm(&c2, false);
  CanSdo sdo(&c2, &cm);
  canMap = &cm;
  TerminalCommands::SetCanMap(&cm);
  SdoCommands::SetCanMap(&cm);

  meb_init(&c1);
  soc_init();
  simpbms_init(&c2);
  sbox_init(&c2, SetContactors);

  BusRx rx1(0), rx2(1);
  c1.AddCallback(&rx1);
  c2.AddCallback(&rx2);
  sdo.SetNodeId(SDO_NODE_ID); /* clears CAN2 filters -> rx2.HandleClear() */
  rx1.HandleClear();

  power_init(millis());

  Stm32Scheduler s(TIM4); // We never exit main so it's ok to put it on stack
  scheduler = &s;
  s.AddTask(Ms10Task, 10);
  s.AddTask(Ms100Task, 100);

  Param::SetInt(Param::version, 4); // shows VERSTR

  while (1) {
    t.Run();
    if (sdo.GetPrintRequest() == PRINT_JSON) {
      TerminalCommands::PrintParamsJson(&sdo, &c);
    }
    if (isaInitPending)
      RunIsaInit();
    power_task(millis());
  }

  return 0;
}
