/*
 * Sleep when the real ZombieVerter has not been heard on CAN2 for
 * SLEEP_TIMEOUT_MS and the contactors are open.
 *
 * Port of the STM32-canbridge power.c to libopencm3. The ISA shunt is on
 * CAN1 here, so CAN2 normally goes quiet when the Zombie sleeps. Other
 * permanently powered nodes may still share CAN2, so every wake starts with
 * a short "sniff": clocks up, CAN2 only, listen for WAKE_SNIFF_MS. Only a
 * Zombie frame (0x100/0x300) commits to a full wake; otherwise we go straight
 * back to STOP with cell and SoC state untouched.
 *
 *  - edge mode: CAN2 quiet, wake on the first dominant edge on CAN2 RX (PB5).
 *    The RTC alarm (LSI) wakes us every RTC_WAKE_S just to feed the IWDG,
 *    which the bootloader starts and which keeps running in STOP.
 *  - poll mode: other nodes chattering, edge wake off, sniff every POLL_S.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "power.h"
#include "bms_config.h"
#include "meb_bms.h"
#include "sbox_emu.h"
#include "soc.h"
#include "sysclk.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/timer.h>

#define WAKE_EXTI EXTI5 /* PB5 = CAN2_RX (remapped) */
#define WAKE_PORT GPIOB
#define WAKE_IRQ NVIC_EXTI9_5_IRQ

#define CAN_TME_ALL (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)
#define CAN_ABRQ_ALL (CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2)

static uint32_t lastZombieRx;
static uint8_t zombieSeen;
static uint32_t lastOtherRx;
static uint8_t otherSeen;
static volatile uint8_t wakeByBus;

/* set by the note functions while sniffing */
static volatile uint8_t sniffZombie;
static volatile uint8_t sniffOther;

typedef enum { SNIFF_QUIET, SNIFF_OTHER_ONLY, SNIFF_ZOMBIE } SNIFF_RESULT;

/* ---------------------------------------------------------------- RTC --- */

static void rtc_alarm_arm(uint32_t seconds) {
  rtc_set_alarm_time(rtc_get_counter_val() + seconds);
  rtc_clear_flag(RTC_ALR);
  exti_reset_request(EXTI17);
  rtc_interrupt_enable(RTC_ALR);
}

static void rtc_alarm_disarm(void) {
  rtc_interrupt_disable(RTC_ALR);
  rtc_clear_flag(RTC_ALR);
  exti_reset_request(EXTI17);
}

extern "C" void rtc_alarm_isr(void) {
  rtc_clear_flag(RTC_ALR);
  exti_reset_request(EXTI17);
}

/* ----------------------------------------------------------- bus wake --- */

static void wake_pin_arm(void) {
  /* the pin stays a CAN input; EXTI only watches it */
  exti_select_source(WAKE_EXTI, WAKE_PORT);
  exti_set_trigger(WAKE_EXTI, EXTI_TRIGGER_FALLING);
  exti_reset_request(WAKE_EXTI);
  wakeByBus = 0;
  exti_enable_request(WAKE_EXTI);
  nvic_set_priority(WAKE_IRQ, 0x10);
  nvic_enable_irq(WAKE_IRQ);
}

static void wake_pin_disarm(void) {
  nvic_disable_irq(WAKE_IRQ);
  exti_disable_request(WAKE_EXTI);
  exti_reset_request(WAKE_EXTI);
}

extern "C" void exti9_5_isr(void) {
  if (exti_get_flag_status(WAKE_EXTI)) {
    exti_reset_request(WAKE_EXTI);
    wakeByBus = 1;
  }
}

/* ---------------------------------------------------------------- CAN --- */

static void can_enter_init(uint32_t can) {
  uint32_t t0 = millis();

  CAN_MCR(can) |= CAN_MCR_INRQ;
  while (!(CAN_MSR(can) & CAN_MSR_INAK) && (millis() - t0) < 10) {
  }
}

static void can_leave_init(uint32_t can) {
  uint32_t t0 = millis();

  CAN_MCR(can) &= ~CAN_MCR_INRQ;
  /* needs 11 recessive bits on the bus; don't hang on a stuck bus */
  while ((CAN_MSR(can) & CAN_MSR_INAK) && (millis() - t0) < 10) {
  }
}

static bool tx_idle(uint32_t can) {
  return (CAN_TSR(can) & CAN_TME_ALL) == CAN_TME_ALL;
}

static void flush_tx(void) {
  uint32_t t0 = millis();
  uint32_t idleSince = 0;
  bool idle = false;

  /* the TX ISR refills the mailboxes from the driver's send buffer, so wait
   * until they have stayed empty for a couple of ms */
  while ((millis() - t0) < 100) {
    iwdg_reset();
    if (tx_idle(CAN1) && tx_idle(CAN2)) {
      if (!idle) {
        idle = true;
        idleSince = millis();
      } else if ((millis() - idleSince) >= 3) {
        break;
      }
    } else {
      idle = false;
    }
  }

  /* nobody acks on CAN2 once the Zombie sleeps: drop what is left */
  CAN_TSR(CAN1) |= CAN_ABRQ_ALL;
  CAN_TSR(CAN2) |= CAN_ABRQ_ALL;
}

/* --------------------------------------------------------------- sleep --- */

static void enter_stop(void) {
  pwr_set_stop_mode();
  pwr_voltage_regulator_low_power_in_stop();
  SCB_SCR |= SCB_SCR_SLEEPDEEP;
  __asm__ volatile("wfi");
  SCB_SCR &= ~SCB_SCR_SLEEPDEEP;
}

/* listen on CAN2 only; the caller has restored the clocks */
static SNIFF_RESULT sniff(void) {
  uint32_t t0;

  sniffZombie = 0;
  sniffOther = 0;

  can_leave_init(CAN2);

  t0 = millis();
  while ((millis() - t0) < WAKE_SNIFF_MS) {
    iwdg_reset();
    if (sniffZombie)
      return SNIFF_ZOMBIE;
  }
  return sniffOther ? SNIFF_OTHER_ONLY : SNIFF_QUIET;
}

static void power_sleep(void) {
  uint32_t sleepStart, sleptS;
  uint8_t pollMode;
  SNIFF_RESULT r;

  /* stop the 10/100 ms tasks so nothing transmits or closes a contactor */
  timer_disable_counter(TIM4);

  sbox_force_off();
  meb_balance_all_off(); /* make sure no module is left bleeding */
  flush_tx();

  can_enter_init(CAN1);
  can_enter_init(CAN2);

  iwdg_set_period_ms(IWDG_SLEEP_MS);
  iwdg_reset();

  /* other CAN2 nodes heard recently: their frames would keep waking us */
  pollMode = otherSeen && (millis() - lastOtherRx) <= SLEEP_TIMEOUT_MS;
  sleepStart = rtc_get_counter_val();

  for (;;) {
    if (!pollMode)
      wake_pin_arm();

    rtc_alarm_arm(pollMode ? POLL_S : RTC_WAKE_S);
    systick_counter_disable();
    enter_stop();
    /* running on HSI here; enough to feed the dog and go back to sleep */
    iwdg_reset();

    if (!pollMode) {
      if (!wakeByBus)
        continue; /* watchdog alarm only */
      wake_pin_disarm();
    }

    rtc_alarm_disarm();
    sysclk_restore();
    systick_counter_enable();

    r = sniff();
    if (r == SNIFF_ZOMBIE)
      break;

    can_enter_init(CAN2);
    if (r == SNIFF_OTHER_ONLY)
      pollMode = 1;
  }

  rtc_alarm_disarm();
  sleptS = rtc_get_counter_val() - sleepStart;

  iwdg_set_period_ms(IWDG_RUN_MS);
  iwdg_reset();

  /* CAN2 is already running from the sniff */
  can_leave_init(CAN1);

  /* cell data is stale after any sleep; SoC only re-anchors after a long one */
  meb_reset();
  soc_note_wake(sleptS >= LONG_SLEEP_S);
  lastZombieRx = millis();
  otherSeen = 0;

  timer_enable_counter(TIM4);
}

void power_init(uint32_t now) {
  lastZombieRx = now;
  zombieSeen = 0;
  otherSeen = 0;
}

void power_note_zombie_rx(uint32_t now) {
  lastZombieRx = now;
  zombieSeen = 1;
  sniffZombie = 1;
}

void power_note_other_rx(uint32_t now) {
  lastOtherRx = now;
  otherSeen = 1;
  sniffOther = 1;
}

bool power_zombie_active(uint32_t now) {
  return zombieSeen && (now - lastZombieRx) <= ZOMBIE_ACTIVE_MS;
}

void power_task(uint32_t now) {
#if SLEEP_ENABLE
  if ((now - lastZombieRx) > SLEEP_TIMEOUT_MS && sbox_state() == SBOX_OFF) {
    power_sleep();
  }
#else
  (void)now;
#endif
}
