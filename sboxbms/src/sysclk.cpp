/*
 * Clocks, 1 ms SysTick time base and LSI RTC for the S-Box/BMS firmware.
 *
 * The VCU firmware runs its RTC from HSE/128, which stops in STOP mode.
 * This firmware needs a clock that keeps running while it sleeps (to feed
 * the watchdog and to know how long it slept), so the RTC moves to LSI.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "sysclk.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>

/* nominal; the F1 LSI is only accurate to about +-50% */
#define LSI_HZ 40000u

static volatile uint32_t msTicks;

extern "C" void sys_tick_handler(void) { msTicks = msTicks + 1; }

uint32_t millis(void) { return msTicks; }

void sysclk_setup(void) {
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

  // The reset value for PRIGROUP (=0) is not actually a defined
  // value. Explicitly set 16 preemption priorities
  SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_GPIOD);
  rcc_periph_clock_enable(RCC_GPIOE);
  rcc_periph_clock_enable(RCC_USART3); // terminal / web interface
  rcc_periph_clock_enable(RCC_TIM4);   // scheduler
  rcc_periph_clock_enable(RCC_DMA1);   // ADC and terminal UART
  rcc_periph_clock_enable(RCC_ADC1);
  rcc_periph_clock_enable(RCC_CRC); // parameter and CAN map flash CRC
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_CAN1);
  rcc_periph_clock_enable(RCC_CAN2);
  rcc_periph_clock_enable(RCC_PWR);
  rcc_periph_clock_enable(RCC_BKP);
}

void sysclk_restore(void) {
  /* STOP mode wakes up on HSI with PLL and HSE off */
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void systick_setup(void) {
  systick_set_frequency(1000, rcc_ahb_frequency);
  systick_clear();
  systick_interrupt_enable();
  systick_counter_enable();
}

void rtc_lsi_setup(void) {
  pwr_disable_backup_domain_write_protect();

  rcc_osc_on(RCC_LSI);
  rcc_wait_for_osc_ready(RCC_LSI);

  /* RTCSEL (RCC_BDCR[9:8], 2 = LSI) can only be changed by resetting the
   * backup domain, e.g. when the VCU firmware left it on HSE/128 */
  if (((RCC_BDCR >> 8) & 3) != 2 || !(RCC_BDCR & RCC_BDCR_RTCEN)) {
    rcc_backupdomain_reset();
    pwr_disable_backup_domain_write_protect();
    rtc_awake_from_off(RCC_LSI);
  } else {
    rtc_awake_from_standby();
  }
  rtc_set_prescale_val(LSI_HZ - 1);

  /* RTC alarm -> EXTI line 17, used to wake from STOP */
  exti_set_trigger(EXTI17, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI17);
  rtc_clear_flag(RTC_ALR);
  exti_reset_request(EXTI17);
}

void nvic_setup(void) {
  nvic_enable_irq(NVIC_TIM4_IRQ);      // scheduler
  nvic_set_priority(NVIC_TIM4_IRQ, 0); // highest priority

  nvic_enable_irq(NVIC_RTC_ALARM_IRQ);
  nvic_set_priority(NVIC_RTC_ALARM_IRQ, 0x20);
}
