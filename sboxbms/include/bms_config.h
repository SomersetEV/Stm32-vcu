/*
 * This file is part of the ZombieVerter project.
 *
 * Tunables for the Zombie-hardware S-Box + SimpBMS firmware.
 * All voltages in mV, currents in A, temperatures in degC, times in ms
 * unless the name says otherwise.
 *
 * Ported from the STM32-canbridge meb-simpbms branch (source/Inc/bms_config.h).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

/* ---- Bus assignment ---------------------------------------------------- */
/* CAN1 (PA11/PA12): VW MEB module CMUs + ISA shunt                          */
/* CAN2 (PB5/PB6, remapped): the real ZombieVerter (S-Box + SimpBMS frames)  */

/* ---- Pack layout ------------------------------------------------------- */
#define NUM_CMUS 8
#define CELLS_PER_CMU 12
#define NUM_CELLS (NUM_CMUS * CELLS_PER_CMU)

/* ---- Cell limits ------------------------------------------------------- */
#define CUTOFF_MV 3000           /* discharge limit reaches 0 A here  */
#define CELL_VMAX_MV 4200        /* final charge taper set point      */
#define CHG_SP1_MV 3950          /* first charge taper set point      */
#define CHG_SP2_MV 4050          /* second charge taper set point     */
#define CHG_I1_A 275.0f          /* max current below SP1             */
#define CHG_I2_A 170.0f          /* max current below SP2             */
#define CHG_I3_A 114.0f          /* max current below VMAX            */
#define CHG_KP_A_PER_MV 2.0f     /* taper slope per stage             */
#define CHG_RAMP_A_PER_TICK 5.0f /* max rise of charge limit per 100 ms */
#define DCHG_MAX_A 500.0f
#define DCHG_A_PER_MV 5.0f
#define CHG_TMAX_C 51.0f  /* charge fully derated at this temp */
#define DCHG_TMAX_C 53.0f /* discharge fully derated here      */

/* ---- Balancing --------------------------------------------------------- */
#define BALANCE_ENABLE 1
#define BAL_MIN_MV 3730
#define BAL_HYST_MV 4

/* ---- Liveness / scheduling --------------------------------------------- */
#define CMU_TIMEOUT_MS 3000
#define TASK_PERIOD_MS 100

/* ---- SoC / ISA shunt (on CAN1) ----------------------------------------- */
#define MAX_AH 148.0f
/* little-endian int32 in bytes 2..5, as ZombieVerter configures the ISA   */
#define SHUNT_ID_CURRENT 0x521 /* mA                                   */
#define SHUNT_ID_U1 0x522      /* mV                                   */
#define SHUNT_ID_U2 0x523      /* mV                                   */
#define SHUNT_ID_AS 0x527      /* As                                   */
#define SHUNT_ID_FIRST 0x521
#define SHUNT_ID_LAST 0x528
/*
 * Which ISA voltage input is on which side of the contactors. The defaults
 * follow the ZombieVerter's own ISA convention: U1 = inverter side (udc,
 * used for the precharge check), U2 = battery side (udc2). Check your wiring.
 */
#define SHUNT_ID_UOUT SHUNT_ID_U1
#define SHUNT_ID_UBATT SHUNT_ID_U2
#define SHUNT_SIGN (1)    /* -1 if the shunt reads discharge as negative */
#define SHUNT_USE_UBATT 1 /* 0x356 pack voltage from the shunt when present */
#define SHUNT_TIMEOUT_MS 1000
#define REST_CURRENT_A 2.0f
#define REST_MS (30UL * 60UL * 1000UL)

/* ---- BMW S-Box emulation (on CAN2) ------------------------------------- */
#define SBOX_ID_CMD 0x100     /* from Zombie: contactor command + CRC  */
#define SBOX_ID_ENABLE 0x300  /* from Zombie: byte 1 = 0xFE enables     */
#define SBOX_ID_CURRENT 0x200 /* to Zombie: int24 LE mA                 */
#define SBOX_ID_UBATT 0x210   /* to Zombie: int24 LE mV, battery side   */
#define SBOX_ID_UOUT 0x220    /* to Zombie: int24 LE mV, inverter side  */
#define SBOX_CMD_OFF 0x00
#define SBOX_CMD_PRECHARGE 0xA6
#define SBOX_CMD_RUN 0xAA
#define SBOX_ENABLE_BYTE 0xFE
#define SBOX_CHECK_CRC 1
#define SBOX_TX_PERIOD_MS 20
#define SBOX_CURRENT_SIGN (1) /* sign applied to the raw ISA mA          */
#define SBOX_MAX_MV 599999    /* Zombie ignores >= 600000 mV             */
#define CMD_TIMEOUT_MS 500    /* no valid 0x100/0x300 -> all contactors open */

/* ---- Contactors -------------------------------------------------------- */
/*
 * Interlock, not a BMS trip: on a RUN command the positive contactor only
 * closes once precharge has been on for PRECHG_MIN_MS and the inverter-side
 * voltage is within PRECHG_DV_MV of a plausible battery voltage.
 */
#define PRECHG_MIN_MS 100
#define PRECHG_DV_MV 20000
#define PRECHG_MIN_BATT_MV 50000
#define PRECHG_OVERLAP_MS 100 /* precharge stays on this long after positive   \
                               */

/* Zombie board outputs (DigIo names from ../include/digio_prj.h)          */
#define CONT_PRECHARGE prec_out /* PC6  */
#define CONT_POSITIVE dcsw_out  /* PC7  */
#define CONT_NEGATIVE gp_out1   /* PD15 */

/* ---- Link to the real Zombie ------------------------------------------- */
/* CAN2 frames are only sent while the Zombie has been heard this recently,
 * so this box never keeps a sleeping Zombie awake.                         */
#define ZOMBIE_ACTIVE_MS 1000
#define SDO_NODE_ID 5 /* Zombie uses 3 */

/* ---- Sleep ------------------------------------------------------------- */
#define SLEEP_ENABLE 1
#define SLEEP_TIMEOUT_MS 10000 /* no Zombie frames for this long -> sleep */
#define RTC_WAKE_S 10          /* periodic wake to feed the IWDG          */
/*
 * After any wake we listen on CAN2 for WAKE_SNIFF_MS and only wake fully on
 * a Zombie frame. If other nodes keep CAN2 busy while the Zombie sleeps, the
 * edge wake is turned off and we sniff every POLL_S seconds instead.
 */
#define WAKE_SNIFF_MS 120
#define POLL_S 2
#define LONG_SLEEP_S (REST_MS / 1000UL) /* re-anchor SoC after this long */

/* ---- Watchdog ---------------------------------------------------------- */
#define IWDG_RUN_MS 2000
#define IWDG_SLEEP_MS 32000 /* clamps to the IWDG maximum */

#endif
