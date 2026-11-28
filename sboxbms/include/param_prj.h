/*
 * This file is part of the ZombieVerter project.
 *
 * Parameters and spot values of the Zombie-hardware S-Box + SimpBMS
 * firmware. Tuning lives in bms_config.h as #defines; this list is only
 * what the web interface needs to show, plus the one-shot ISA setup switch.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#define VER 0.1.0SB

/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
// Next param id (increase when adding new parameter!): 2
// Next value Id: 2027
/*              category     name         unit       min     max     default id
 */
#define PARAM_LIST                                                             \
  PARAM_ENTRY(CAT_SETUP, IsaInit, ONOFF, 0, 1, 0, 1)                           \
  VALUE_ENTRY(version, VERSTR, 2000)                                           \
  VALUE_ENTRY(lasterr, errorListString, 2001)                                  \
  VALUE_ENTRY(boxstate, BOXSTATES, 2002)                                       \
  VALUE_ENTRY(cmdbyte, "dig", 2003)                                            \
  VALUE_ENTRY(cmdvalid, ONOFF, 2004)                                           \
  VALUE_ENTRY(crcerrors, "", 2005)                                             \
  VALUE_ENTRY(prechg_out, ONOFF, 2006)                                         \
  VALUE_ENTRY(pos_out, ONOFF, 2007)                                            \
  VALUE_ENTRY(neg_out, ONOFF, 2008)                                            \
  VALUE_ENTRY(udc_out, "V", 2009)                                              \
  VALUE_ENTRY(udc_batt, "V", 2010)                                             \
  VALUE_ENTRY(idc, "A", 2011)                                                  \
  VALUE_ENTRY(shuntok, ONOFF, 2012)                                            \
  VALUE_ENTRY(soc, "%", 2013)                                                  \
  VALUE_ENTRY(umin, "mV", 2014)                                                \
  VALUE_ENTRY(umax, "mV", 2015)                                                \
  VALUE_ENTRY(udelta, "mV", 2016)                                              \
  VALUE_ENTRY(packv, "V", 2017)                                                \
  VALUE_ENTRY(tmin, "°C", 2018)                                                \
  VALUE_ENTRY(tmax, "°C", 2019)                                                \
  VALUE_ENTRY(chglim, "A", 2020)                                               \
  VALUE_ENTRY(dischglim, "A", 2021)                                            \
  VALUE_ENTRY(cmusok, ONOFF, 2022)                                             \
  VALUE_ENTRY(balancing, "", 2023)                                             \
  VALUE_ENTRY(zombieok, ONOFF, 2024)                                           \
  VALUE_ENTRY(uaux, "V", 2025)                                                 \
  VALUE_ENTRY(cpuload, "%", 2026)

// Generate the version string. This must be formatted without spaces to meet
// the format requirements of the "units".
// clang-format off
#define VERSTR STRINGIFY(4=VER)
// clang-format on
#define ONOFF "0=Off, 1=On, 2=na"
#define BOXSTATES "0=Off, 1=Precharge, 2=Run"
#define CAT_SETUP "General Setup"

extern const char *errorListString;
