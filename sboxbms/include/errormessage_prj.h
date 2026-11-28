/*
 * This file is part of the ZombieVerter project.
 *
 * Error list of the Zombie-hardware S-Box + SimpBMS firmware.
 * All entries are for display only: the contactors follow the real Zombie.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef ERRORMESSAGE_PRJ_H_INCLUDED
#define ERRORMESSAGE_PRJ_H_INCLUDED

#define ERROR_BUF_SIZE 8

#define ERROR_MESSAGE_LIST                                                     \
  ERROR_MESSAGE_ENTRY(CMDTIMEOUT, ERROR_DISPLAY)                               \
  ERROR_MESSAGE_ENTRY(CMDCRC, ERROR_DISPLAY)                                   \
  ERROR_MESSAGE_ENTRY(BMSCOMM, ERROR_DISPLAY)                                  \
  ERROR_MESSAGE_ENTRY(SHUNTTIMEOUT, ERROR_DISPLAY)

#endif // ERRORMESSAGE_PRJ_H_INCLUDED
