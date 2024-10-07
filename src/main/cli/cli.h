/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

extern bool cliMode;

typedef enum dumpFlags_e {
    DUMP_MASTER = (1 << 0),
    DUMP_PROFILE = (1 << 1),
    DUMP_RATES = (1 << 2),
    DUMP_ALL = (1 << 3),
    DO_DIFF = (1 << 4),
    SHOW_DEFAULTS = (1 << 5),
    HIDE_UNUSED = (1 << 6),
    HARDWARE_ONLY = (1 << 7),
    BARE = (1 << 8),
} dumpFlags_t;

bool cliProcess(void);
struct serialPort_s;
void cliEnter(struct serialPort_s *serialPort, bool interactive);

#ifdef USE_CLI_DEBUG_PRINT
void cliPrint(const char *str);
void cliPrintLinefeed(void);
void cliPrintLine(const char *str);
void cliPrintf(const char *format, ...);
void cliPrintLinef(const char *format, ...);
#endif
