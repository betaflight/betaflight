/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>

#include "platform.h"

#include "debug.h"

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

#ifdef DEBUG_SECTION_TIMES
uint32_t sectionTimes[2][4];
#endif

const char * const debugModeNames[DEBUG_COUNT] = {
    "NONE",
    "CYCLETIME",
    "BATTERY",
    "GYRO",
    "ACCELEROMETER",
    "PIDLOOP",
    "NOTCH",
    "RC_INTERPOLATION",
    "ANGLERATE",
    "ESC_SENSOR",
    "SCHEDULER",
    "STACK",
    "ESC_SENSOR_RPM",
    "ESC_SENSOR_TMP",
    "ALTITUDE",
    "FFT",
    "FFT_TIME",
    "FFT_FREQ",
    "RX_FRSKY_SPI",
    "GYRO_RAW",
    "MAX7456_SIGNAL",
    "MAX7456_SPICLOCK",
    "SBUS",
    "FPORT",
    "RANGEFINDER",
    "RANGEFINDER_QUALITY",
    "LIDAR_TF",
    "CORE_TEMP",
    "RUNAWAY_TAKEOFF",
};

static uint64_t dbgMsk = DBG_MSK(DBG_SYSTEM) | DBG_MSK(DBG_INIT) | DBG_MSK(DBG_RX);
static uint8_t  dbgLvl = 3;

void dbgInit()
{
#ifdef SEGGER_RTT
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
#endif
}

void dbgLevel(uint8_t dbgLevel)
{
    dbgLvl = dbgLevel;
}

void dbgMask(uint8_t dbgMask)
{
    dbgMsk = dbgMask;
}

int dbgPrintf(dbgSrc_e src, uint8_t lvl, const char *fmt, ...)
{
    va_list args;
    int len = 0;

    if ((DBG_MSK(src) & dbgMsk) && (lvl <= dbgLvl)) {
        va_start (args, fmt);
#ifdef SEGGER_RTT
        len = SEGGER_RTT_vprintf(0, fmt, &args);
#endif /* SEGGER_RTT */
        va_end (args);
    }

    return len;
}

