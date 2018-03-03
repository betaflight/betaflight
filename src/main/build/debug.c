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

// Debug values
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

// Optional timing information
#ifdef DEBUG_SECTION_TIMES
uint32_t sectionTimes[2][4];
#endif

// Debug modes used in the CMS menus
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

#ifdef SEGGER_RTT
// Debug mask and levels
static uint64_t dbgMsk = DBG_MSK(DBG_SYSTEM) | DBG_MSK(DBG_INIT) | DBG_MSK(DBG_RX);
static uint8_t  dbgLvl = 3;

// Initialise debug output stream
void dbgInit()
{
#if (RTT_DEBUG_CHANNEL == 0)
    SEGGER_RTT_ConfigUpBuffer(RTT_DEBUG_CHANNEL, "RTT Debug", NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
#else
    static char dbgBuf[BUFFER_SIZE_UP];
    SEGGER_RTT_ConfigUpBuffer(RTT_DEBUG_CHANNEL, "RTT Debug", dbgBuf, BUFFER_SIZE_UP, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
#endif
}

// Dynamically set debug level above which no debug will be emitted
// The higher the debug level, the more debug will be seen
void dbgLevel(uint8_t dbgLevel)
{
    dbgLvl = dbgLevel;
}

// Set the debug mask to dynamically select which debug streams will be seen
void dbgMask(uint64_t dbgMask)
{
    dbgMsk = dbgMask;
}

// Emit debug information from a given source with a given level
int dbgPrintf(dbgSrc_e src, uint8_t lvl, const char *fmt, ...)
{
    va_list args;
    int len = 0;

    if ((DBG_MSK(src) & dbgMsk) && (lvl <= dbgLvl)) {
        SEGGER_RTT_printf(RTT_DEBUG_CHANNEL, "%d:%d ", (int)src, lvl);
        va_start (args, fmt);
        len = SEGGER_RTT_vprintf(RTT_DEBUG_CHANNEL, fmt, &args);
        va_end (args);
    }

    return len;
}
#endif /* SEGGER_RTT */

