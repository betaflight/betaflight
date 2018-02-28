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

#pragma once

#include <stdarg.h>

// Debug values
#define DEBUG16_VALUE_COUNT 4
extern int16_t debug[DEBUG16_VALUE_COUNT];
extern uint8_t debugMode;

#define DEBUG_SET(mode, index, value) {if (debugMode == (mode)) {debug[(index)] = (value);}}

#define DEBUG_SECTION_TIMES

// Optional timing information capture
#ifdef DEBUG_SECTION_TIMES
extern uint32_t sectionTimes[2][4];

#define TIME_SECTION_BEGIN(index) { \
    extern uint32_t sectionTimes[2][4]; \
    sectionTimes[0][index] = micros(); \
}

#define TIME_SECTION_END(index) { \
    extern uint32_t sectionTimes[2][4]; \
    sectionTimes[1][index] = micros(); \
    debug[index] = sectionTimes[1][index] - sectionTimes[0][index]; \
}
#else

#define TIME_SECTION_BEGIN(index) {}
#define TIME_SECTION_END(index) {}

#endif

// Debug modes used in the CMS menus
typedef enum {
    DEBUG_NONE,
    DEBUG_CYCLETIME,
    DEBUG_BATTERY,
    DEBUG_GYRO,
    DEBUG_ACCELEROMETER,
    DEBUG_PIDLOOP,
    DEBUG_GYRO_NOTCH,
    DEBUG_RC_INTERPOLATION,
    DEBUG_ANGLERATE,
    DEBUG_ESC_SENSOR,
    DEBUG_SCHEDULER,
    DEBUG_STACK,
    DEBUG_ESC_SENSOR_RPM,
    DEBUG_ESC_SENSOR_TMP,
    DEBUG_ALTITUDE,
    DEBUG_FFT,
    DEBUG_FFT_TIME,
    DEBUG_FFT_FREQ,
    DEBUG_RX_FRSKY_SPI,
    DEBUG_GYRO_RAW,
    DEBUG_MAX7456_SIGNAL,
    DEBUG_MAX7456_SPICLOCK,
    DEBUG_SBUS,
    DEBUG_FPORT,
    DEBUG_RANGEFINDER,
    DEBUG_RANGEFINDER_QUALITY,
    DEBUG_LIDAR_TF,
    DEBUG_CORE_TEMP,
    DEBUG_RUNAWAY_TAKEOFF,
    DEBUG_COUNT
} debugType_e;

extern const char * const debugModeNames[DEBUG_COUNT];

// Source of debug information
typedef enum {
    DBG_INIT,
    DBG_SYSTEM,
    DBG_RX,
    DBG_LED,
    DBG_MOTOR
} dbgSrc_e;

#ifdef SEGGER_RTT
#include "SEGGER_RTT.h"
extern int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList);

// Set RTT_DEBUG_CHANNEL to 0 to use the default channel and on the host use 'telnet localhost 19021'
// Set RTT_DEBUG_CHANNEL to 1-3 to use alternate channels and JLinkRTTLogger to capture debug output to a log file
#define RTT_DEBUG_CHANNEL 0

#define DBG_MSK(src) (1<<src)

extern void dbgInit();
extern void dbgLevel(uint8_t dbgLevel);
extern void dbgMask(uint64_t dbgMask);
extern int dbgPrintf(dbgSrc_e src, uint8_t lvl, const char *fmt, ...);
#else /* SEGGER_RTT */
// If the debugger isn't being used then define all debug calls to be empty to save code space
#define dbgInit()
#define dbgLevel(dbgLevel)
#define dbgMask(dbgMask)
#define dbgPrintf(...)
#endif /* SEGGER_RTT */

