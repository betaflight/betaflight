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

#include "common/time.h"

#include "drivers/io.h"

#define RANGEFINDER_OUT_OF_RANGE        (-1)
#define RANGEFINDER_HARDWARE_FAILURE    (-2)
#define RANGEFINDER_NO_NEW_DATA         (-3)

struct rangefinderDev_s;

typedef struct rangefinderHardwarePins_s {
    ioTag_t triggerTag;
    ioTag_t echoTag;
} rangefinderHardwarePins_t;

typedef void (*rangefinderOpInitFuncPtr)(void);
typedef void (*rangefinderOpStartFuncPtr)(void);
typedef int32_t (*rangefinderOpReadFuncPtr)(void);

typedef struct rangefinderDev_s {
    timeMs_t delayMs;
    int16_t maxRangeCm;

    // these are full detection cone angles, maximum tilt is half of this
    int16_t detectionConeDeciDegrees; // detection cone angle as in device spec
    int16_t detectionConeExtendedDeciDegrees; // device spec is conservative, in practice have slightly larger detection cone

    // function pointers
    rangefinderOpInitFuncPtr init;
    rangefinderOpStartFuncPtr update;
    rangefinderOpReadFuncPtr read;
} rangefinderDev_t;
