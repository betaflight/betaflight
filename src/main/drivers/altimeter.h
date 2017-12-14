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

typedef struct altimeterVTable_s {
    void (*startReading)(void);
    int32_t (*getDistance)(void);
} altimeterVTable_t;

typedef struct altimeterRange_s {
    int16_t maxRangeCm;
    // these are full detection cone angles, maximum tilt is half of this
    int16_t detectionConeDeciDegrees; // detection cone angle
    int16_t detectionConeExtendedDeciDegrees; // pragmatic value
} altimeterRange_t;

typedef struct altimeterDevice_s {
    const altimeterRange_t *range;
    const altimeterVTable_t *vTable;
} altimeterDevice_t;


#define ALTIMETER_OUT_OF_RANGE (-1)
