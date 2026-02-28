/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef USE_WING

#include <stdint.h>

#include "pg/pg.h"

typedef enum {
    POSHOLD_SOURCE_AUTO = 0,
    POSHOLD_SOURCE_GPS_ONLY,
    POSHOLD_SOURCE_OPTICALFLOW_ONLY
} posHoldSource_e;

typedef struct posHoldConfig_s {
    bool posHoldWithoutMag;
    uint8_t deadband;
    uint8_t positionSource;              // Position source selection
    uint8_t opticalflowQualityMin;       // Minimum optical flow quality threshold
    uint16_t opticalflowMaxRange;        // Maximum altitude for optical flow (cm)
} posHoldConfig_t;

PG_DECLARE(posHoldConfig_t, posHoldConfig);

#endif // !USE_WING
