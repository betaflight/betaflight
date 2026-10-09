/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef USE_WING

#include <stdint.h>

#include "pg/pg.h"

typedef struct gpsRescue_s {
    uint16_t minStartDistM;         // m
    uint8_t  altitudeMode;          // gpsRescueAltitudeMode_e
    uint16_t initialClimbM;         // m
    uint16_t ascendRate;            // cm/s
    uint16_t returnAltitudeM;       // m
    uint16_t descendRate;           // cm/s
    uint8_t  minSats;
    uint8_t  allowArmingWithoutFix;
} gpsRescueConfig_t;

PG_DECLARE(gpsRescueConfig_t, gpsRescueConfig);

#endif // USE_WING
