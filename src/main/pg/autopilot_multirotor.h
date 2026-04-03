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

typedef struct autopilotConfig_s {
    uint8_t landingAltitudeM;   // altitude below which landing behaviours can change, metres
    uint16_t hoverThrottle;      // value used at the start of a rescue or position hold
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint8_t altitudeP;
    uint8_t altitudeI;
    uint8_t altitudeD;
    uint8_t altitudeF;
    uint8_t positionP;
    uint8_t positionI;
    uint8_t positionII;
    uint8_t positionD;
    uint8_t positionA;
    uint8_t positionCutoff;
    uint8_t maxAngle;
} autopilotConfig_t;

PG_DECLARE(autopilotConfig_t, autopilotConfig);

#endif // !USE_WING
