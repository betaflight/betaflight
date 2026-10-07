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

#ifndef USE_WING

#include <stdint.h>

#include "pg/pg.h"

#define GPS_RESCUE_MIN_START_DIST_MIN_M 5
#define GPS_RESCUE_MIN_START_DIST_MAX_M 30
#define GPS_RESCUE_INITIAL_CLIMB_MAX_M 100
#define GPS_RESCUE_ASCEND_RATE_MIN 50
#define GPS_RESCUE_ASCEND_RATE_MAX 2500
#define GPS_RESCUE_RETURN_ALT_MIN_M 5
#define GPS_RESCUE_RETURN_ALT_MAX_M 1000
#define GPS_RESCUE_DESCEND_RATE_MIN 25
#define GPS_RESCUE_DESCEND_RATE_MAX 500

typedef struct gpsRescue_s {

    uint16_t minStartDistM; // meters
    uint8_t  altitudeMode;
    uint16_t initialClimbM; // meters
    uint16_t ascendRate;

    uint16_t returnAltitudeM; // meters
    uint16_t groundSpeedCmS; // centimeters per second

    uint16_t descentDistanceM; // meters
    uint16_t descendRate;
    uint8_t  sanityChecks;
    uint8_t  minSats;
    uint8_t  allowArmingWithoutFix;

    uint8_t  yawP;
} gpsRescueConfig_t;

PG_DECLARE(gpsRescueConfig_t, gpsRescueConfig);

#endif // !USE_WING
