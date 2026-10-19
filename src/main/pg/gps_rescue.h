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

#include "pg/gps_rescue_multirotor.h"
#include "pg/gps_rescue_wing.h"

#define GPS_RESCUE_MIN_START_DIST_MIN_M 5
#ifdef USE_WING
#define GPS_RESCUE_MIN_START_DIST_MAX_M 1000
#else
#define GPS_RESCUE_MIN_START_DIST_MAX_M 30
#endif
#define GPS_RESCUE_INITIAL_CLIMB_MAX_M 100
#define GPS_RESCUE_ASCEND_RATE_MIN 50
#define GPS_RESCUE_ASCEND_RATE_MAX 2500
#define GPS_RESCUE_RETURN_ALT_MIN_M 5
#define GPS_RESCUE_RETURN_ALT_MAX_M 1000
#define GPS_RESCUE_DESCEND_RATE_MIN 25
#define GPS_RESCUE_DESCEND_RATE_MAX 500
#define GPS_RESCUE_MIN_SATS_MIN 5
#define GPS_RESCUE_MIN_SATS_MAX 50

typedef enum {
    GPS_RESCUE_ALT_MODE_MAX = 0,
    GPS_RESCUE_ALT_MODE_FIXED,
    GPS_RESCUE_ALT_MODE_CURRENT,
    GPS_RESCUE_ALT_MODE_COUNT
} gpsRescueAltitudeMode_e;
