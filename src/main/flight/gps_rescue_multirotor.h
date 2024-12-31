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

#ifndef USE_WING

#include <stdbool.h>

#include "common/axis.h"

#include "pg/gps_rescue.h"

#define TASK_GPS_RESCUE_RATE_HZ 100  // in sync with altitude task rate

typedef enum {
    RESCUE_SANITY_OFF = 0,
    RESCUE_SANITY_ON,
    RESCUE_SANITY_FS_ONLY,
    RESCUE_SANITY_COUNT
} gpsRescueSanity_e;

typedef enum {
    GPS_RESCUE_ALT_MODE_MAX = 0,
    GPS_RESCUE_ALT_MODE_FIXED,
    GPS_RESCUE_ALT_MODE_CURRENT,
    GPS_RESCUE_ALT_MODE_COUNT
} gpsRescueAltitudeMode_e;

void gpsRescueInit(void);
void gpsRescueUpdate(void);
float gpsRescueGetYawRate(void);
bool gpsRescueIsConfigured(void);
bool gpsRescueIsAvailable(void);
bool gpsRescueIsOK(void);
bool gpsRescueDisableMag(void);

#endif // !USE_WING
