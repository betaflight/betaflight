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

#include "common/time.h"

#define TASK_ALTITUDE_RATE_HZ 100

typedef enum {
    ALTITUDE_SOURCE_DEFAULT = 0,
    ALTITUDE_SOURCE_BARO_ONLY,
    ALTITUDE_SOURCE_GPS_ONLY,
    ALTITUDE_SOURCE_RANGEFINDER_PREFER,
    ALTITUDE_SOURCE_RANGEFINDER_ONLY
} altitudeSource_e;

typedef struct positionConfig_s {
    uint8_t altitude_source;
    uint8_t altitude_prefer_baro;
    uint16_t altitude_lpf;                // lowpass cutoff (value / 100) Hz for altitude smoothing
    uint16_t altitude_d_lpf;              // lowpass for (value / 100) Hz for altitude derivative smoothing
    uint16_t rangefinder_max_range_cm;    // Maximum rangefinder range to trust (cm)
} positionConfig_t;

PG_DECLARE(positionConfig_t, positionConfig);

float getAltitudeCm(void);
float getAltitudeDerivative(void);
void calculateEstimatedAltitude(void);
void positionInit(void);
int32_t getEstimatedAltitudeCm(void);
float getAltitudeAsl(void);
int16_t getEstimatedVario(void);
bool isAltitudeAvailable(void);

