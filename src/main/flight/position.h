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

typedef struct positionConfig_s {
    uint8_t altitude_source;
    uint8_t altitude_prefer_baro;
    uint16_t altitude_lpf;                // lowpass cutoff (value / 100) Hz for altitude smoothing
    uint16_t altitude_d_lpf;              // lowpass for (value / 100) Hz for altitude derivative smoothing
    uint16_t hover_throttle;              // value used at the start of a rescue or position hold
    uint8_t landing_altitude_m;           // altitude below which landing behaviours can change, metres
} positionConfig_t;

PG_DECLARE(positionConfig_t, positionConfig);

typedef struct altitudeData_s {
    float altitudeCm;
    float altitudeDerivativeCmS;
} altitudeData_t;
void getAltitudeData(altitudeData_t* data);

void calculateEstimatedAltitude(void);
void positionInit(void);
int32_t getEstimatedAltitudeCm(void);
bool isAltitudeLow(void);
float getAltitudeAsl(void);
int16_t getEstimatedVario(void);
bool isAltitudeAvailable(void);

