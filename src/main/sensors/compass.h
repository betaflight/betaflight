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

#include "sensors/sensors.h"

// Type of magnetometer used/detected
typedef enum {
    MAG_DEFAULT = 0,
    MAG_NONE = 1,
    MAG_HMC5883 = 2,
    MAG_AK8975 = 3,
    MAG_GPS = 4,
    MAG_MAG3110 = 5,
    MAG_AK8963 = 6,
    MAG_FAKE = 7,
} magSensor_e;

#define MAG_MAX  MAG_FAKE

bool compassInit(int16_t magDeclinationFromConfig);
union flightDynamicsTrims_u;
void compassUpdate(timeUs_t currentTimeUs, union flightDynamicsTrims_u *magZero);
bool isCompassReady(void);

extern int32_t magADC[XYZ_AXIS_COUNT];

extern sensor_align_e magAlign;
extern float magneticDeclination;
