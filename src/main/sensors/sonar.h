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

#include "drivers/sonar_hcsr04.h"
#include "drivers/rangefinder.h"
#include "sensors/battery.h"

typedef enum {
    SONAR_NONE = 0,
    SONAR_HCSR04,
    SONAR_SRF10
} sonarHardwareType_e;

typedef enum {
    SONAR_HCSR04_PINS_RC,
    SONAR_HCSR04_PINS_PWM,
} sonarHCSR04Pins_e;

struct sonarRange_s;
typedef void (*sonarInitFunctionPtr)(struct sonarRange_s *sonarRange);
typedef void (*sonarUpdateFunctionPtr)(void);
typedef int32_t (*sonarReadFunctionPtr)(void);

typedef struct sonarFunctionPointers_s {
    sonarInitFunctionPtr init;
    sonarUpdateFunctionPtr update;
    sonarReadFunctionPtr read;
} sonarFunctionPointers_t;

const sonarHcsr04Hardware_t *sonarGetHardwareConfiguration(currentSensor_e currentSensor);
int32_t sonarCalculateAltitude(int32_t sonarDistance, float cosTiltAngle);
int32_t sonarGetLatestAltitude(void);
void sonarInit(void);
void sonarUpdate(void);
int32_t sonarRead(void);

