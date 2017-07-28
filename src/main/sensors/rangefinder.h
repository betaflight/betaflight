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

#include <stdint.h>
#include "config/parameter_group.h"
#include "drivers/rangefinder.h"

typedef enum {
    RANGEFINDER_NONE        = 0,
    RANGEFINDER_HCSR04      = 1,
    RANGEFINDER_SRF10       = 2,
    RANGEFINDER_HCSR04I2C   = 3,
    RANGEFINDER_VL53L0X     = 4,
} rangefinderType_e;

typedef struct rangefinderConfig_s {
    uint8_t rangefinder_hardware;
} rangefinderConfig_t;

PG_DECLARE(rangefinderConfig_t, rangefinderConfig);

typedef struct rangefinder_s {
    rangefinderDev_t dev;
    float maxTiltCos;
    int32_t rawAltitude;
    int32_t calculatedAltitude;
    timeMs_t lastValidResponseTimeMs;
    
    bool snrThresholdReached;
    int32_t dynamicDistanceThreshold;
    int16_t snr;
} rangefinder_t;

extern rangefinder_t rangefinder;

const rangefinderHardwarePins_t * rangefinderGetHardwarePins(void);

void rangefinderResetDynamicThreshold(void);
bool rangefinderInit(void);

int32_t rangefinderGetLatestAltitude(void);
int32_t rangefinderGetLatestRawAltitude(void);

timeDelta_t rangefinderUpdate(void);
bool rangefinderProcess(float cosTiltAngle);
bool rangefinderIsHealthy(void);
