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

#include "common/time.h"
#include "drivers/pitot/pitot.h"
#include "pg/pg.h"

typedef enum {
    PITOT_DEFAULT = 0,
    PITOT_NONE = 1,
    PITOT_MS4525 = 2,
    PITOT_DRONECAN = 3,
    PITOT_HARDWARE_COUNT
} pitotSensor_e;

typedef struct pitotConfig_s {
    uint8_t pitot_busType;
    uint8_t pitot_i2c_device;
    uint8_t pitot_i2c_address;
    uint8_t pitot_hardware;
} pitotConfig_t;

PG_DECLARE(pitotConfig_t, pitotConfig);

#ifndef TASK_PITOT_RATE_HZ
#define TASK_PITOT_RATE_HZ 20   // oversamples the differential-pressure LPF; avoids a 100 Hz blocking I2C read
#endif

typedef struct pitot_s {
    pitotDev_t dev;
    float airspeed;             // cm/s, indicated airspeed
    float diffPressure;         // Pa, zero-corrected differential pressure
    float temperature;          // kelvin
    float pressureZero;         // Pa, at-rest offset captured by calibration
} pitot_t;

extern pitot_t pitot;

void pitotInit(void);
uint32_t pitotUpdate(timeUs_t currentTimeUs);
bool pitotIsConfigured(void);
bool pitotIsCalibrated(void);
void pitotStartCalibration(void);
float pitotGetAirspeed(void);
