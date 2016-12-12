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

#include "drivers/barometer.h"

typedef enum {
    BARO_DEFAULT = 0,
    BARO_NONE = 1,
    BARO_BMP085 = 2,
    BARO_MS5611 = 3,
    BARO_BMP280 = 4
} baroSensor_e;

#define BARO_SAMPLE_COUNT_MAX   48

typedef struct barometerConfig_s {
    uint8_t baro_hardware;                  // Barometer hardware to use
    uint8_t baro_sample_count;              // size of baro filter array
    float baro_noise_lpf;                   // additional LPF to reduce baro noise
    float baro_cf_vel;                      // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
    float baro_cf_alt;                      // apply CF to use ACC for height estimation
} barometerConfig_t;

typedef struct baro_s {
    baroDev_t dev;
    int32_t BaroAlt;
    int32_t baroTemperature;             // Use temperature for telemetry
} baro_t;

extern baro_t baro;

bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse);
void useBarometerConfig(const barometerConfig_t *barometerConfigToUse);
bool isBaroCalibrationComplete(void);
void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
uint32_t baroUpdate(void);
bool isBaroReady(void);
int32_t baroCalculateAltitude(void);
void performBaroCalibrationCycle(void);
