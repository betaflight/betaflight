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
    BARO_NONE = 0,
    BARO_AUTODETECT = 1,
    BARO_BMP085 = 2,
    BARO_MS5611 = 3,
    BARO_BMP280 = 4,
    BARO_FAKE = 5,
    BARO_MAX = BARO_FAKE
} baroSensor_e;

typedef struct barometerConfig_s {
    uint8_t baro_hardware;              // Barometer hardware to use
    uint8_t use_median_filtering;       // Use 3-point median filtering
} barometerConfig_t;

typedef struct baro_s {
    baroDev_t dev;
    int32_t BaroAlt;
    int32_t baroTemperature;            // Use temperature for telemetry
} baro_t;

extern baro_t baro;

bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse);
void useBarometerConfig(barometerConfig_t *barometerConfigToUse);
bool isBaroCalibrationComplete(void);
void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
uint32_t baroUpdate(void);
bool isBaroReady(void);
int32_t baroCalculateAltitude(void);
bool isBarometerHealthy(void);
