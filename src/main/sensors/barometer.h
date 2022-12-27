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

#include "pg/pg.h"
#include "drivers/barometer/barometer.h"

typedef enum {
    BARO_DEFAULT = 0,
    BARO_NONE = 1,
    BARO_BMP085 = 2,
    BARO_MS5611 = 3,
    BARO_BMP280 = 4,
    BARO_LPS = 5,
    BARO_QMP6988 = 6,
    BARO_BMP388 = 7,
    BARO_DPS310 = 8,
    BARO_2SMPB_02B = 9,
} baroSensor_e;

typedef struct barometerConfig_s {
    uint8_t baro_busType;
    uint8_t baro_spi_device;
    ioTag_t baro_spi_csn;                   // Also used as XCLR (positive logic) for BMP085
    uint8_t baro_i2c_device;
    uint8_t baro_i2c_address;
    uint8_t baro_hardware;                  // Barometer hardware to use
    ioTag_t baro_eoc_tag;
    ioTag_t baro_xclr_tag;
} barometerConfig_t;

PG_DECLARE(barometerConfig_t, barometerConfig);

#define TASK_BARO_RATE_HZ 40                // Will be overwritten by the baro device driver

typedef struct baro_s {
    baroDev_t dev;
    float altitude;
    int32_t temperature;                    // Use temperature for telemetry
    int32_t pressure;                       // Use pressure for telemetry
} baro_t;

extern baro_t baro;

void baroPreInit(void);
void baroInit(void);
bool baroIsCalibrated(void);
void baroStartCalibration(void);
void baroSetGroundLevel(void);
uint32_t baroUpdate(timeUs_t currentTimeUs);
bool isBaroReady(void);
float getBaroAltitude(void);
