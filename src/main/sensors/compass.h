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
#include "drivers/io_types.h"
#include "drivers/sensor.h"
#include "pg/pg.h"
#include "sensors/sensors.h"


// Type of magnetometer used/detected
typedef enum {
    MAG_DEFAULT = 0,
    MAG_NONE = 1,
    MAG_HMC5883 = 2,
    MAG_AK8975 = 3,
    MAG_AK8963 = 4,
    MAG_QMC5883 = 5
} magSensor_e;

typedef struct mag_s {
    float magADC[XYZ_AXIS_COUNT];
    float magneticDeclination;
} mag_t;

extern mag_t mag;

typedef struct compassConfig_s {
    int16_t mag_declination;                // Get your magnetic decliniation from here : http://magnetic-declination.com/
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
    sensor_align_e mag_align;               // mag alignment
    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
    uint8_t mag_bustype;
    uint8_t mag_i2c_device;
    uint8_t mag_i2c_address;
    uint8_t mag_spi_device;
    ioTag_t mag_spi_csn;
    ioTag_t interruptTag;
    flightDynamicsTrims_t magZero;
} compassConfig_t;

PG_DECLARE(compassConfig_t, compassConfig);

bool compassIsHealthy(void);
void compassUpdate(timeUs_t currentTime);
bool compassInit(void);
