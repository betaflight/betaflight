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


#include <stdint.h>
//#include <stdbool.h>

#include "pg/pg.h"
#include "drivers/io_types.h"
#include "drivers/sensor.h"
#include "sensors/gyro.h"

typedef struct gyroDeviceConfig_s {
    int8_t index;
    uint8_t bustype;
    uint8_t spiBus;
    ioTag_t csnTag;
    uint8_t i2cBus;
    uint8_t i2cAddress;
    ioTag_t extiTag;
    sensor_align_e align;
} gyroDeviceConfig_t;

#ifdef USE_MULTI_GYRO
#define MAX_GYRODEV_COUNT 2
#else
#define MAX_GYRODEV_COUNT 1
#endif

PG_DECLARE_ARRAY(gyroDeviceConfig_t, MAX_GYRODEV_COUNT, gyroDeviceConfig);
