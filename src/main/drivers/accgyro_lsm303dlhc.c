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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"

#include "system.h"
#include "gpio.h"
#include "bus_i2c.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_lsm303dlhc.h"

// Addresses (7 bit address format)

#define LSM303DLHC_ACCEL_ADDRESS 0x19
#define LSM303DLHC_MAG_ADDRESS 0x1E

// Registers

#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define OUT_X_L_A 0x28
#define CRA_REG_M 0x00
#define CRB_REG_M 0x01
#define MR_REG_M 0x02
#define OUT_X_H_M 0x03

///////////////////////////////////////

#define ODR_1344_HZ 0x90
#define AXES_ENABLE 0x07

#define FULLSCALE_2G 0x00
#define FULLSCALE_4G 0x10
#define FULLSCALE_8G 0x20
#define FULLSCALE_16G 0x30

#define BOOT 0x80

///////////////////////////////////////

#define ODR_75_HZ 0x18
#define ODR_15_HZ 0x10

#define FS_1P3_GA 0x20
#define FS_1P9_GA 0x40
#define FS_2P5_GA 0x60
#define FS_4P0_GA 0x80
#define FS_4P7_GA 0xA0
#define FS_5P6_GA 0xC0
#define FS_8P1_GA 0xE0

#define CONTINUOUS_CONVERSION 0x00

uint8_t accelCalibrating = false;

float accelOneG = 9.8065;

int32_t accelSum100Hz[3] = { 0, 0, 0 };

int32_t accelSum500Hz[3] = { 0, 0, 0 };

int32_t accelSummedSamples100Hz[3];

int32_t accelSummedSamples500Hz[3];

void lsm303dlhcAccInit(void)
{
    i2cWrite(LSM303DLHC_ACCEL_ADDRESS, CTRL_REG5_A, BOOT);

    delay(100);

    i2cWrite(LSM303DLHC_ACCEL_ADDRESS, CTRL_REG1_A, ODR_1344_HZ | AXES_ENABLE);

    delay(10);

    i2cWrite(LSM303DLHC_ACCEL_ADDRESS, CTRL_REG4_A, FULLSCALE_4G);

    delay(100);

    acc_1G = 512 * 8;
}

// Read 3 gyro values into user-provided buffer. No overrun checking is done.
static void lsm303dlhcAccRead(int16_t *gyroADC)
{
    uint8_t buf[6];

    bool ok = i2cRead(LSM303DLHC_ACCEL_ADDRESS, OUT_X_L_A, 6, buf);

    if (!ok)
        return;

    // the values range from -8192 to +8191
    gyroADC[X] = (int16_t)((buf[1] << 8) | buf[0]) / 2;
    gyroADC[Y] = (int16_t)((buf[3] << 8) | buf[2]) / 2;
    gyroADC[Z] = (int16_t)((buf[5] << 8) | buf[4]) / 2;

#if 0
    debug[0] = (int16_t)((buf[1] << 8) | buf[0]);
    debug[1] = (int16_t)((buf[3] << 8) | buf[2]);
    debug[2] = (int16_t)((buf[5] << 8) | buf[4]);
#endif
}

bool lsm303dlhcAccDetect(acc_t *acc)
{
    bool ack;
    uint8_t status;

    ack = i2cRead(LSM303DLHC_ACCEL_ADDRESS, LSM303DLHC_STATUS_REG_A, 1, &status);
    if (!ack)
        return false;

    acc->init = lsm303dlhcAccInit;
    acc->read = lsm303dlhcAccRead;
    return true;
}

