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

#include "common/maths.h"

#include "system.h"
#include "bus_i2c.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu3050.h"



// MPU3050, Standard address 0x68
#define MPU3050_ADDRESS         0x68

// Registers
#define MPU3050_SMPLRT_DIV      0x15
#define MPU3050_DLPF_FS_SYNC    0x16
#define MPU3050_INT_CFG         0x17
#define MPU3050_TEMP_OUT        0x1B
#define MPU3050_GYRO_OUT        0x1D
#define MPU3050_USER_CTRL       0x3D
#define MPU3050_PWR_MGM         0x3E

// Bits
#define MPU3050_FS_SEL_2000DPS  0x18
#define MPU3050_DLPF_10HZ       0x05
#define MPU3050_DLPF_20HZ       0x04
#define MPU3050_DLPF_42HZ       0x03
#define MPU3050_DLPF_98HZ       0x02
#define MPU3050_DLPF_188HZ      0x01
#define MPU3050_DLPF_256HZ      0x00

#define MPU3050_USER_RESET      0x01
#define MPU3050_CLK_SEL_PLL_GX  0x01

static uint8_t mpuLowPassFilter = MPU3050_DLPF_42HZ;

static void mpu3050Init(void);
static void mpu3050Read(int16_t *gyroADC);
static void mpu3050ReadTemp(int16_t *tempData);

bool mpu3050Detect(gyro_t *gyro, uint16_t lpf)
{
    bool ack;

    delay(25); // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe

    ack = i2cWrite(MPU3050_ADDRESS, MPU3050_SMPLRT_DIV, 0);
    if (!ack)
        return false;

    gyro->init = mpu3050Init;
    gyro->read = mpu3050Read;
    gyro->temperature = mpu3050ReadTemp;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    // default lpf is 42Hz
    switch (lpf) {
        case 256:
            mpuLowPassFilter = MPU3050_DLPF_256HZ;
            break;
        case 188:
            mpuLowPassFilter = MPU3050_DLPF_188HZ;
            break;
        case 98:
            mpuLowPassFilter = MPU3050_DLPF_98HZ;
            break;
        default:
        case 42:
            mpuLowPassFilter = MPU3050_DLPF_42HZ;
            break;
        case 20:
            mpuLowPassFilter = MPU3050_DLPF_20HZ;
            break;
        case 10:
            mpuLowPassFilter = MPU3050_DLPF_10HZ;
            break;
    }

    return true;
}

static void mpu3050Init(void)
{
    bool ack;

    delay(25); // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe

    ack = i2cWrite(MPU3050_ADDRESS, MPU3050_SMPLRT_DIV, 0);
    if (!ack)
        failureMode(3);

    i2cWrite(MPU3050_ADDRESS, MPU3050_DLPF_FS_SYNC, MPU3050_FS_SEL_2000DPS | mpuLowPassFilter);
    i2cWrite(MPU3050_ADDRESS, MPU3050_INT_CFG, 0);
    i2cWrite(MPU3050_ADDRESS, MPU3050_USER_CTRL, MPU3050_USER_RESET);
    i2cWrite(MPU3050_ADDRESS, MPU3050_PWR_MGM, MPU3050_CLK_SEL_PLL_GX);
}

// Read 3 gyro values into user-provided buffer. No overrun checking is done.
static void mpu3050Read(int16_t *gyroADC)
{
    uint8_t buf[6];

    i2cRead(MPU3050_ADDRESS, MPU3050_GYRO_OUT, 6, buf);

    gyroADC[0] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroADC[1] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroADC[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

static void mpu3050ReadTemp(int16_t *tempData)
{
    uint8_t buf[2];
    i2cRead(MPU3050_ADDRESS, MPU3050_TEMP_OUT, 2, buf);

    *tempData = 35 + ((int32_t)(buf[0] << 8 | buf[1]) + 13200) / 280;
}
