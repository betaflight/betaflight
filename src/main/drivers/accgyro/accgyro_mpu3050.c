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

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/exti.h"
#include "drivers/bus_i2c.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu3050.h"

// MPU3050, Standard address 0x68
#define MPU3050_ADDRESS         0x68

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

static void mpu3050Init(gyroDev_t *gyro)
{
    bool ack;

    delay(25); // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe

    ack = gyro->mpuConfiguration.writeFn(&gyro->bus, MPU3050_SMPLRT_DIV, 0);
    if (!ack)
        failureMode(FAILURE_ACC_INIT);

    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU3050_DLPF_FS_SYNC, MPU3050_FS_SEL_2000DPS | gyro->lpf);
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU3050_INT_CFG, 0);
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU3050_USER_CTRL, MPU3050_USER_RESET);
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU3050_PWR_MGM, MPU3050_CLK_SEL_PLL_GX);
}

static bool mpu3050ReadTemperature(gyroDev_t *gyro, int16_t *tempData)
{
    uint8_t buf[2];
    if (!gyro->mpuConfiguration.readFn(&gyro->bus, MPU3050_TEMP_OUT, 2, buf)) {
        return false;
    }

    *tempData = 35 + ((int32_t)(buf[0] << 8 | buf[1]) + 13200) / 280;

    return true;
}

bool mpu3050Detect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_3050) {
        return false;
    }
    gyro->initFn = mpu3050Init;
    gyro->readFn = mpuGyroRead;
    gyro->temperatureFn = mpu3050ReadTemperature;
    gyro->intStatusFn = mpuCheckDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}
