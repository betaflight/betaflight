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

// NOTE: This gyro is considered obsolete and may be removed in the future.

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_GYRO_MPU3050

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus_i2c.h"
#include "drivers/exti.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu3050.h"

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
    delay(25); // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe

    const bool ack = busWriteRegister(&gyro->bus, MPU3050_SMPLRT_DIV, 0);
    if (!ack) {
        failureMode(FAILURE_ACC_INIT);
    }

    busWriteRegister(&gyro->bus, MPU3050_DLPF_FS_SYNC, MPU3050_FS_SEL_2000DPS | MPU3050_DLPF_256HZ);
    busWriteRegister(&gyro->bus, MPU3050_INT_CFG, 0);
    busWriteRegister(&gyro->bus, MPU3050_USER_CTRL, MPU3050_USER_RESET);
    busWriteRegister(&gyro->bus, MPU3050_PWR_MGM, MPU3050_CLK_SEL_PLL_GX);
}

static bool mpu3050GyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&gyro->bus, MPU3050_GYRO_OUT, data, 6);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

static bool mpu3050ReadTemperature(gyroDev_t *gyro, int16_t *tempData)
{
    uint8_t buf[2];
    if (!busReadRegisterBuffer(&gyro->bus, MPU3050_TEMP_OUT, buf, 2)) {
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
    gyro->readFn = mpu3050GyroRead;
    gyro->temperatureFn = mpu3050ReadTemperature;

    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
#endif
