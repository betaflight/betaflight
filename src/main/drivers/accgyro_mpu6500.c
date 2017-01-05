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
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "exti.h"
#include "gpio.h"
#include "gyro_sync.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6500.h"

void mpu6500AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

bool mpu6500AccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    acc->init = mpu6500AccInit;
    acc->read = mpuAccRead;

    return true;
}

void mpu6500GyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    gyro->mpuConfiguration.write(MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
    delay(100);
    gyro->mpuConfiguration.write(MPU_RA_SIGNAL_PATH_RESET, 0x07);
    delay(100);
    gyro->mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0);
    delay(100);
    gyro->mpuConfiguration.write(MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    const uint8_t raGyroConfigData = gyro->gyroRateKHz > GYRO_RATE_8_kHz ? (INV_FSR_2000DPS << 3 | FCB_3600_32) : (INV_FSR_2000DPS << 3 | FCB_DISABLED);
    gyro->mpuConfiguration.write(MPU_RA_GYRO_CONFIG, raGyroConfigData);
    delay(15);
    gyro->mpuConfiguration.write(MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delay(15);
    gyro->mpuConfiguration.write(MPU_RA_CONFIG, gyro->lpf);
    delay(15);
    gyro->mpuConfiguration.write(MPU_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops(gyro)); // Get Divider Drops
    delay(100);

    // Data ready interrupt configuration
#ifdef USE_MPU9250_MAG
    gyro->mpuConfiguration.write(MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR | MPU6500_BIT_BYPASS_EN);  // INT_ANYRD_2CLEAR, BYPASS_EN
#else
    gyro->mpuConfiguration.write(MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR);  // INT_ANYRD_2CLEAR
#endif
    delay(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    gyro->mpuConfiguration.write(MPU_RA_INT_ENABLE, MPU6500_BIT_RAW_RDY_EN); // RAW_RDY_EN interrupt enable
#endif
    delay(15);
}

bool mpu6500GyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    gyro->init = mpu6500GyroInit;
    gyro->read = mpuGyroRead;
    gyro->intStatus = mpuCheckDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}
