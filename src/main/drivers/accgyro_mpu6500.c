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

#include <platform.h>
#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

#include "system.h"
#include "exti.h"
#include "gpio.h"
#include "gyro_sync.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6500.h"

extern uint8_t mpuIntDenominator;

static bool mpu6500ReadTemp(int16_t *tempData)
{
    uint8_t buf[2];
    if (!mpuConfiguration.read(MPU_RA_TEMP_OUT_H, 2, buf)) {
        return false;
    }

    // MPU-6500-Datasheet2.pdf P12
    const int16_t RoomTemp_Offset = 21;
    const int16_t Temp_Sensitivity = 333;

    // RM-MPU-6500A-00.pdf P33
    *tempData = RoomTemp_Offset + ((int16_t)(buf[0] << 8 | buf[1]) - RoomTemp_Offset) / Temp_Sensitivity;

    return true;
}

bool mpu6500AccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    acc->init = mpu6500AccInit;
    acc->read = mpuAccRead;

    return true;
}

bool mpu6500GyroDetect(gyro_t *gyro)
{
    if (mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    gyro->init = mpu6500GyroInit;
    gyro->read = mpuGyroRead;
    gyro->temperature = mpu6500ReadTemp;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}

void mpu6500AccInit(acc_t *acc)
{
    mpuIntExtiInit();

    acc->acc_1G = 512 * 8;
}

void mpu6500GyroInit(gyro_t* gyro, uint8_t lpf)
{
    uint16_t intFrequencyHz;
    switch(lpf) {
        case 0:
            intFrequencyHz = 8000;
        break;
        default:
            intFrequencyHz = 1000;
        break;
    }

    mpuIntDenominator = intFrequencyHz / gyro->sampleFrequencyHz;

    // handle cases where the refresh period was set higher than the LPF allows
    if (mpuIntDenominator == 0) {
        mpuIntDenominator = 1;
        gyro->sampleFrequencyHz = intFrequencyHz;
    }

    mpuIntExtiInit();

    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
    delay(100);
    mpuConfiguration.write(MPU_RA_SIGNAL_PATH_RESET, 0x07);
    delay(100);
    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0);
    delay(100);
    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    mpuConfiguration.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delay(15);
    mpuConfiguration.write(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    delay(15);
    mpuConfiguration.write(MPU_RA_CONFIG, lpf);
    delay(15);
    mpuConfiguration.write(MPU_RA_SMPLRT_DIV, 0);
    delay(100);

    // Data ready interrupt configuration
#ifdef USE_MPU9250_MAG
    mpuConfiguration.write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
#else
    mpuConfiguration.write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
#endif

    delay(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    mpuConfiguration.write(MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
#endif
}
