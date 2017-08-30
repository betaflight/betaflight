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

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/exti.h"
#include "drivers/gpio.h"
#include "drivers/gyro_sync.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu6500.h"

void mpu6500AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 8;
}

bool mpu6500AccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    acc->initFn = mpu6500AccInit;
    acc->readFn = mpuAccRead;

    return true;
}

void mpu6500GyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

#ifdef NAZE
    // FIXME target specific code in driver code.

    gpio_config_t gpio;
    // MPU_INT output on rev5 hardware (PC13). rev4 was on PB13, conflicts with SPI devices
    if (hse_value == 12000000) {
        gpio.pin = Pin_13;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_IN_FLOATING;
        gpioInit(GPIOC, &gpio);
    }
#endif

    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
    delay(100);
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_SIGNAL_PATH_RESET, 0x07);
    delay(100);
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_PWR_MGMT_1, 0);
    delay(100);
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    const uint8_t raGyroConfigData = gyro->gyroRateKHz > GYRO_RATE_8_kHz ? (INV_FSR_2000DPS << 3 | FCB_3600_32) : (INV_FSR_2000DPS << 3 | FCB_DISABLED);
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_GYRO_CONFIG, raGyroConfigData);
    delay(15);
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    delay(15);
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_CONFIG, gyro->lpf);
    delay(15);
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops(gyro)); // Get Divider
    delay(100);

    // Data ready interrupt configuration
#ifdef USE_MPU9250_MAG
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
#else
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
#endif
    delay(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
#endif
    delay(15);
}

bool mpu6500GyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    gyro->initFn = mpu6500GyroInit;
    gyro->readFn = mpuGyroRead;
    gyro->intStatusFn = mpuCheckDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}
