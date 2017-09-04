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

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/nvic.h"

#include "drivers/time.h"
#include "drivers/gpio.h"
#include "drivers/exti.h"
#include "drivers/bus_i2c.h"
#include "drivers/gyro_sync.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu6050.h"

//#define DEBUG_MPU_DATA_READY_INTERRUPT

// MPU6050, Standard address 0x68
// MPU_INT on PB13 on rev4 Naze32 hardware
#define MPU6050_ADDRESS         0x68

#define DMP_MEM_START_ADDR 0x6E
#define DMP_MEM_R_W 0x6F

#define MPU6050_SMPLRT_DIV      0       // 8000Hz

static void mpu6050AccInit(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.resolution) {
        case MPU_HALF_RESOLUTION:
            acc->acc_1G = 256 * 8;
            break;
        case MPU_FULL_RESOLUTION:
            acc->acc_1G = 512 * 8;
            break;
    }
}

bool mpu6050AccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }

    acc->initFn = mpu6050AccInit;
    acc->readFn = mpuAccRead;
    acc->revisionCode = (acc->mpuDetectionResult.resolution == MPU_HALF_RESOLUTION ? 'o' : 'n'); // es/non-es variance between MPU6050 sensors, half of the naze boards are mpu6000ES.

    return true;
}

static void mpu6050GyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    bool ack = gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(100);
    ack = gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_PWR_MGMT_1, 0x03); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    ack = gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops(gyro)); //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    delay(15); //PLL Settling time when changing CLKSEL is max 10ms.  Use 15ms to be sure
    ack = gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_CONFIG, gyro->lpf); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    ack = gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);   //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec

    // ACC Init stuff.
    // Accel scale 8g (4096 LSB/g)
    ack = gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);

    ack = gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_INT_PIN_CFG,
            0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0); // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS

#ifdef USE_MPU_DATA_READY_SIGNAL
    ack = gyro->mpuConfiguration.writeFn(&gyro->bus, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
#endif
    UNUSED(ack);
}

bool mpu6050GyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }
    gyro->initFn = mpu6050GyroInit;
    gyro->readFn = mpuGyroRead;
    gyro->intStatusFn = mpuCheckDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}
