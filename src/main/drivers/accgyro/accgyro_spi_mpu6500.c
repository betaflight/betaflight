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

#if defined(USE_ACC_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU6500)

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/bus_spi.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"

#define DISABLE_MPU6500(spiCsnPin)       IOHi(spiCsnPin)
#define ENABLE_MPU6500(spiCsnPin)        IOLo(spiCsnPin)

#define BIT_SLEEP                   0x40

bool mpu6500SpiWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    ENABLE_MPU6500(bus->spi.csnPin);
    delayMicroseconds(1);
    spiTransferByte(MPU6500_SPI_INSTANCE, reg);
    spiTransferByte(MPU6500_SPI_INSTANCE, data);
    DISABLE_MPU6500(bus->spi.csnPin);
    delayMicroseconds(1);

    return true;
}

bool mpu6500SpiReadRegister(const busDevice_t *bus, uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_MPU6500(bus->spi.csnPin);
    spiTransferByte(MPU6500_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU6500_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU6500(bus->spi.csnPin);

    return true;
}

static void mpu6500SpiInit(const busDevice_t *bus)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

    IOInit(bus->spi.csnPin, OWNER_MPU, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(bus->spi.csnPin, SPI_IO_CS_CFG);

    spiSetDivisor(MPU6500_SPI_INSTANCE, SPI_CLOCK_FAST);

    hardwareInitialised = true;
}

uint8_t mpu6500SpiDetect(const busDevice_t *bus)
{
    mpu6500SpiInit(bus);

    mpu6500SpiWriteRegister(bus, MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);

    for (int attempts = 0; attempts < 5; attempts++) {
        uint8_t tmp;

        delay(150);

        mpu6500SpiReadRegister(bus, MPU_RA_WHO_AM_I, 1, &tmp);

        switch (tmp) {
        case MPU6500_WHO_AM_I_CONST:
            return MPU_65xx_SPI;
        case MPU9250_WHO_AM_I_CONST:
            return MPU_9250_SPI;
        case ICM20608G_WHO_AM_I_CONST:
            return ICM_20608_SPI;
        case ICM20602_WHO_AM_I_CONST:
            return ICM_20602_SPI;
        case ICM20689_WHO_AM_I_CONST:
            return ICM_20689_SPI;
        }
    }

    return MPU_NONE;
}

void mpu6500SpiAccInit(accDev_t *acc)
{
    mpu6500AccInit(acc);
}

bool mpu6500SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_65xx_SPI &&
        acc->mpuDetectionResult.sensor != MPU_9250_SPI &&
        acc->mpuDetectionResult.sensor != ICM_20608_SPI &&
        acc->mpuDetectionResult.sensor != ICM_20602_SPI &&
        acc->mpuDetectionResult.sensor != ICM_20689_SPI) {
        return false;
    }

    acc->initFn = mpu6500SpiAccInit;
    acc->readFn = mpuAccRead;

    return true;
}

void mpu6500SpiGyroInit(gyroDev_t *gyro)
{
    spiSetDivisor(MPU6500_SPI_INSTANCE, SPI_CLOCK_SLOW);
    delayMicroseconds(1);

    mpu6500GyroInit(gyro);

    // Disable Primary I2C Interface
    mpu6500SpiWriteRegister(&gyro->bus, MPU_RA_USER_CTRL, MPU6500_BIT_I2C_IF_DIS);
    delay(100);

    spiSetDivisor(MPU6500_SPI_INSTANCE, SPI_CLOCK_FAST);
    delayMicroseconds(1);
}

bool mpu6500SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_65xx_SPI &&
        gyro->mpuDetectionResult.sensor != MPU_9250_SPI &&
        gyro->mpuDetectionResult.sensor != ICM_20608_SPI &&
        gyro->mpuDetectionResult.sensor != ICM_20602_SPI &&
        gyro->mpuDetectionResult.sensor != ICM_20689_SPI) {
        return false;
    }

    gyro->initFn = mpu6500SpiGyroInit;
    gyro->readFn = mpuGyroRead;
    gyro->intStatusFn = mpuCheckDataReady;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}
#endif
