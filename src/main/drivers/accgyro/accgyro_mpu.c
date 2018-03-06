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
#include <string.h>

#include "platform.h"

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/accgyro/accgyro_mpu.h"


mpuResetFnPtr mpuResetFn;

#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE I2C_DEVICE
#endif

#ifndef MPU_ADDRESS
#define MPU_ADDRESS             0x68
#endif

#define MPU_INQUIRY_MASK   0x7E

#ifdef USE_I2C
static void mpu6050FindRevision(gyroDev_t *gyro)
{
    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c

    // determine product ID and revision
    uint8_t readBuffer[6];
    bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_XA_OFFS_H, readBuffer, 6);
    uint8_t revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (ack && revision) {
        // Congrats, these parts are better
        if (revision == 1) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else if (revision == 2) {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else if ((revision == 3) || (revision == 7)) {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        }
    } else {
        uint8_t productId;
        ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_PRODUCT_ID, &productId, 1);
        revision = productId & 0x0F;
        if (!ack || revision == 0) {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if (revision == 4) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        }
    }
}
#endif

/*
 * Gyro interrupt service routine
 */
#if defined(MPU_INT_EXTI)
static void mpuIntExtiHandler(extiCallbackRec_t *cb)
{
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    static uint32_t lastCalledAtUs = 0;
    const uint32_t nowUs = micros();
    debug[0] = (uint16_t)(nowUs - lastCalledAtUs);
    lastCalledAtUs = nowUs;
#endif
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    const uint32_t now2Us = micros();
    debug[1] = (uint16_t)(now2Us - nowUs);
#endif
}

static void mpuIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = IORead(mpuIntIO);
    if (status) {
        return;
    }
#endif

#if defined (STM32F7)
    IOInit(mpuIntIO, OWNER_MPU_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IO_CONFIG(GPIO_MODE_INPUT,0,GPIO_NOPULL));   // TODO - maybe pullup / pulldown ?
#else
    IOInit(mpuIntIO, OWNER_MPU_EXTI, 0);
    IOConfigGPIO(mpuIntIO, IOCFG_IN_FLOATING);   // TODO - maybe pullup / pulldown ?

    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, EXTI_Trigger_Rising);
#endif
    EXTIEnable(mpuIntIO, true);
}
#endif // MPU_INT_EXTI

bool mpuAccRead(accDev_t *acc)
{
    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&acc->bus, MPU_RA_ACCEL_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuGyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_GYRO_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuGyroReadSPI(gyroDev_t *gyro)
{
    static const uint8_t dataToSend[7] = {MPU_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[7];

    const bool ack = spiBusTransfer(&gyro->bus, dataToSend, data, 7);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);

    return true;
}

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro)
{
    UNUSED(gyro); // since there are FCs which have gyro on I2C but other devices on SPI

    uint8_t sensor = MPU_NONE;
    UNUSED(sensor);

    // note, when USE_DUAL_GYRO is enabled the gyro->bus must already be initialised.

#ifdef USE_GYRO_SPI_MPU6000
#ifndef USE_DUAL_GYRO
    spiBusSetInstance(&gyro->bus, MPU6000_SPI_INSTANCE);
#endif
#ifdef MPU6000_CS_PIN
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6000_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
#endif
    sensor = mpu6000SpiDetect(&gyro->bus);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_MPU6500
#ifndef USE_DUAL_GYRO
    spiBusSetInstance(&gyro->bus, MPU6500_SPI_INSTANCE);
#endif
#ifdef MPU6500_CS_PIN
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6500_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
#endif
    sensor = mpu6500SpiDetect(&gyro->bus);
    // some targets using MPU_9250_SPI, ICM_20608_SPI or ICM_20602_SPI state sensor is MPU_65xx_SPI
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif

#ifdef  USE_GYRO_SPI_MPU9250
#ifndef USE_DUAL_GYRO
    spiBusSetInstance(&gyro->bus, MPU9250_SPI_INSTANCE);
#endif
#ifdef MPU9250_CS_PIN
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU9250_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
#endif
    sensor = mpu9250SpiDetect(&gyro->bus);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        gyro->mpuConfiguration.resetFn = mpu9250SpiResetGyro;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_ICM20649
#ifdef ICM20649_SPI_INSTANCE
    spiBusSetInstance(&gyro->bus, ICM20649_SPI_INSTANCE);
#endif
#ifdef ICM20649_CS_PIN
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM20649_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
#endif
    sensor = icm20649SpiDetect(&gyro->bus);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_ICM20689
#ifndef USE_DUAL_GYRO
    spiBusSetInstance(&gyro->bus, ICM20689_SPI_INSTANCE);
#endif
#ifdef ICM20689_CS_PIN
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM20689_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
#endif
    sensor = icm20689SpiDetect(&gyro->bus);
    // icm20689SpiDetect detects ICM20602 and ICM20689
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif

#ifdef USE_ACCGYRO_BMI160
#ifndef USE_DUAL_GYRO
    spiBusSetInstance(&gyro->bus, BMI160_SPI_INSTANCE);
#endif
#ifdef BMI160_CS_PIN
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(BMI160_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
#endif
    sensor = bmi160Detect(&gyro->bus);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif

    return false;
}
#endif

void mpuDetect(gyroDev_t *gyro)
{
    // MPU datasheet specifies 30ms.
    delay(35);

#ifdef USE_I2C
    if (gyro->bus.bustype == BUSTYPE_NONE) {
        // if no bustype is selected try I2C first.
        gyro->bus.bustype = BUSTYPE_I2C;
    }

    if (gyro->bus.bustype == BUSTYPE_I2C) {
        gyro->bus.busdev_u.i2c.device = MPU_I2C_INSTANCE;
        gyro->bus.busdev_u.i2c.address = MPU_ADDRESS;

        uint8_t sig = 0;
        bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_WHO_AM_I, &sig, 1);

        if (ack) {
            // If an MPU3050 is connected sig will contain 0.
            uint8_t inquiryResult;
            ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_WHO_AM_I_LEGACY, &inquiryResult, 1);
            inquiryResult &= MPU_INQUIRY_MASK;
            if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_3050;
                return;
            }

            sig &= MPU_INQUIRY_MASK;
            if (sig == MPUx0x0_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_60x0;
                mpu6050FindRevision(gyro);
            } else if (sig == MPU6500_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_65xx_I2C;
            }
            return;
        }
    }
#endif

#ifdef USE_SPI
    gyro->bus.bustype = BUSTYPE_SPI;
    detectSPISensorsAndUpdateDetectionResult(gyro);
#endif
}

void mpuGyroInit(gyroDev_t *gyro)
{
#ifdef MPU_INT_EXTI
    mpuIntExtiInit(gyro);
#else
    UNUSED(gyro);
#endif
}
