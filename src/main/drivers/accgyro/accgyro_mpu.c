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

#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/accgyro/accgyro_mpu.h"

//#define DEBUG_MPU_DATA_READY_INTERRUPT

mpuResetFnPtr mpuResetFn;

#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE I2C_DEVICE
#endif

#define MPU_ADDRESS             0x68

#define MPU_INQUIRY_MASK   0x7E

static void mpu6050FindRevision(gyroDev_t *gyro)
{
    bool ack;
    UNUSED(ack);

    uint8_t readBuffer[6];
    uint8_t revision;
    uint8_t productId;

    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c

    // determine product ID and accel revision
    ack = gyro->mpuConfiguration.readFn(&gyro->bus, MPU_RA_XA_OFFS_H, 6, readBuffer);
    revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (revision) {
        /* Congrats, these parts are better. */
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
        ack = gyro->mpuConfiguration.readFn(&gyro->bus, MPU_RA_PRODUCT_ID, 1, &productId);
        revision = productId & 0x0F;
        if (!revision) {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if (revision == 4) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        }
    }
}

/*
 * Gyro interrupt service routine
 */
#if defined(USE_MPU_DATA_READY_SIGNAL) && defined(USE_EXTI)
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
    if (gyro->updateFn) {
        gyro->updateFn(gyro);
    }
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    const uint32_t now2Us = micros();
    debug[1] = (uint16_t)(now2Us - nowUs);
#endif
}
#endif

static void mpuIntExtiInit(gyroDev_t *gyro)
{
    if (!gyro->mpuIntExtiConfig) {
        return;
    }

#if defined(USE_MPU_DATA_READY_SIGNAL) && defined(USE_EXTI)

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiConfig->tag);

#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = IORead(mpuIntIO);
    if (status) {
        return;
    }
#endif

#if defined (STM32F7)
    IOInit(mpuIntIO, OWNER_MPU, RESOURCE_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IO_CONFIG(GPIO_MODE_INPUT,0,GPIO_NOPULL));   // TODO - maybe pullup / pulldown ?
#else

    IOInit(mpuIntIO, OWNER_MPU, RESOURCE_EXTI, 0);
    IOConfigGPIO(mpuIntIO, IOCFG_IN_FLOATING);   // TODO - maybe pullup / pulldown ?

    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(mpuIntIO, true);
#endif
#endif
}

bool mpuReadRegisterI2C(const busDevice_t *bus, uint8_t reg, uint8_t length, uint8_t* data)
{
    UNUSED(bus);
    const bool ack = i2cRead(MPU_I2C_INSTANCE, MPU_ADDRESS, reg, length, data);
    return ack;
}

bool mpuWriteRegisterI2C(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    UNUSED(bus);
    const bool ack = i2cWrite(MPU_I2C_INSTANCE, MPU_ADDRESS, reg, data);
    return ack;
}

bool mpuAccRead(accDev_t *acc)
{
    uint8_t data[6];

    const bool ack = acc->mpuConfiguration.readFn(&acc->bus, MPU_RA_ACCEL_XOUT_H, 6, data);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

void mpuGyroSetIsrUpdate(gyroDev_t *gyro, sensorGyroUpdateFuncPtr updateFn)
{
    ATOMIC_BLOCK(NVIC_PRIO_MPU_INT_EXTI) {
        gyro->updateFn = updateFn;
    }
}

bool mpuGyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = gyro->mpuConfiguration.readFn(&gyro->bus, gyro->mpuConfiguration.gyroReadXRegister, 6, data);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuCheckDataReady(gyroDev_t* gyro)
{
    bool ret;
    if (gyro->dataReady) {
        ret = true;
        gyro->dataReady= false;
    } else {
        ret = false;
    }
    return ret;
}

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro)
{
    UNUSED(gyro); // since there are FCs which have gyro on I2C but other devices on SPI

    uint8_t sensor = MPU_NONE;
    UNUSED(sensor);

#ifdef USE_GYRO_SPI_MPU6000
#ifdef MPU6000_CS_PIN
    gyro->bus.spi.csnPin = gyro->bus.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6000_CS_PIN)) : gyro->bus.spi.csnPin;
#endif
    sensor = mpu6000SpiDetect(&gyro->bus);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = MPU_60x0_SPI;
        gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        gyro->mpuConfiguration.readFn = mpu6000SpiReadRegister;
        gyro->mpuConfiguration.writeFn = mpu6000SpiWriteRegister;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_MPU6500
#ifdef MPU6500_CS_PIN
    gyro->bus.spi.csnPin = gyro->bus.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6500_CS_PIN)) : gyro->bus.spi.csnPin;
#endif
    sensor = mpu6500SpiDetect(&gyro->bus);
    // some targets using MPU_9250_SPI, ICM_20608_SPI or ICM_20602_SPI state sensor is MPU_65xx_SPI
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        gyro->mpuConfiguration.readFn = mpu6500SpiReadRegister;
        gyro->mpuConfiguration.writeFn = mpu6500SpiWriteRegister;
        return true;
    }
#endif

#ifdef  USE_GYRO_SPI_MPU9250
#ifdef MPU9250_CS_PIN
    gyro->bus.spi.csnPin = gyro->bus.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU9250_CS_PIN)) : gyro->bus.spi.csnPin;
#endif
    sensor = mpu9250SpiDetect(&gyro->bus);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = MPU_9250_SPI;
        gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        gyro->mpuConfiguration.readFn = mpu9250SpiReadRegister;
        gyro->mpuConfiguration.writeFn = mpu9250SpiWriteRegister;
        gyro->mpuConfiguration.resetFn = mpu9250SpiResetGyro;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_ICM20608
#ifdef ICM20608_CS_PIN
    gyro->bus.spi.csnPin = gyro->bus.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM20608_CS_PIN)) : gyro->bus.spi.csnPin;
#endif
    sensor = icm20608SpiDetect(&gyro->bus);
    if (sensor != MPU_NONE) {
        mpuDetectionResult.sensor = ICM_20608_SPI;
        mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        mpuConfiguration.readFn = icm20608SpiReadRegister;
        mpuConfiguration.writeFn = icm20608SpiWriteRegister;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_ICM20689
#ifdef ICM20689_CS_PIN
    gyro->bus.spi.csnPin = gyro->bus.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM20689_CS_PIN)) : gyro->bus.spi.csnPin;
#endif
    sensor = icm20689SpiDetect(&gyro->bus);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = ICM_20689_SPI;
        gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        gyro->mpuConfiguration.readFn = icm20689SpiReadRegister;
        gyro->mpuConfiguration.writeFn = icm20689SpiWriteRegister;
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

    uint8_t sig = 0;
#ifdef USE_I2C
    bool ack = mpuReadRegisterI2C(&gyro->bus, MPU_RA_WHO_AM_I, 1, &sig);
#else
    bool ack = false;
#endif
    if (ack) {
        gyro->mpuConfiguration.readFn = mpuReadRegisterI2C;
        gyro->mpuConfiguration.writeFn = mpuWriteRegisterI2C;
    } else {
#ifdef USE_SPI
        detectSPISensorsAndUpdateDetectionResult(gyro);
#endif
        return;
    }

    gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;

    // If an MPU3050 is connected sig will contain 0.
    uint8_t inquiryResult;
    ack = mpuReadRegisterI2C(&gyro->bus, MPU_RA_WHO_AM_I_LEGACY, 1, &inquiryResult);
    inquiryResult &= MPU_INQUIRY_MASK;
    if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
        gyro->mpuDetectionResult.sensor = MPU_3050;
        gyro->mpuConfiguration.gyroReadXRegister = MPU3050_GYRO_OUT;
        return;
    }

    sig &= MPU_INQUIRY_MASK;
    if (sig == MPUx0x0_WHO_AM_I_CONST) {
        gyro->mpuDetectionResult.sensor = MPU_60x0;
        mpu6050FindRevision(gyro);
    } else if (sig == MPU6500_WHO_AM_I_CONST) {
        gyro->mpuDetectionResult.sensor = MPU_65xx_I2C;
    }
}

void mpuGyroInit(gyroDev_t *gyro)
{
    mpuIntExtiInit(gyro);
}
