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

#include "nvic.h"

#include "system.h"
#include "io.h"
#include "exti.h"
#include "bus_i2c.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu3050.h"
#include "accgyro_mpu6050.h"
#include "accgyro_mpu6500.h"
#include "accgyro_spi_mpu6000.h"
#include "accgyro_spi_mpu6500.h"
#include "accgyro_spi_icm20689.h"
#include "accgyro_spi_mpu9250.h"
#include "accgyro_mpu.h"


mpuResetFuncPtr mpuReset;

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t* data);
static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data);

static void mpu6050FindRevision(gyroDev_t *gyro);

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro);
#endif

#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE I2C_DEVICE
#endif

#define MPU_ADDRESS             0x68

// WHO_AM_I register contents for MPU3050, 6050 and 6500
#define MPU6500_WHO_AM_I_CONST              (0x70)
#define MPUx0x0_WHO_AM_I_CONST              (0x68)

#define MPU_INQUIRY_MASK   0x7E

mpuDetectionResult_t *mpuDetect(gyroDev_t *gyro)
{
    bool ack;
    uint8_t sig;
    uint8_t inquiryResult;

    // MPU datasheet specifies 30ms.
    delay(35);

#ifndef USE_I2C
    ack = false;
    sig = 0;
#else
    ack = mpuReadRegisterI2C(MPU_RA_WHO_AM_I, 1, &sig);
#endif
    if (ack) {
        gyro->mpuConfiguration.read = mpuReadRegisterI2C;
        gyro->mpuConfiguration.write = mpuWriteRegisterI2C;
    } else {
#ifdef USE_SPI
        bool detectedSpiSensor = detectSPISensorsAndUpdateDetectionResult(gyro);
        UNUSED(detectedSpiSensor);
#endif

        return &gyro->mpuDetectionResult;
    }

    gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;

    // If an MPU3050 is connected sig will contain 0.
    ack = mpuReadRegisterI2C(MPU_RA_WHO_AM_I_LEGACY, 1, &inquiryResult);
    inquiryResult &= MPU_INQUIRY_MASK;
    if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
        gyro->mpuDetectionResult.sensor = MPU_3050;
        gyro->mpuConfiguration.gyroReadXRegister = MPU3050_GYRO_OUT;
        return &gyro->mpuDetectionResult;
    }

    sig &= MPU_INQUIRY_MASK;

    if (sig == MPUx0x0_WHO_AM_I_CONST) {

        gyro->mpuDetectionResult.sensor = MPU_60x0;

        mpu6050FindRevision(gyro);
    } else if (sig == MPU6500_WHO_AM_I_CONST) {
        gyro->mpuDetectionResult.sensor = MPU_65xx_I2C;
    }

    return &gyro->mpuDetectionResult;
}

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro)
{
#ifdef USE_GYRO_SPI_MPU6500
    uint8_t mpu6500Sensor = mpu6500SpiDetect();
    if (mpu6500Sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = mpu6500Sensor;
        gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        gyro->mpuConfiguration.read = mpu6500ReadRegister;
        gyro->mpuConfiguration.write = mpu6500WriteRegister;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_ICM20689
    if (icm20689SpiDetect()) {
        gyro->mpuDetectionResult.sensor = ICM_20689_SPI;
        gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        gyro->mpuConfiguration.read = icm20689ReadRegister;
        gyro->mpuConfiguration.write = icm20689WriteRegister;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_MPU6000
    if (mpu6000SpiDetect()) {
        gyro->mpuDetectionResult.sensor = MPU_60x0_SPI;
        gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        gyro->mpuConfiguration.read = mpu6000ReadRegister;
        gyro->mpuConfiguration.write = mpu6000WriteRegister;
        return true;
    }
#endif

#ifdef  USE_GYRO_SPI_MPU9250
    if (mpu9250SpiDetect()) {
        gyro->mpuDetectionResult.sensor = MPU_9250_SPI;
        gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        gyro->mpuConfiguration.read = mpu9250ReadRegister;
        gyro->mpuConfiguration.slowread = mpu9250SlowReadRegister;
        gyro->mpuConfiguration.verifywrite = verifympu9250WriteRegister;
        gyro->mpuConfiguration.write = mpu9250WriteRegister;
        gyro->mpuConfiguration.reset = mpu9250ResetGyro;
        return true;
    }
#endif

    UNUSED(gyro);
    return false;
}
#endif

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
    ack = gyro->mpuConfiguration.read(MPU_RA_XA_OFFS_H, 6, readBuffer);
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
        ack = gyro->mpuConfiguration.read(MPU_RA_PRODUCT_ID, 1, &productId);
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
    if (gyro->update) {
        gyro->update(gyro);
    }
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    const uint32_t now2Us = micros();
    debug[1] = (uint16_t)(now2Us - nowUs);
#endif
}
#endif

static void mpuIntExtiInit(gyroDev_t *gyro)
{
#if defined(MPU_INT_EXTI)
    if (!gyro->mpuIntExtiConfig) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiConfig->tag);

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
    EXTIEnable(mpuIntIO, true);
#endif

#else
    UNUSED(gyro);
#endif
}

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t* data)
{
    bool ack = i2cRead(MPU_I2C_INSTANCE, MPU_ADDRESS, reg, length, data);
    return ack;
}

static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data)
{
    bool ack = i2cWrite(MPU_I2C_INSTANCE, MPU_ADDRESS, reg, data);
    return ack;
}

bool mpuAccRead(accDev_t *acc)
{
    uint8_t data[6];

    bool ack = acc->mpuConfiguration.read(MPU_RA_ACCEL_XOUT_H, 6, data);
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
        gyro->update = updateFn;
    }
}

bool mpuGyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = gyro->mpuConfiguration.read(gyro->mpuConfiguration.gyroReadXRegister, 6, data);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

void mpuGyroInit(gyroDev_t *gyro)
{
    mpuIntExtiInit(gyro);
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
