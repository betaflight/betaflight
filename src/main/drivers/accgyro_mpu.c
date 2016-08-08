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
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"

#include "nvic.h"

#include "system.h"
#include "gpio.h"
#include "exti.h"
#include "bus_i2c.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu3050.h"
#include "accgyro_mpu6050.h"
#include "accgyro_mpu6500.h"
#include "accgyro_spi_mpu6000.h"
#include "accgyro_spi_mpu6500.h"
#include "accgyro_spi_mpu9250.h"
#include "accgyro_mpu.h"

//#define DEBUG_MPU_DATA_READY_INTERRUPT

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t* data);
static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data);

static void mpu6050FindRevision(void);

static volatile bool mpuDataReady;

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(void);
#endif

#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE I2C_DEVICE
#endif

mpuDetectionResult_t mpuDetectionResult;

mpuConfiguration_t mpuConfiguration;
static const extiConfig_t *mpuIntExtiConfig = NULL;

#define MPU_ADDRESS             0x68

// WHO_AM_I register contents for MPU3050, 6050 and 6500
#define MPU6500_WHO_AM_I_CONST              (0x70)
#define MPUx0x0_WHO_AM_I_CONST              (0x68)

#define MPU_INQUIRY_MASK   0x7E

mpuDetectionResult_t *detectMpu(const extiConfig_t *configToUse)
{
    memset(&mpuDetectionResult, 0, sizeof(mpuDetectionResult));
    memset(&mpuConfiguration, 0, sizeof(mpuConfiguration));

    mpuIntExtiConfig = configToUse;

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
        mpuConfiguration.read = mpuReadRegisterI2C;
        mpuConfiguration.write = mpuWriteRegisterI2C;
    } else {
#ifdef USE_SPI
        bool detectedSpiSensor = detectSPISensorsAndUpdateDetectionResult();
        UNUSED(detectedSpiSensor);
#endif

        return &mpuDetectionResult;
    }

    mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;

    // If an MPU3050 is connected sig will contain 0.
    ack = mpuReadRegisterI2C(MPU_RA_WHO_AM_I_LEGACY, 1, &inquiryResult);
    inquiryResult &= MPU_INQUIRY_MASK;
    if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
        mpuDetectionResult.sensor = MPU_3050;
        mpuConfiguration.gyroReadXRegister = MPU3050_GYRO_OUT;
        return &mpuDetectionResult;
    }

    sig &= MPU_INQUIRY_MASK;

    if (sig == MPUx0x0_WHO_AM_I_CONST) {

        mpuDetectionResult.sensor = MPU_60x0;

        mpu6050FindRevision();
    } else if (sig == MPU6500_WHO_AM_I_CONST) {
        mpuDetectionResult.sensor = MPU_65xx_I2C;
    }

    return &mpuDetectionResult;
}

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(void)
{
#ifdef USE_GYRO_SPI_MPU6500
    if (mpu6500SpiDetect()) {
        mpuDetectionResult.sensor = MPU_65xx_SPI;
        mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        mpuConfiguration.read = mpu6500ReadRegister;
        mpuConfiguration.write = mpu6500WriteRegister;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_MPU6000
    if (mpu6000SpiDetect()) {
        mpuDetectionResult.sensor = MPU_60x0_SPI;
        mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        mpuConfiguration.read = mpu6000ReadRegister;
        mpuConfiguration.write = mpu6000WriteRegister;
        return true;
    }
#endif

#ifdef  USE_GYRO_SPI_MPU9250
    if (mpu9250SpiDetect()) {
        mpuDetectionResult.sensor = MPU_9250_SPI;
        mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        mpuConfiguration.read = mpu9250ReadRegister;
        mpuConfiguration.slowread = mpu9250SlowReadRegister;
        mpuConfiguration.verifywrite = verifympu9250WriteRegister;
        mpuConfiguration.write = mpu9250WriteRegister;
        mpuConfiguration.reset = mpu9250ResetGyro;
        return true;
    }
#endif

    return false;
}
#endif

static void mpu6050FindRevision(void)
{
    bool ack;
    UNUSED(ack);

    uint8_t readBuffer[6];
    uint8_t revision;
    uint8_t productId;

    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c

    // determine product ID and accel revision
    ack = mpuConfiguration.read(MPU_RA_XA_OFFS_H, 6, readBuffer);
    revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (revision) {
        /* Congrats, these parts are better. */
        if (revision == 1) {
            mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else if (revision == 2) {
            mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else if ((revision == 3) || (revision == 7)) {
            mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        }
    } else {
        ack = mpuConfiguration.read(MPU_RA_PRODUCT_ID, 1, &productId);
        revision = productId & 0x0F;
        if (!revision) {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if (revision == 4) {
            mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else {
            mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        }
    }
}

extiCallbackRec_t mpuIntCallbackRec;

void mpuIntExtiHandler(extiCallbackRec_t *cb)
{
    UNUSED(cb);
    mpuDataReady = true;

#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    static uint32_t lastCalledAt = 0;
    uint32_t now = micros();
    uint32_t callDelta = now - lastCalledAt;
    debug[0] = callDelta;
    lastCalledAt = now;
#endif
}

void mpuIntExtiInit(void)
{
    static bool mpuExtiInitDone = false;

    if (mpuExtiInitDone || !mpuIntExtiConfig) {
        return;
    }

#if defined(USE_MPU_DATA_READY_SIGNAL) && defined(USE_EXTI)

    IO_t mpuIntIO = IOGetByTag(mpuIntExtiConfig->tag);

#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = IORead(mpuIntIO);
    if (status) {
        return;
    }
#endif

    IOInit(mpuIntIO, OWNER_MPU, RESOURCE_EXTI, 0);
    IOConfigGPIO(mpuIntIO, IOCFG_IN_FLOATING);   // TODO - maybe pullup / pulldown ?

    EXTIHandlerInit(&mpuIntCallbackRec, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &mpuIntCallbackRec, NVIC_PRIO_MPU_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(mpuIntIO, true);
#endif

    mpuExtiInitDone = true;
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

bool mpuAccRead(int16_t *accData)
{
    uint8_t data[6];

    bool ack = mpuConfiguration.read(MPU_RA_ACCEL_XOUT_H, 6, data);
    if (!ack) {
        return false;
    }

    accData[0] = (int16_t)((data[0] << 8) | data[1]);
    accData[1] = (int16_t)((data[2] << 8) | data[3]);
    accData[2] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuGyroRead(int16_t *gyroADC)
{
    uint8_t data[6];

    bool ack = mpuConfiguration.read(mpuConfiguration.gyroReadXRegister, 6, data);
    if (!ack) {
        return false;
    }

    gyroADC[0] = (int16_t)((data[0] << 8) | data[1]);
    gyroADC[1] = (int16_t)((data[2] << 8) | data[3]);
    gyroADC[2] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool checkMPUDataReady(void)
{
    bool ret;
    if (mpuDataReady) {
        ret = true;
        mpuDataReady= false;
    } else {
        ret = false;
    }
    return ret;
}
