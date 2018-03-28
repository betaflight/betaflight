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

#ifdef USE_DMA_SPI_DEVICE
#include "drivers/dma_spi.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#endif //USE_DMA_SPI_DEVICE

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
#ifdef USE_GYRO_IMUF9001
#include "drivers/accgyro/accgyro_imuf9001.h"
#endif //USE_GYRO_IMUF9001
#include "drivers/accgyro/accgyro_mpu.h"


#ifdef USE_DMA_SPI_DEVICE
volatile int dmaSpiGyroDataReady = 0;
volatile uint32_t imufCrcErrorCount = 0;
#endif //USE_DMA_SPI_DEVICE

mpuResetFnPtr mpuResetFn;
#ifdef USE_GYRO_IMUF9001
    imufData_t imufData;
#endif
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
#ifdef USE_DMA_SPI_DEVICE
    //start dma read
    (void)(cb);
    gyroDmaSpiStartRead();
#else
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
    EXTIEnable(mpuIntIO, true);
#endif
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

#ifdef USE_DMA_SPI_DEVICE
bool mpuGyroDmaSpiReadStart(gyroDev_t * gyro)
{
    (void)(gyro); ///not used at this time
    //no reason not to get acc and gyro data at the same time
    lastImufExtiTime = micros();
    #ifdef USE_GYRO_IMUF9001
    if (isImufCalibrating) //calibrating
    {
        //two steps
        //step 1 is isImufCalibrating=1, this starts the calibration command and sends it to the IMU-f
        //step 2 is isImufCalibrating=2, this sets the tx buffer back to 0 so we don't keep sending the calibration command over and over
        memset(dmaTxBuffer, 0, sizeof(imufCommand_t)); //clear buffer
        if (isImufCalibrating == IMUF_CALIBRATION_STEP1) //step 1, set the command to be sent
        {
            //set calibration command with CRC, typecast the dmaTxBuffer as imufCommand_t
            (*(imufCommand_t *)(dmaTxBuffer)).command = IMUF_COMMAND_CALIBRATE;
            (*(imufCommand_t *)(dmaTxBuffer)).crc     = getCrcImuf9001((uint32_t *)dmaTxBuffer, 11); //typecast the dmaTxBuffer as a uint32_t array which is what the crc command needs
            //set isImufCalibrating to step 2, which is just used so the memset to 0 runs after the calibration commmand is sent
            isImufCalibrating = IMUF_CALIBRATION_STEP2; //go to step two
        }
        else
        {   //step 2, memset of the tx buffer has run, set isImufCalibrating to 0.
            isImufCalibrating = IMUF_NOT_CALIBRATING;
        }

    }
    memset(dmaRxBuffer, 0, gyroConfig()->imuf_mode); //clear buffer
    //send and receive data using SPI and DMA
    dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, gyroConfig()->imuf_mode, 0);
    #else
    dmaTxBuffer[0] = MPU_RA_ACCEL_XOUT_H | 0x80;
    dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, 15, 0);
    #endif
    return true;
}

void mpuGyroDmaSpiReadFinish(gyroDev_t * gyro)
{
    //spi rx dma callback
    #ifdef USE_GYRO_IMUF9001
    volatile uint32_t crc1 = ( (*(uint32_t *)(dmaRxBuffer+gyroConfig()->imuf_mode-4)) & 0xFF );
    volatile uint32_t crc2 = ( getCrcImuf9001((uint32_t *)(dmaRxBuffer), (gyroConfig()->imuf_mode >> 2)-1) & 0xFF );
    if(crc1 == crc2)
    {
        memcpy(&imufData, dmaRxBuffer, sizeof(imufData_t));
        acc.accADC[X]    = imufData.accX * acc.dev.acc_1G;
        acc.accADC[Y]    = imufData.accY * acc.dev.acc_1G;
        acc.accADC[Z]    = imufData.accZ * acc.dev.acc_1G;
        gyro->gyroADC[X] = imufData.gyroX;
        gyro->gyroADC[Y] = imufData.gyroY;
        gyro->gyroADC[Z] = imufData.gyroZ;
    }
    else
    {
        //error handler
        imufCrcErrorCount++; //check every so often and cause a failsafe is this number is above a certain ammount
    }
    #else
    acc.dev.ADCRaw[X]   = (int16_t)((dmaRxBuffer[1] << 8)  | dmaRxBuffer[2]);
    acc.dev.ADCRaw[Y]   = (int16_t)((dmaRxBuffer[3] << 8)  | dmaRxBuffer[4]);
    acc.dev.ADCRaw[Z]   = (int16_t)((dmaRxBuffer[5] << 8)  | dmaRxBuffer[6]);
    gyro->gyroADCRaw[X] = (int16_t)((dmaRxBuffer[9] << 8)  | dmaRxBuffer[10]);
    gyro->gyroADCRaw[Y] = (int16_t)((dmaRxBuffer[11] << 8) | dmaRxBuffer[12]);
    gyro->gyroADCRaw[Z] = (int16_t)((dmaRxBuffer[13] << 8) | dmaRxBuffer[14]);
    #endif
    dmaSpiGyroDataReady = 1; //set flag to tell scheduler data is ready
}
#endif


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

#ifdef USE_GYRO_SPI_MPU6000
    #ifdef MPU6000_SPI_INSTANCE
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
#ifdef MPU6500_SPI_INSTANCE
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

#ifdef USE_GYRO_IMUF9001
    #ifdef IMUF9001_SPI_INSTANCE
        spiBusSetInstance(&gyro->bus, IMUF9001_SPI_INSTANCE);
    #else
        #error IMUF9001 is SPI only
    #endif
    #ifdef IMUF9001_CS_PIN
        gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(IMUF9001_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
    #else
       #error IMUF9001 must use a CS pin (IMUF9001_CS_PIN)
    #endif
    #ifdef IMUF9001_RST_PIN
        gyro->bus.busdev_u.spi.rstPin = IOGetByTag(IO_TAG(IMUF9001_RST_PIN));
    #else
        #error IMUF9001 must use a RST pin (IMUF9001_RST_PIN)
    #endif
    sensor = imuf9001SpiDetect(gyro);
    // some targets using MPU_9250_SPI, ICM_20608_SPI or ICM_20602_SPI state sensor is MPU_65xx_SPI
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif

#ifdef  USE_GYRO_SPI_MPU9250
    #ifdef MPU9250_SPI_INSTANCE
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
    #ifdef ICM20689_SPI_INSTANCE
    spiBusSetInstance(&gyro->bus, ICM20689_SPI_INSTANCE);
    #endif
    #ifdef ICM20689_CS_PIN
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM20689_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
    #endif
    sensor = icm20689SpiDetect(&gyro->bus);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif

#ifdef USE_ACCGYRO_BMI160
    #ifdef BMI160_SPI_INSTANCE
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
    uint8_t sig = 0;
#endif

#ifdef USE_DMA_SPI_DEVICE
    bool ack = false;
#elif defined(USE_I2C)
    gyro->bus.bustype = BUSTYPE_I2C;
    gyro->bus.busdev_u.i2c.device = MPU_I2C_INSTANCE;
    gyro->bus.busdev_u.i2c.address = MPU_ADDRESS;
    bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_WHO_AM_I, &sig, 1);
#else
    bool ack = false;
#endif

    if (!ack) {
#ifdef USE_SPI
        detectSPISensorsAndUpdateDetectionResult(gyro);
#endif
        return;
    }

#ifdef USE_I2C
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
