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
#include <string.h>

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_AK8963

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"

#include "drivers/sensor.h"
#include "drivers/compass/compass.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/compass/compass_ak8963.h"

// This sensor is available in MPU-9250.

// AK8963, mag sensor address
#define AK8963_MAG_I2C_ADDRESS          0x0C
#define AK8963_Device_ID                0x48

// Registers
#define AK8963_MAG_REG_WHO_AM_I         0x00
#define AK8963_MAG_REG_INFO             0x01
#define AK8963_MAG_REG_STATUS1          0x02
#define AK8963_MAG_REG_HXL              0x03
#define AK8963_MAG_REG_HXH              0x04
#define AK8963_MAG_REG_HYL              0x05
#define AK8963_MAG_REG_HYH              0x06
#define AK8963_MAG_REG_HZL              0x07
#define AK8963_MAG_REG_HZH              0x08
#define AK8963_MAG_REG_STATUS2          0x09
#define AK8963_MAG_REG_CNTL             0x0a
#define AK8963_MAG_REG_ASCT             0x0c // self test
#define AK8963_MAG_REG_ASAX             0x10 // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAY             0x11 // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAZ             0x12 // Fuse ROM z-axis sensitivity adjustment value

#define READ_FLAG                       0x80

#define STATUS1_DATA_READY              0x01
#define STATUS1_DATA_OVERRUN            0x02

#define STATUS2_DATA_ERROR              0x02
#define STATUS2_MAG_SENSOR_OVERFLOW     0x03

#define CNTL_MODE_POWER_DOWN            0x00
#define CNTL_MODE_ONCE                  0x01
#define CNTL_MODE_CONT1                 0x02
#define CNTL_MODE_CONT2                 0x06
#define CNTL_MODE_SELF_TEST             0x08
#define CNTL_MODE_FUSE_ROM              0x0F

static float magGain[3] = { 1.0f, 1.0f, 1.0f };

#if defined(USE_SPI) && defined(MPU6500_SPI_INSTANCE)
#define DISABLE_SPI_MPU(spiCsnPin)  IOHi(spiCsnPin)
#define ENABLE_SPI_MPU(spiCsnPin)   IOLo(spiCsnPin)
#define MPU_SPI_INSTANCE            MPU6500_SPI_INSTANCE

typedef struct queuedReadState_s {
    bool waiting;
    uint8_t len;
    timeUs_t readStartedAt; // time read was queued in micros.
} queuedReadState_t;

typedef enum {
    CHECK_STATUS = 0,
    WAITING_FOR_STATUS,
    WAITING_FOR_DATA
} ak8963ReadState_e;

static queuedReadState_t queuedRead = { false, 0, 0};

/* We have AK8963 connected internally to MPU9250 I2C master. Accessing the compass sensor requires
 * setting up the MPU's I2C host. We have separate implementation of SPI read/write functions to access
 * the MPU registers
 */
static bool mpuSpiWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    ENABLE_SPI_MPU(bus->spi.csnPin);
    delayMicroseconds(1);
    spiTransferByte(MPU_SPI_INSTANCE, reg);
    spiTransferByte(MPU_SPI_INSTANCE, data);
    DISABLE_SPI_MPU(bus->spi.csnPin);
    delayMicroseconds(1);

    return true;
}

static bool mpuSpiReadRegister(const busDevice_t *bus, uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_SPI_MPU(bus->spi.csnPin);
    spiTransferByte(MPU_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(MPU_SPI_INSTANCE, data, NULL, length);
    DISABLE_SPI_MPU(bus->spi.csnPin);

    return true;
}

static bool ak8963SensorRead(magDev_t *magDev, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *buf)
{
    // Setting up MPU9250's I2C master to read from AK8963 via internal I2C bus
    mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_SLV0_ADDR, addr_ | READ_FLAG);   // set I2C slave address for read
    mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_SLV0_REG, reg_);                 // set I2C slave register
    mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_SLV0_CTRL, len_ | 0x80);         // read number of bytes
    delay(10);                                                                    // wait for transaction to complete
    __disable_irq();
    mpuSpiReadRegister(&magDev->bus, MPU_RA_EXT_SENS_DATA_00, len_, buf);         // read I2C
    __enable_irq();
    return true;
}

static bool ak8963SensorWrite(magDev_t *magDev, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    // Setting up MPU9250's I2C master to write to AK8963 via internal I2C bus
    mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_SLV0_ADDR, addr_);               // set I2C slave address for write
    mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_SLV0_REG, reg_);                 // set I2C slave register
    mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_SLV0_DO, data);                  // set I2C salve value
    mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_SLV0_CTRL, 0x81);                // write 1 byte
    return true;
}

static bool ak8963SensorStartRead(magDev_t *magDev, uint8_t addr_, uint8_t reg_, uint8_t len_)
{
    // Setting up a read. We can't busy-wait with delay() when in normal operation.
    // Instead we'll set up a read and raise the flag to finalize the read after certain timeout

    if (queuedRead.waiting) {
        return false;
    }

    queuedRead.len = len_;

    mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_SLV0_ADDR, addr_ | READ_FLAG);   // set I2C slave address for read
    mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_SLV0_REG, reg_);                 // set I2C slave register
    mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_SLV0_CTRL, len_ | 0x80);         // read number of bytes

    queuedRead.readStartedAt = micros();
    queuedRead.waiting = true;

    return true;
}

static timeDelta_t ak8963SensorQueuedReadTimeRemaining(void)
{
    if (!queuedRead.waiting) {
        return 0;
    }

    timeDelta_t timeSinceStarted = micros() - queuedRead.readStartedAt;

    timeDelta_t timeRemaining = 8000 - timeSinceStarted;

    if (timeRemaining < 0) {
        return 0;
    }

    return timeRemaining;
}

static bool ak8963SensorCompleteRead(magDev_t *magDev, uint8_t *buf)
{
    // Finalizing the read of AK8963 registers via MPU9250's built-in I2C master
    timeDelta_t timeRemaining = ak8963SensorQueuedReadTimeRemaining();

    if (timeRemaining > 0) {
        delayMicroseconds(timeRemaining);
    }

    queuedRead.waiting = false;

    mpuSpiReadRegister(&magDev->bus, MPU_RA_EXT_SENS_DATA_00, queuedRead.len, buf);               // read I2C buffer
    return true;
}
#else
static bool ak8963SensorRead(magDev_t *magDev, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    UNUSED(magDev);
    return i2cRead(MAG_I2C_INSTANCE, addr_, reg_, len, buf);
}

static bool ak8963SensorWrite(magDev_t *magDev, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    UNUSED(magDev);
    return i2cWrite(MAG_I2C_INSTANCE, addr_, reg_, data);
}
#endif

static bool ak8963Init(magDev_t *magDev)
{
    bool ack;
    UNUSED(ack);
    uint8_t calibration[3];
    uint8_t status;

    ack = ak8963SensorWrite(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, CNTL_MODE_POWER_DOWN); // power down before entering fuse mode
    delay(20);

    ack = ak8963SensorWrite(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, CNTL_MODE_FUSE_ROM); // Enter Fuse ROM access mode
    delay(10);

    ack = ak8963SensorRead(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_ASAX, sizeof(calibration), calibration); // Read the x-, y-, and z-axis calibration values
    delay(10);

    magGain[X] = (((((float)(int8_t)calibration[X] - 128) / 256) + 1) * 30);
    magGain[Y] = (((((float)(int8_t)calibration[Y] - 128) / 256) + 1) * 30);
    magGain[Z] = (((((float)(int8_t)calibration[Z] - 128) / 256) + 1) * 30);

    ack = ak8963SensorWrite(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, CNTL_MODE_POWER_DOWN); // power down after reading.
    delay(10);

    // Clear status registers
    ack = ak8963SensorRead(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_STATUS1, 1, &status);
    ack = ak8963SensorRead(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_STATUS2, 1, &status);

    // Trigger first measurement
#if defined(USE_SPI) && defined(MPU6500_SPI_INSTANCE)
    ack = ak8963SensorWrite(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, CNTL_MODE_CONT1);
#else
    ack = ak8963SensorWrite(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, CNTL_MODE_ONCE);
#endif
    return true;
}

static bool ak8963Read(magDev_t *magDev)
{
    bool ack = false;
    uint8_t buf[7];

    static bool lastReadResult = false;

#if defined(USE_SPI) && defined(MPU6500_SPI_INSTANCE)
    static int16_t cachedMagData[3];

    // set magData to latest cached value
    memcpy(&magDev->magADCRaw, cachedMagData, sizeof(cachedMagData));

    // we currently need a different approach for the MPU9250 connected via SPI.
    // we cannot use the ak8963SensorRead() method for SPI, it is to slow and blocks for far too long.

    static ak8963ReadState_e state = CHECK_STATUS;

    bool retry = true;

restart:
    switch (state) {
        case CHECK_STATUS:
            ak8963SensorStartRead(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_STATUS1, 1);
            state++;
            return lastReadResult;

        case WAITING_FOR_STATUS: {
            uint32_t timeRemaining = ak8963SensorQueuedReadTimeRemaining();
            if (timeRemaining) {
                return lastReadResult;
            }

            ack = ak8963SensorCompleteRead(magDev, &buf[0]);

            uint8_t status = buf[0];

            if (!ack || ((status & STATUS1_DATA_READY) == 0 && (status & STATUS1_DATA_OVERRUN) == 0)) {
                // too early. queue the status read again
                state = CHECK_STATUS;
                if (retry) {
                    retry = false;
                    goto restart;
                }

                lastReadResult = false;
                return lastReadResult;
            }

            // read the 6 bytes of data and the status2 register
            ak8963SensorStartRead(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_HXL, 7);

            state++;

            return lastReadResult;
        }

        case WAITING_FOR_DATA: {
            uint32_t timeRemaining = ak8963SensorQueuedReadTimeRemaining();
            if (timeRemaining) {
                return lastReadResult;
            }

            ack = ak8963SensorCompleteRead(magDev, &buf[0]);
        }
    }
#else
    ack = ak8963SensorRead(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_STATUS1, 1, &buf[0]);

    uint8_t status = buf[0];

    if (!ack || (status & STATUS1_DATA_READY) == 0) {
        lastReadResult = false;
        return lastReadResult;
    }

    ack = ak8963SensorRead(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_HXL, 7, &buf[0]);
#endif

    uint8_t status2 = buf[6];
    if (!ack || (status2 & STATUS2_DATA_ERROR) || (status2 & STATUS2_MAG_SENSOR_OVERFLOW)) {
        lastReadResult = false;
        return lastReadResult;
    }

    magDev->magADCRaw[X] = (int16_t)(buf[1] << 8 | buf[0]) * magGain[X];
    magDev->magADCRaw[Y] = (int16_t)(buf[3] << 8 | buf[2]) * magGain[Y];
    magDev->magADCRaw[Z] = (int16_t)(buf[5] << 8 | buf[4]) * magGain[Z];

#if defined(USE_SPI) && defined(MPU6500_SPI_INSTANCE)
    // cache mag data for reuse
    memcpy(cachedMagData, &magDev->magADCRaw, sizeof(cachedMagData));
    state = CHECK_STATUS;
    lastReadResult = true;
#else
    lastReadResult = ak8963SensorWrite(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, CNTL_MODE_ONCE); // start reading again
#endif

    return lastReadResult;
}

#define DETECTION_MAX_RETRY_COUNT   5
bool ak8963Detect(magDev_t *magDev)
{
    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        bool ack = false;
        uint8_t sig = 0;

#if defined(USE_SPI) && defined(MPU6500_SPI_INSTANCE)
        // Initialize I2C master via SPI bus (MPU9250)
        ack = mpuSpiWriteRegister(&magDev->bus, MPU_RA_INT_PIN_CFG, 0x10);               // INT_ANYRD_2CLEAR
        delay(10);

        ack = mpuSpiWriteRegister(&magDev->bus, MPU_RA_I2C_MST_CTRL, 0x0D);              // I2C multi-master / 400kHz
        delay(10);

        ack = mpuSpiWriteRegister(&magDev->bus, MPU_RA_USER_CTRL, 0x30);                 // I2C master mode, SPI mode only
        delay(10);
#endif

        // check for AK8963
        ack = ak8963SensorRead(magDev, AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_WHO_AM_I, 1, &sig);
        if (ack && sig == AK8963_Device_ID) { // 0x48 / 01001000 / 'H'
            magDev->init = ak8963Init;
            magDev->read = ak8963Read;
            return true;
        }
    }

    return false;
}
#endif
