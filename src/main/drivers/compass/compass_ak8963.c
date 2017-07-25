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

#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "drivers/compass/compass.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/compass/compass_ak8963.h"
#include "drivers/compass/compass_spi_ak8963.h"

static float magGain[3] = { 1.0f, 1.0f, 1.0f };

#if defined(MPU6500_SPI_INSTANCE) || defined(MPU9250_SPI_INSTANCE)
static busDevice_t *bus = NULL;

static bool spiWriteRegisterDelay(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    spiBusWriteRegister(bus, reg, data);
    delayMicroseconds(10);
    return true;
}

typedef struct queuedReadState_s {
    bool waiting;
    uint8_t len;
    uint32_t readStartedAt; // time read was queued in micros.
} queuedReadState_t;

typedef enum {
    CHECK_STATUS = 0,
    WAITING_FOR_STATUS,
    WAITING_FOR_DATA
} ak8963ReadState_e;

static queuedReadState_t queuedRead = { false, 0, 0};

static bool ak8963SensorRead(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *buf)
{
    spiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, addr_ | READ_FLAG);        // set I2C slave address for read
    spiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg_);                      // set I2C slave register
    spiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, len_ | 0x80);              // read number of bytes
    delay(4);
    __disable_irq();
    bool ack = spiBusReadRegisterBuffer(bus, MPU_RA_EXT_SENS_DATA_00, buf, len_);    // read I2C
    __enable_irq();
    return ack;
}

static bool ak8963SensorWrite(uint8_t addr_, uint8_t reg_, uint8_t data)
{
    spiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, addr_);                    // set I2C slave address for write
    spiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg_);                      // set I2C slave register
    spiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_DO, data);                       // set I2C salve value
    spiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, 0x81);                     // write 1 byte
    return true;
}

static bool ak8963SensorStartRead(uint8_t addr_, uint8_t reg_, uint8_t len_)
{
    if (queuedRead.waiting) {
        return false;
    }

    queuedRead.len = len_;

    spiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, addr_ | READ_FLAG);        // set I2C slave address for read
    spiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg_);                      // set I2C slave register
    spiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, len_ | 0x80);              // read number of bytes

    queuedRead.readStartedAt = micros();
    queuedRead.waiting = true;

    return true;
}

static uint32_t ak8963SensorQueuedReadTimeRemaining(void)
{
    if (!queuedRead.waiting) {
        return 0;
    }

    int32_t timeSinceStarted = micros() - queuedRead.readStartedAt;

    int32_t timeRemaining = 8000 - timeSinceStarted;

    if (timeRemaining < 0) {
        return 0;
    }

    return timeRemaining;
}

static bool ak8963SensorCompleteRead(uint8_t *buf)
{
    uint32_t timeRemaining = ak8963SensorQueuedReadTimeRemaining();

    if (timeRemaining > 0) {
        delayMicroseconds(timeRemaining);
    }

    queuedRead.waiting = false;

    spiBusReadRegisterBuffer(bus, MPU_RA_EXT_SENS_DATA_00, buf, queuedRead.len);               // read I2C buffer
    return true;
}
#else
static bool ak8963SensorRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return i2cRead(MAG_I2C_INSTANCE, addr_, reg_, len, buf);
}

static bool ak8963SensorWrite(uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWrite(MAG_I2C_INSTANCE, addr_, reg_, data);
}
#endif

static bool ak8963Init()
{
    uint8_t calibration[3];
    uint8_t status;

    ak8963SensorWrite(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN); // power down before entering fuse mode
    ak8963SensorWrite(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL1, CNTL1_MODE_FUSE_ROM); // Enter Fuse ROM access mode
    ak8963SensorRead(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_ASAX, sizeof(calibration), calibration); // Read the x-, y-, and z-axis calibration values

    magGain[X] = ((((float)(int8_t)calibration[X] - 128) / 256) + 1) * 30;
    magGain[Y] = ((((float)(int8_t)calibration[Y] - 128) / 256) + 1) * 30;
    magGain[Z] = ((((float)(int8_t)calibration[Z] - 128) / 256) + 1) * 30;

    ak8963SensorWrite(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN); // power down after reading.

    // Clear status registers
    ak8963SensorRead(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_ST1, 1, &status);
    ak8963SensorRead(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_ST2, 1, &status);

    // Trigger first measurement
    ak8963SensorWrite(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL1, CNTL1_MODE_ONCE);
    return true;
}

static bool ak8963Read(int16_t *magData)
{
    bool ack = false;
    uint8_t buf[7];

#if defined(MPU6500_SPI_INSTANCE) || defined(MPU9250_SPI_INSTANCE)

    // we currently need a different approach for the MPU9250 connected via SPI.
    // we cannot use the ak8963SensorRead() method for SPI, it is to slow and blocks for far too long.

    static ak8963ReadState_e state = CHECK_STATUS;

    bool retry = true;

restart:
    switch (state) {
        case CHECK_STATUS:
            ak8963SensorStartRead(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_ST1, 1);
            state++;
            return false;

        case WAITING_FOR_STATUS: {
            uint32_t timeRemaining = ak8963SensorQueuedReadTimeRemaining();
            if (timeRemaining) {
                return false;
            }

            ack = ak8963SensorCompleteRead(&buf[0]);

            uint8_t status = buf[0];

            if (!ack || (status & ST1_DATA_READY) == 0) {
                // too early. queue the status read again
                state = CHECK_STATUS;
                if (retry) {
                    retry = false;
                    goto restart;
                }
                return false;
            }


            // read the 6 bytes of data and the status2 register
            ak8963SensorStartRead(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_HXL, 7);

            state++;

            return false;
        }

        case WAITING_FOR_DATA: {
            uint32_t timeRemaining = ak8963SensorQueuedReadTimeRemaining();
            if (timeRemaining) {
                return false;
            }

            ack = ak8963SensorCompleteRead(&buf[0]);
        }
    }
#else
    ack = ak8963SensorRead(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_ST1, 1, &buf[0]);

    uint8_t status = buf[0];

    if (!ack || (status & ST1_DATA_READY) == 0) {
        return false;
    }

    ack = ak8963SensorRead(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_HXL, 7, &buf[0]);
#endif
    uint8_t status2 = buf[6];
    if (!ack || (status2 & ST2_DATA_ERROR) || (status2 & ST2_MAG_SENSOR_OVERFLOW)) {
        return false;
    }

    magData[X] = -(int16_t)(buf[1] << 8 | buf[0]) * magGain[X];
    magData[Y] = -(int16_t)(buf[3] << 8 | buf[2]) * magGain[Y];
    magData[Z] = -(int16_t)(buf[5] << 8 | buf[4]) * magGain[Z];

#if defined(MPU6500_SPI_INSTANCE) || defined(MPU9250_SPI_INSTANCE)
    state = CHECK_STATUS;
#endif
    return ak8963SensorWrite(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL1, CNTL1_MODE_ONCE); // start reading again
}

bool ak8963Detect(magDev_t *mag)
{
    uint8_t sig = 0;

#if defined(USE_SPI) && defined(AK8963_SPI_INSTANCE)
    spiBusSetInstance(&mag->bus, AK8963_SPI_INSTANCE);
    mag->bus.busdev_u.spi.csnPin = mag->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(AK8963_CS_PIN)) : mag->bus.busdev_u.spi.csnPin;

    // check for SPI AK8963
    if (ak8963SpiDetect(mag)) return true;
#endif

#if defined(MPU6500_SPI_INSTANCE) || defined(MPU9250_SPI_INSTANCE)
    bus = &mag->bus;
#if defined(MPU6500_SPI_INSTANCE)
    spiBusSetInstance(&mag->bus, MPU6500_SPI_INSTANCE);
#elif defined(MPU9250_SPI_INSTANCE)
    spiBusSetInstance(&mag->bus, MPU9250_SPI_INSTANCE);
#endif

    // initialze I2C master via SPI bus (MPU9250)
    spiWriteRegisterDelay(&mag->bus, MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR | MPU6500_BIT_BYPASS_EN);
    spiWriteRegisterDelay(&mag->bus, MPU_RA_I2C_MST_CTRL, 0x0D);                // I2C multi-master / 400kHz
    spiWriteRegisterDelay(&mag->bus, MPU_RA_USER_CTRL, 0x30);                   // I2C master mode, SPI mode only
#endif

    ak8963SensorWrite(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL2, CNTL2_SOFT_RESET); // reset MAG
    delay(4);

    bool ack = ak8963SensorRead(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_WIA, 1, &sig);  // check for AK8963
    if (ack && sig == AK8963_Device_ID) // 0x48 / 01001000 / 'H'
    {
        mag->init = ak8963Init;
        mag->read = ak8963Read;

        return true;
    }
    return false;
}
