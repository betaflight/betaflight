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
#include "drivers/bus_i2c_busdev.h"
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

// This sensor is available in MPU-9250.

// AK8963
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
#define AK8963_MAG_REG_CNTL1            0x0a
#define AK8963_MAG_REG_CNTL2            0x0b
#define AK8963_MAG_REG_ASCT             0x0c // self test
#define AK8963_MAG_REG_ASAX             0x10 // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAY             0x11 // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAZ             0x12 // Fuse ROM z-axis sensitivity adjustment value

#define READ_FLAG                       0x80

#define STATUS1_DATA_READY              0x01
#define STATUS1_DATA_OVERRUN            0x02

#define STATUS2_DATA_ERROR              0x02
#define STATUS2_MAG_SENSOR_OVERFLOW     0x03

#define CNTL1_MODE_POWER_DOWN           0x00
#define CNTL1_MODE_ONCE                 0x01
#define CNTL1_MODE_CONT1                0x02
#define CNTL1_MODE_CONT2                0x06
#define CNTL1_MODE_SELF_TEST            0x08
#define CNTL1_MODE_FUSE_ROM             0x0F

#define CNTL2_SOFT_RESET                0x01

static float magGain[3] = { 1.0f, 1.0f, 1.0f };

#if defined(MPU6500_SPI_INSTANCE) || defined(MPU9250_SPI_INSTANCE)

static bool ak8963SpiBusWriteRegisterDelay(const busDevice_t *bus, uint8_t reg, uint8_t data)
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

static bool ak8963SlaveSensorRead(busDevice_t *slavedev, uint8_t reg_, uint8_t *buf, uint8_t len_)
{
    const busDevice_t *bus = slavedev->busdev_u.i2c.master;

    ak8963SpiBusWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, slavedev->busdev_u.i2c.address | READ_FLAG);        // set I2C slave address for read
    ak8963SpiBusWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg_);                      // set I2C slave register
    ak8963SpiBusWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, len_ | 0x80);              // read number of bytes
    delay(4);
    __disable_irq();
    bool ack = spiBusReadRegisterBuffer(bus, MPU_RA_EXT_SENS_DATA_00, buf, len_);    // read I2C
    __enable_irq();
    return ack;
}

static bool ak8963SlaveSensorWrite(busDevice_t *slavedev, uint8_t reg_, uint8_t data)
{
    const busDevice_t *bus = slavedev->busdev_u.i2c.master;

    ak8963SpiBusWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, slavedev->busdev_u.i2c.address);                    // set I2C slave address for write
    ak8963SpiBusWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg_);                      // set I2C slave register
    ak8963SpiBusWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_DO, data);                       // set I2C salve value
    ak8963SpiBusWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, 0x81);                     // write 1 byte
    return true;
}

static bool ak8963SensorStartRead(busDevice_t *slavedev, uint8_t reg_, uint8_t len_)
{
    if (queuedRead.waiting) {
        return false;
    }

    const busDevice_t *bus = slavedev->busdev_u.i2c.master;

    queuedRead.len = len_;

    ak8963SpiBusWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, slavedev->busdev_u.i2c.address | READ_FLAG);        // set I2C slave address for read
    ak8963SpiBusWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg_);                      // set I2C slave register
    ak8963SpiBusWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, len_ | 0x80);              // read number of bytes

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

static bool ak8963SensorCompleteRead(busDevice_t *slavedev, uint8_t *buf)
{
    const busDevice_t *bus = slavedev->busdev_u.i2c.master;

    uint32_t timeRemaining = ak8963SensorQueuedReadTimeRemaining();

    if (timeRemaining > 0) {
        delayMicroseconds(timeRemaining);
    }

    queuedRead.waiting = false;

    spiBusReadRegisterBuffer(bus, MPU_RA_EXT_SENS_DATA_00, buf, queuedRead.len);               // read I2C buffer
    return true;
}
#endif

static bool ak8963SensorRead(busDevice_t *busdev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    switch (busdev->bustype) {
    case BUSTYPE_I2C:
        return i2cBusReadRegisterBuffer(busdev, reg, buf, len);

    case BUSTYPE_SPI:
        return spiBusReadRegisterBuffer(busdev, reg, buf, len);

#if defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)
    case BUSTYPE_SLAVE:
        return ak8963SlaveSensorRead(busdev, reg, buf, len);
#endif
    }
    return false;
}

static bool ak8963SensorWrite(busDevice_t *busdev, uint8_t reg, uint8_t data)
{
    switch (busdev->bustype) {
    case BUSTYPE_I2C:
        return i2cBusWriteRegister(busdev, reg, data);

    case BUSTYPE_SPI:
        return i2cBusWriteRegister(busdev, reg, data);

#if defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)
    case BUSTYPE_SLAVE:
        return ak8963SlaveSensorWrite(busdev, reg, data);
#endif
    }
    return false;
}

static bool ak8963Init(magDev_t *mag)
{
    uint8_t calibration[3];
    uint8_t status;

    busDevice_t *busdev = &mag->busdev;

    ak8963SensorWrite(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN); // power down before entering fuse mode
    ak8963SensorWrite(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_FUSE_ROM); // Enter Fuse ROM access mode
    ak8963SensorRead(busdev, AK8963_MAG_REG_ASAX, calibration, sizeof(calibration)); // Read the x-, y-, and z-axis calibration values

    magGain[X] = (((((float)(int8_t)calibration[X] - 128) / 256) + 1) * 30);
    magGain[Y] = (((((float)(int8_t)calibration[Y] - 128) / 256) + 1) * 30);
    magGain[Z] = (((((float)(int8_t)calibration[Z] - 128) / 256) + 1) * 30);

    ak8963SensorWrite(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN); // power down after reading.

    // Clear status registers
    ak8963SensorRead(busdev, AK8963_MAG_REG_STATUS1, &status, 1);
    ak8963SensorRead(busdev, AK8963_MAG_REG_STATUS2, &status, 1);

    // Trigger first measurement
    ak8963SensorWrite(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_ONCE);
    return true;
}

static bool ak8963Read(magDev_t *mag, int16_t *magData)
{
    bool ack = false;
    uint8_t buf[7];

    busDevice_t *busdev = &mag->busdev;

#if defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)

    static ak8963ReadState_e state = CHECK_STATUS;

    if (busdev->bustype == BUSTYPE_SLAVE) {

        // we currently need a different approach for the MPU9250 connected via SPI.
        // we cannot use the ak8963SensorRead() method for SPI, it is to slow and blocks for far too long.
        bool retry = true;

    restart:
        switch (state) {
            case CHECK_STATUS:
                ak8963SensorStartRead(busdev, AK8963_MAG_REG_STATUS1, 1);
                state++;
                return false;

            case WAITING_FOR_STATUS: {
                uint32_t timeRemaining = ak8963SensorQueuedReadTimeRemaining();
                if (timeRemaining) {
                    return false;
                }

                ack = ak8963SensorCompleteRead(busdev, &buf[0]);

                uint8_t status = buf[0];

                if (!ack || (status & STATUS1_DATA_READY) == 0) {
                    // too early. queue the status read again
                    state = CHECK_STATUS;
                    if (retry) {
                        retry = false;
                        goto restart;
                    }
                    return false;
                }

                // read the 6 bytes of data and the status2 register
                ak8963SensorStartRead(busdev, AK8963_MAG_REG_HXL, 7);

                state++;

                return false;
            }

            case WAITING_FOR_DATA: {
                uint32_t timeRemaining = ak8963SensorQueuedReadTimeRemaining();
                if (timeRemaining) {
                    return false;
                }

                ack = ak8963SensorCompleteRead(busdev, &buf[0]);
            }
        }
    } else
#endif
    {
        ack = ak8963SensorRead(busdev, AK8963_MAG_REG_STATUS1, &buf[0], 1);

        uint8_t status = buf[0];

        if (!ack || (status & STATUS1_DATA_READY) == 0) {
            return false;
        }

        ack = ak8963SensorRead(busdev, AK8963_MAG_REG_HXL, &buf[0], 7);
    }

    uint8_t status2 = buf[6];
    if (!ack || (status2 & STATUS2_DATA_ERROR) || (status2 & STATUS2_MAG_SENSOR_OVERFLOW)) {
        return false;
    }

    magData[X] = -(int16_t)(buf[1] << 8 | buf[0]) * magGain[X];
    magData[Y] = -(int16_t)(buf[3] << 8 | buf[2]) * magGain[Y];
    magData[Z] = -(int16_t)(buf[5] << 8 | buf[4]) * magGain[Z];

#if defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)
    state = CHECK_STATUS;
#endif
    return ak8963SensorWrite(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_ONCE); // start reading again
}

bool ak8963Detect(magDev_t *mag)
{
    uint8_t sig = 0;
    busDevice_t *bus = &mag->busdev;

#if defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)
    if (bus->bustype == BUSTYPE_SLAVE) {
        // initialze I2C master via SPI bus (MPU9250)
        const busDevice_t *masterbus = bus->busdev_u.i2c.master;

        if (!masterbus) {
            return false;
        }

        ak8963SpiBusWriteRegisterDelay(masterbus, MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR | MPU6500_BIT_BYPASS_EN);
        ak8963SpiBusWriteRegisterDelay(masterbus, MPU_RA_I2C_MST_CTRL, 0x0D);                // I2C multi-master / 400kHz
        ak8963SpiBusWriteRegisterDelay(masterbus, MPU_RA_USER_CTRL, 0x30);                   // I2C master mode, SPI mode only
    }
#endif

    if ((bus->bustype == BUSTYPE_I2C || bus->bustype == BUSTYPE_SLAVE) && bus->busdev_u.i2c.address == 0) {
        bus->busdev_u.i2c.address = AK8963_MAG_I2C_ADDRESS;
    }

    ak8963SensorWrite(bus, AK8963_MAG_REG_CNTL2, CNTL2_SOFT_RESET); // reset MAG
    delay(4);

    bool ack = ak8963SensorRead(bus, AK8963_MAG_REG_WHO_AM_I, &sig, 1);  // check for AK8963
    if (ack && sig == AK8963_Device_ID) // 0x48 / 01001000 / 'H'
    {
        mag->init = ak8963Init;
        mag->read = ak8963Read;
        return true;
    }
    return false;
}
