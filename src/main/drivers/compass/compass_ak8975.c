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

#ifdef USE_MAG_AK8975

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/gpio.h"
#include "drivers/bus_i2c.h"

#include "drivers/sensor.h"
#include "drivers/compass/compass.h"

#include "drivers/compass/compass_ak8975.h"

// This sensor is available in MPU-9150.

// AK8975, mag sensor address
#define AK8975_MAG_I2C_ADDRESS      0x0C


// Registers
#define AK8975_MAG_REG_WHO_AM_I     0x00
#define AK8975_MAG_REG_INFO         0x01
#define AK8975_MAG_REG_STATUS1      0x02
#define AK8975_MAG_REG_HXL          0x03
#define AK8975_MAG_REG_HXH          0x04
#define AK8975_MAG_REG_HYL          0x05
#define AK8975_MAG_REG_HYH          0x06
#define AK8975_MAG_REG_HZL          0x07
#define AK8975_MAG_REG_HZH          0x08
#define AK8975_MAG_REG_STATUS2      0x09
#define AK8975_MAG_REG_CNTL         0x0a
#define AK8975_MAG_REG_ASCT         0x0c // self test

#define AK8975A_ASAX 0x10 // Fuse ROM x-axis sensitivity adjustment value
#define AK8975A_ASAY 0x11 // Fuse ROM y-axis sensitivity adjustment value
#define AK8975A_ASAZ 0x12 // Fuse ROM z-axis sensitivity adjustment value

static bool ak8975Init(magDev_t *magDev)
{
    bool ack;
    uint8_t buffer[3];
    uint8_t status;

    UNUSED(ack);
    UNUSED(magDev);

    ack = i2cWrite(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_CNTL, 0x00); // power down before entering fuse mode
    delay(20);

    ack = i2cWrite(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(10);

    ack = i2cRead(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975A_ASAX, 3, &buffer[0]); // Read the x-, y-, and z-axis calibration values
    delay(10);

    ack = i2cWrite(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_CNTL, 0x00); // power down after reading.
    delay(10);

    // Clear status registers
    ack = i2cRead(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_STATUS1, 1, &status);
    ack = i2cRead(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_STATUS2, 1, &status);

    // Trigger first measurement
    ack = i2cWrite(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_CNTL, 0x01);
    return true;
}

#define BIT_STATUS1_REG_DATA_READY              (1 << 0)

#define BIT_STATUS2_REG_DATA_ERROR              (1 << 2)
#define BIT_STATUS2_REG_MAG_SENSOR_OVERFLOW     (1 << 3)

static bool ak8975Read(magDev_t *magDev)
{
    uint8_t status;
    uint8_t buf[6];

    // set magData to zero for case of failed read
    magDev->magADCRaw[X] = 0;
    magDev->magADCRaw[Y] = 0;
    magDev->magADCRaw[Z] = 0;

    bool ack = i2cRead(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_STATUS1, 1, &status);
    if (!ack || (status & BIT_STATUS1_REG_DATA_READY) == 0) {
        return false;
    }

#if 1 // USE_I2C_SINGLE_BYTE_READS
    ack = i2cRead(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_HXL, 6, buf); // read from AK8975_MAG_REG_HXL to AK8975_MAG_REG_HZH
#else
    for (uint8_t i = 0; i < 6; i++) {
        ack = i2cRead(AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_HXL + i, 1, &buf[i]); // read from AK8975_MAG_REG_HXL to AK8975_MAG_REG_HZH
        if (!ack) {
            return false
        }
    }
#endif

    ack = i2cRead(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_STATUS2, 1, &status);
    if (!ack || (status & BIT_STATUS2_REG_DATA_ERROR) || (status & BIT_STATUS2_REG_MAG_SENSOR_OVERFLOW)) {
        return false;
    }

    magDev->magADCRaw[X] = -(int16_t)(buf[1] << 8 | buf[0]) * 4;
    magDev->magADCRaw[Y] = -(int16_t)(buf[3] << 8 | buf[2]) * 4;
    magDev->magADCRaw[Z] = -(int16_t)(buf[5] << 8 | buf[4]) * 4;


    ack = i2cWrite(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_CNTL, 0x01); // start reading again
    return true;
}

#define DETECTION_MAX_RETRY_COUNT   5
bool ak8975Detect(magDev_t *magDev)
{
    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        uint8_t sig = 0;
        bool ack = i2cRead(MAG_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_WHO_AM_I, 1, &sig);
        if (ack && sig == 'H') { // 0x48 / 01001000 / 'H'
            magDev->init = ak8975Init;
            magDev->read = ak8975Read;
            return true;
        }
    }

    return false;
}
#endif
