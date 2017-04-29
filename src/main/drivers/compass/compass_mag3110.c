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

#ifdef USE_MAG_MAG3110

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/gpio.h"
#include "drivers/bus_i2c.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"

#include "drivers/sensor.h"
#include "drivers/compass/compass.h"

#include "drivers/compass/compass_mag3110.h"


#define MAG3110_MAG_I2C_ADDRESS     0x0E


// Registers
#define MAG3110_MAG_REG_STATUS       0x00
#define MAG3110_MAG_REG_HXL          0x01
#define MAG3110_MAG_REG_HXH          0x02
#define MAG3110_MAG_REG_HYL          0x03
#define MAG3110_MAG_REG_HYH          0x04
#define MAG3110_MAG_REG_HZL          0x05
#define MAG3110_MAG_REG_HZH          0x06
#define MAG3110_MAG_REG_WHO_AM_I     0x07
#define MAG3110_MAG_REG_SYSMODE      0x08
#define MAG3110_MAG_REG_CTRL_REG1    0x10
#define MAG3110_MAG_REG_CTRL_REG2    0x11

static bool mag3110Init(magDev_t *magDev)
{
    UNUSED(magDev);

    bool ack = i2cWrite(MAG_I2C_INSTANCE, MAG3110_MAG_I2C_ADDRESS, MAG3110_MAG_REG_CTRL_REG1, 0x01); //  active mode 80 Hz ODR with OSR = 1
    delay(20);
    if (!ack) {
        return false;
    }

    ack = i2cWrite(MAG_I2C_INSTANCE, MAG3110_MAG_I2C_ADDRESS, MAG3110_MAG_REG_CTRL_REG2, 0xA0); // AUTO_MRST_EN + RAW
    delay(10);
    if (!ack) {
        return false;
    }

    return true;
}

#define BIT_STATUS_REG_DATA_READY               (1 << 3)

static bool mag3110Read(magDev_t *magDev)
{
    uint8_t status;
    uint8_t buf[6];

    // set magData to zero for case of failed read
    magDev->magADCRaw[X] = 0;
    magDev->magADCRaw[Y] = 0;
    magDev->magADCRaw[Z] = 0;

    bool ack = i2cRead(MAG_I2C_INSTANCE, MAG3110_MAG_I2C_ADDRESS, MAG3110_MAG_REG_STATUS, 1, &status);
    if (!ack || (status & BIT_STATUS_REG_DATA_READY) == 0) {
        return false;
    }

    ack = i2cRead(MAG_I2C_INSTANCE, MAG3110_MAG_I2C_ADDRESS, MAG3110_MAG_REG_HXL, 6, buf);
    if (!ack) {
        return false;
    }

    magDev->magADCRaw[X] = (int16_t)(buf[0] << 8 | buf[1]);
    magDev->magADCRaw[Y] = (int16_t)(buf[2] << 8 | buf[3]);
    magDev->magADCRaw[Z] = (int16_t)(buf[4] << 8 | buf[5]);

    return true;
}

#define DETECTION_MAX_RETRY_COUNT   5
bool mag3110detect(magDev_t *magDev)
{
    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        uint8_t sig = 0;
        bool ack = i2cRead(MAG_I2C_INSTANCE, MAG3110_MAG_I2C_ADDRESS, MAG3110_MAG_REG_WHO_AM_I, 1, &sig);
        if (ack && sig == 0xC4) {
            magDev->init = mag3110Init;
            magDev->read = mag3110Read;
            return true;
        }
    }

    return false;
}
#endif
