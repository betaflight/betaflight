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

#include "build_config.h"

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "gpio.h"
#include "bus_i2c.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"

#include "sensor.h"
#include "compass.h"

#include "compass_mag3110.h"

#ifdef USE_MAG_MAG3110

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

bool mag3110detect(mag_t *mag)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(MAG3110_MAG_I2C_ADDRESS, MAG3110_MAG_REG_WHO_AM_I, 1, &sig);
    if (!ack || sig != 0xC4)
        return false;

    mag->init = mag3110Init;
    mag->read = mag3110Read;

    return true;
}

void mag3110Init()
{
    bool ack;
    UNUSED(ack);

    ack = i2cWrite(MAG3110_MAG_I2C_ADDRESS, MAG3110_MAG_REG_CTRL_REG1, 0x01); //  active mode 80 Hz ODR with OSR = 1
    delay(20);

    ack = i2cWrite(MAG3110_MAG_I2C_ADDRESS, MAG3110_MAG_REG_CTRL_REG2, 0xA0); // AUTO_MRST_EN + RAW
    delay(10);
}

#define BIT_STATUS_REG_DATA_READY               (1 << 3)

bool mag3110Read(int16_t *magData)
{
    bool ack;
    UNUSED(ack);
    uint8_t status;
    uint8_t buf[6];

    ack = i2cRead(MAG3110_MAG_I2C_ADDRESS, MAG3110_MAG_REG_STATUS, 1, &status);
    if (!ack || (status & BIT_STATUS_REG_DATA_READY) == 0) {
        return false;
    }

    ack = i2cRead(MAG3110_MAG_I2C_ADDRESS, MAG3110_MAG_REG_HXL, 6, buf);

    magData[X] = (int16_t)(buf[0] << 8 | buf[1]);
    magData[Y] = (int16_t)(buf[2] << 8 | buf[3]);
    magData[Z] = (int16_t)(buf[4] << 8 | buf[5]);

    return true;
}

#endif
