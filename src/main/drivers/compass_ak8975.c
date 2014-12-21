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

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "gpio.h"
#include "bus_i2c.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"

#include "sensor.h"
#include "compass.h"

#include "compass_ak8975.h"

// This sensor is available in MPU-9150.

// AK8975, mag sensor address
#define AK8975_MAG_I2C_ADDRESS     0x0C
#define AK8975_MAG_ID_ADDRESS      0x00
#define AK8975_MAG_DATA_ADDRESS    0x03
#define AK8975_MAG_CONTROL_ADDRESS 0x0A

bool ak8975detect(mag_t *mag)
{
    bool ack = false;
    uint8_t sig = 0;
    uint8_t ackCount = 0;

    for (uint8_t address = 0; address < 0xff; address++) {
        ack = i2cRead(address, AK8975_MAG_ID_ADDRESS, 1, &sig);
        if (ack) {
            ackCount++;
        }
    }
    // device ID is in register 0 and is equal to 'H'
    ack = i2cRead(AK8975_MAG_I2C_ADDRESS, AK8975_MAG_ID_ADDRESS, 1, &sig);
    if (!ack || sig != 'H')
        return false;

    mag->init = ak8975Init;
    mag->read = ak8975Read;

    return true;
}

void ak8975Init()
{
    i2cWrite(AK8975_MAG_I2C_ADDRESS, AK8975_MAG_CONTROL_ADDRESS, 0x01); // start reading
}

void ak8975Read(int16_t *magData)
{
    uint8_t buf[6];

    i2cRead(AK8975_MAG_I2C_ADDRESS, AK8975_MAG_DATA_ADDRESS, 6, buf);
    // align sensors to match MPU6050:
    // x -> y
    // y -> x
    // z-> -z
    magData[X] = -(int16_t)(buf[3] << 8 | buf[2]) * 4;
    magData[Y] = -(int16_t)(buf[1] << 8 | buf[0]) * 4;
    magData[Z] = -(int16_t)(buf[5] << 8 | buf[4]) * 4;

    i2cWrite(AK8975_MAG_I2C_ADDRESS, AK8975_MAG_CONTROL_ADDRESS, 0x01); // start reading again
}
