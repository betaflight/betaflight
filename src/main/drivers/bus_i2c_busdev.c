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

#include <platform.h>

#if defined(USE_I2C)

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"

bool i2cBusWriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
    return i2cWrite(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, data);
}

bool i2cBusReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return i2cRead(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, length, data);
}

uint8_t i2cBusReadRegister(const busDevice_t *busdev, uint8_t reg)
{
    uint8_t data;
    i2cRead(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, 1, &data);
    return data;
}
#endif
