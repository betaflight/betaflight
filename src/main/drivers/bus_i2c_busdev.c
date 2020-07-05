/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C)

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"

static uint8_t i2cRegisteredDeviceCount = 0;

bool i2cBusWriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
    return i2cWrite(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, data);
}

bool i2cBusWriteRegisterStart(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
    // Need a static value, not on the stack
    static uint8_t byte;

    byte = data;

    return i2cWriteBuffer(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, sizeof (byte), &byte);
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

bool i2cBusReadRegisterBufferStart(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return i2cReadBuffer(busdev->busdev_u.i2c.device, busdev->busdev_u.i2c.address, reg, length, data);
}

bool i2cBusBusy(const busDevice_t *busdev, bool *error)
{
    return i2cBusy(busdev->busdev_u.i2c.device, error);
}

void i2cBusDeviceRegister(const busDevice_t *busdev)
{
    UNUSED(busdev);

    i2cRegisteredDeviceCount++;
}

uint8_t i2cGetRegisteredDeviceCount(void)
{
    return i2cRegisteredDeviceCount;
}
#endif
