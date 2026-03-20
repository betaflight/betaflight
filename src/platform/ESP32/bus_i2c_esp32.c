/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_I2C

#include "common/utils.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

const i2cHardware_t i2cHardware[] = { 0 };
i2cDevice_t i2cDevice[I2CDEV_COUNT] = { 0 };

void i2cInit(i2cDevice_e device)
{
    UNUSED(device);
    // TODO: i2c_master_bus_new()
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    UNUSED(device);
    if (error) {
        *error = false;
    }
    return false;
}

bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    UNUSED(device);
    UNUSED(addr_);
    UNUSED(reg_);
    UNUSED(data);
    return false;
}

bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    UNUSED(device);
    UNUSED(addr_);
    UNUSED(reg_);
    UNUSED(len_);
    UNUSED(data);
    return false;
}

bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    UNUSED(device);
    UNUSED(addr_);
    UNUSED(reg_);
    UNUSED(len);
    UNUSED(buf);
    return false;
}

bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    UNUSED(device);
    UNUSED(addr_);
    UNUSED(reg_);
    UNUSED(len);
    UNUSED(buf);
    return false;
}

void i2cPinConfigure(const struct i2cConfig_s *i2cConfig)
{
    UNUSED(i2cConfig);
}

uint16_t i2cGetErrorCounter(void)
{
    return 0;
}

#endif // USE_I2C
