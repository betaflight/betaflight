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
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"

#define I2C_TX_BUFFER_LENGTH 32

static volatile uint16_t i2cErrorCount = 0;

i2cDevice_t i2cDevice[I2CDEV_COUNT];

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_0
    {
        .device = I2CDEV_0,
        .reg = I2C0,
        .sclPins = {
            { DEFIO_TAG_E(PA1) },
            { DEFIO_TAG_E(PA5) },
            { DEFIO_TAG_E(PA9) },
            { DEFIO_TAG_E(PA13) },
        },
        .sdaPins = {
            { DEFIO_TAG_E(PA0) },
            { DEFIO_TAG_E(PA4) },
            { DEFIO_TAG_E(PA8) },
            { DEFIO_TAG_E(PA12) },
        },
    },
#endif
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = {
            { DEFIO_TAG_E(PA3) },
            { DEFIO_TAG_E(PA7) },
            { DEFIO_TAG_E(PA11) },
            { DEFIO_TAG_E(PA15) },
        },
        .sdaPins = {
            { DEFIO_TAG_E(PA2) },
            { DEFIO_TAG_E(PA6) },
            { DEFIO_TAG_E(PA10) },
            { DEFIO_TAG_E(PA14) },
        }
    },
#endif
};

static bool i2cHandleHardwareFailure(I2CDevice device)
{
    UNUSED(device);
    i2cErrorCount++;
    return false;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    // TODO: Implement non-blocking write using DMA or similar mechanism
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    i2c_inst_t *port = I2C_INST(&i2cHardware[device].reg);

    if (!port) {
        return false;
    }

    if (len_ > I2C_TX_BUFFER_LENGTH - 1) {
        return false; // Buffer too long
    }

    uint8_t buf[I2C_TX_BUFFER_LENGTH] = { reg_, 0 };
    memcpy(&buf[1], data, len_);
    int status = i2c_write_timeout_us(port, addr_ << 1, buf, len_ + 1, true, I2C_TIMEOUT_US);

    if (status < 0) {
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    i2c_inst_t *port = I2C_INST(&i2cHardware[device].reg);

    if (!port) {
        return false;
    }

    int status = i2c_write_timeout_us(port, addr_ << 1, &reg_, 1, true, I2C_TIMEOUT_US);
    if (status < 0) {
        return i2cHandleHardwareFailure(device);
    }

    status = i2c_read_timeout_us(port, addr_ << 1, buf, len, true, I2C_TIMEOUT_US);
    if (status < 0) {
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    // TODO: Implement genuine non-blocking read using DMA or similar mechanism
    return i2cRead(device, addr_, reg_, len, buf);
}

bool i2cBusy(I2CDevice device, bool *error)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    i2c_inst_t *port = I2C_INST(&i2cHardware[device].reg);

    if (!port) {
        return false;
    }

    if (error) {
        *error = 0;
    }

    // Read the IC_STATUS register
    uint32_t status_reg = port->hw->status;

    // The bit for MST_ACTIVITY is (1 << 5).
    return (status_reg & (1 << 5)) != 0;
}

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID) {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];

    const i2cHardware_t *hardware = pDev->hardware;
    const IO_t scl = pDev->scl;
    const IO_t sda = pDev->sda;

    if (!hardware || IOGetOwner(scl) || IOGetOwner(sda)) {
        return;
    }

    i2c_init(I2C_INST(hardware->reg), pDev->clockSpeed);

    // Set up GPIO pins for I2C
    gpio_set_function(IO_Pin(sda), GPIO_FUNC_I2C);
    gpio_set_function(IO_Pin(scl), GPIO_FUNC_I2C);

    // Enable internal pull-up resistors
    gpio_pull_up(IO_Pin(sda));
    gpio_pull_up(IO_Pin(scl));
}

#endif
