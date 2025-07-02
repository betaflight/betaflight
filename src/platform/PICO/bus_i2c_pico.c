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

#include "pg/bus_i2c.h"

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
        .reg = i2c0,
    },
#endif
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = i2c1,
    },
#endif
};

void i2cPinConfigure(const i2cConfig_t *i2cConfig)
{
    for (int index = 0 ; index < I2CDEV_COUNT ; index++) {
        const i2cHardware_t *hardware = &i2cHardware[index];

        if (!hardware->reg) {
            continue;
        }

        I2CDevice device = hardware->device;
        i2cDevice_t *pDev = &i2cDevice[device];

        memset(pDev, 0, sizeof(*pDev));
        IO_t confSclIO = IOGetByTag(i2cConfig[device].ioTagScl);
        IO_t confSdaIO = IOGetByTag(i2cConfig[device].ioTagSda);
        int confSclPin = IO_GPIOPinIdx(confSclIO);
        int confSdaPin = IO_GPIOPinIdx(confSdaIO);

#ifdef RP2350B
        uint16_t numPins = 48;
#else
        uint16_t numPins = 30;
#endif

        // I2C0 on pins 0,1 mod 4, I2C1 on pins 2,3 mod 4
        // SDA on pins 0 mod 2, SCL on pins 1 mod 2
        int pinOffset = device == I2CDEV_0 ? 0 : 2;
        if (confSdaPin >= 0 && confSclPin >= 0 &&
            confSdaPin < numPins && confSclPin < numPins &&
            (confSdaPin % 4) == pinOffset && (confSclPin % 4) == (pinOffset + 1)) {
            pDev->scl = confSclIO;
            pDev->sda = confSdaIO;
            pDev->hardware = hardware;
            pDev->reg = hardware->reg;
            pDev->pullUp = i2cConfig[device].pullUp;
            pDev->clockSpeed = i2cConfig[device].clockSpeed;
        }
    }
}

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

    i2c_inst_t *port = I2C_INST(i2cHardware[device].reg);

    if (!port) {
        return false;
    }

    if (len_ > I2C_TX_BUFFER_LENGTH - 1) {
        return false; // Buffer too long
    }

    uint8_t buf[I2C_TX_BUFFER_LENGTH] = { reg_, 0 };
    memcpy(&buf[1], data, len_);
    bool nostop = false;
    int status = i2c_write_timeout_us(port, addr_, buf, len_ + 1, nostop, I2C_TIMEOUT_US);

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

    i2c_inst_t *port = I2C_INST(i2cHardware[device].reg);

    if (!port) {
        return false;
    }

    bool nostop = true;
    int status = i2c_write_timeout_us(port, addr_, &reg_, 1, nostop, I2C_TIMEOUT_US);
    if (status < 0) {
        return i2cHandleHardwareFailure(device);
    }

    nostop = false;
    status = i2c_read_timeout_us(port, addr_, buf, len, nostop, I2C_TIMEOUT_US);
    if (status < 0) {
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    // TODO: Implement genuine non-blocking read using DMA or similar mechanism
    // ( I2C0_IRQ, I2C1_IRQ ...)
    return i2cRead(device, addr_, reg_, len, buf);
}

bool i2cBusy(I2CDevice device, bool *error)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    i2c_inst_t *port = I2C_INST(i2cHardware[device].reg);

    if (!port) {
        return false;
    }

    if (error) {
        *error = 0;
    }

    // TODO check: If we are using DMA (for a sequence of transfers?), then we will need to
    // protect against that being in progress

    // Read the IC_STATUS register
    uint32_t status_reg = port->hw->status;

    // The bit for (combined master/slave) ACTIVITY is (1 << 0).
    return (status_reg & (1 << 0)) != 0;
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
    const uint8_t sclPin = IO_Pin(scl);
    const uint8_t sdaPin = IO_Pin(sda);

    if (!hardware || !scl || !sda) {
        return;
    }

    // Set owners
    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Initialise device
    i2c_init(I2C_INST(hardware->reg), 1000 * pDev->clockSpeed);

    // Set up GPIO pins for I2C
    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sclPin, GPIO_FUNC_I2C);

    // Enable internal pull-up resistors
    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);
}

#endif // #if defined(USE_I2C) && !defined(SOFT_I2C)
