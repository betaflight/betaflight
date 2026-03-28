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

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

// I2C event IRQ handler
static void i2cEVIRQHandler(i2cDevice_e device)
{
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].hardware->reg;
    i2cState_t *state = &i2cDevice[device].state;

    // NACK received
    if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
        LL_I2C_ClearFlag_NACK(I2Cx);
        state->error = true;
        state->busy = false;
        // Disable all transfer interrupts
        LL_I2C_DisableIT_TX(I2Cx);
        LL_I2C_DisableIT_RX(I2Cx);
        LL_I2C_DisableIT_TC(I2Cx);
        LL_I2C_DisableIT_STOP(I2Cx);
        LL_I2C_DisableIT_NACK(I2Cx);
        LL_I2C_DisableIT_ERR(I2Cx);
        return;
    }

    // Transmit data register empty
    if (LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
        if (state->writing && state->bytes > 0) {
            LL_I2C_TransmitData8(I2Cx, *state->write_p++);
            state->bytes--;
        }
        return;
    }

    // Receive data register not empty
    if (LL_I2C_IsActiveFlag_RXNE(I2Cx)) {
        if (state->reading && state->bytes > 0) {
            *state->read_p++ = LL_I2C_ReceiveData8(I2Cx);
            state->bytes--;
        }
        return;
    }

    // Transfer complete (used for restart between write-reg-addr and read-data)
    if (LL_I2C_IsActiveFlag_TC(I2Cx)) {
        if (state->reading) {
            // Register address phase complete, now start read phase
            LL_I2C_HandleTransfer(I2Cx, state->addr, LL_I2C_ADDRSLAVE_7BIT,
                                  state->bytes, LL_I2C_MODE_AUTOEND,
                                  LL_I2C_GENERATE_START_READ);
            // Switch from TX to RX interrupts
            LL_I2C_DisableIT_TX(I2Cx);
            LL_I2C_EnableIT_RX(I2Cx);
        }
        return;
    }

    // Stop condition detected
    if (LL_I2C_IsActiveFlag_STOP(I2Cx)) {
        LL_I2C_ClearFlag_STOP(I2Cx);
        // Disable all transfer interrupts
        LL_I2C_DisableIT_TX(I2Cx);
        LL_I2C_DisableIT_RX(I2Cx);
        LL_I2C_DisableIT_TC(I2Cx);
        LL_I2C_DisableIT_STOP(I2Cx);
        LL_I2C_DisableIT_NACK(I2Cx);
        LL_I2C_DisableIT_ERR(I2Cx);
        state->busy = false;
        return;
    }
}

// I2C error IRQ handler
static void i2cERIRQHandler(i2cDevice_e device)
{
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].hardware->reg;
    i2cState_t *state = &i2cDevice[device].state;

    if (LL_I2C_IsActiveFlag_BERR(I2Cx)) {
        LL_I2C_ClearFlag_BERR(I2Cx);
        state->error = true;
    }

    if (LL_I2C_IsActiveFlag_ARLO(I2Cx)) {
        LL_I2C_ClearFlag_ARLO(I2Cx);
        state->error = true;
    }

    if (LL_I2C_IsActiveFlag_OVR(I2Cx)) {
        LL_I2C_ClearFlag_OVR(I2Cx);
        state->error = true;
    }

    if (state->error) {
        // Disable all transfer interrupts
        LL_I2C_DisableIT_TX(I2Cx);
        LL_I2C_DisableIT_RX(I2Cx);
        LL_I2C_DisableIT_TC(I2Cx);
        LL_I2C_DisableIT_STOP(I2Cx);
        LL_I2C_DisableIT_NACK(I2Cx);
        LL_I2C_DisableIT_ERR(I2Cx);
        state->busy = false;
    }
}

#ifdef USE_I2C_DEVICE_1
void I2C1_ER_IRQHandler(void)
{
    i2cERIRQHandler(I2CDEV_1);
}

void I2C1_EV_IRQHandler(void)
{
    i2cEVIRQHandler(I2CDEV_1);
}
#endif

#ifdef USE_I2C_DEVICE_2
void I2C2_ER_IRQHandler(void)
{
    i2cERIRQHandler(I2CDEV_2);
}

void I2C2_EV_IRQHandler(void)
{
    i2cEVIRQHandler(I2CDEV_2);
}
#endif

#ifdef USE_I2C_DEVICE_3
void I2C3_ER_IRQHandler(void)
{
    i2cERIRQHandler(I2CDEV_3);
}

void I2C3_EV_IRQHandler(void)
{
    i2cEVIRQHandler(I2CDEV_3);
}
#endif

#ifdef USE_I2C_DEVICE_4
void I2C4_ER_IRQHandler(void)
{
    i2cERIRQHandler(I2CDEV_4);
}

void I2C4_EV_IRQHandler(void)
{
    i2cEVIRQHandler(I2CDEV_4);
}
#endif

static volatile uint16_t i2cErrorCount = 0;

static bool i2cHandleHardwareFailure(i2cDevice_e device)
{
    (void)device;
    i2cErrorCount++;
    return false;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

// Blocking write
bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    if (reg_ == 0xFF) {
        // No register address, just send data byte
        LL_I2C_HandleTransfer(I2Cx, addr_ << 1, LL_I2C_ADDRSLAVE_7BIT,
                              1, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_WRITE);

        // Wait for TXIS
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device);
            }
        }
        LL_I2C_TransmitData8(I2Cx, data);
    } else {
        // Send register address + data byte
        LL_I2C_HandleTransfer(I2Cx, addr_ << 1, LL_I2C_ADDRSLAVE_7BIT,
                              2, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_WRITE);

        // Wait for TXIS then send register address
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device);
            }
        }
        LL_I2C_TransmitData8(I2Cx, reg_);

        // Wait for TXIS then send data byte
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device);
            }
        }
        LL_I2C_TransmitData8(I2Cx, data);
    }

    // Wait for STOP flag (AUTOEND generates stop automatically)
    while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device);
        }
    }
    LL_I2C_ClearFlag_STOP(I2Cx);

    return true;
}

// Non-blocking write
bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;

    if (state->busy) {
        return false;
    }

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 1;
    state->reading = 0;
    state->write_p = data;
    state->read_p = data;
    state->bytes = len_;
    state->busy = true;
    state->error = false;

    if (reg_ == 0xFF) {
        // No register address, just send data bytes
        LL_I2C_HandleTransfer(I2Cx, state->addr, LL_I2C_ADDRSLAVE_7BIT,
                              len_, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_WRITE);
    } else {
        // Send register address + data bytes
        // First byte is register address, transmitted in ISR before data
        LL_I2C_HandleTransfer(I2Cx, state->addr, LL_I2C_ADDRSLAVE_7BIT,
                              1 + len_, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_WRITE);

        // Wait for TXIS to send the register address byte synchronously
        // so the ISR only deals with data bytes
        timeUs_t timeoutStartUs = microsISR();
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                state->error = true;
                state->busy = false;
                return i2cHandleHardwareFailure(device);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                state->busy = false;
                return i2cHandleHardwareFailure(device);
            }
        }
        LL_I2C_TransmitData8(I2Cx, reg_);
    }

    // Enable interrupts to handle remaining data bytes
    LL_I2C_EnableIT_TX(I2Cx);
    LL_I2C_EnableIT_NACK(I2Cx);
    LL_I2C_EnableIT_STOP(I2Cx);
    LL_I2C_EnableIT_ERR(I2Cx);

    return true;
}

// Blocking read
bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    if (reg_ == 0xFF) {
        // No register address, just receive data
        LL_I2C_HandleTransfer(I2Cx, addr_ << 1, LL_I2C_ADDRSLAVE_7BIT,
                              len, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_READ);
    } else {
        // First write the register address with SOFTEND (no stop, restart follows)
        LL_I2C_HandleTransfer(I2Cx, addr_ << 1, LL_I2C_ADDRSLAVE_7BIT,
                              1, LL_I2C_MODE_SOFTEND,
                              LL_I2C_GENERATE_START_WRITE);

        // Wait for TXIS then send register address
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                return i2cHandleHardwareFailure(device);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device);
            }
        }
        LL_I2C_TransmitData8(I2Cx, reg_);

        // Wait for Transfer Complete (TC) - SOFTEND means no auto-stop
        while (!LL_I2C_IsActiveFlag_TC(I2Cx)) {
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device);
            }
        }

        // Now send restart for the read phase
        LL_I2C_HandleTransfer(I2Cx, addr_ << 1, LL_I2C_ADDRSLAVE_7BIT,
                              len, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_READ);
    }

    // Read all bytes
    for (uint8_t i = 0; i < len; i++) {
        while (!LL_I2C_IsActiveFlag_RXNE(I2Cx)) {
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device);
            }
        }
        buf[i] = LL_I2C_ReceiveData8(I2Cx);
    }

    // Wait for STOP flag (AUTOEND generates stop automatically)
    while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device);
        }
    }
    LL_I2C_ClearFlag_STOP(I2Cx);

    return true;
}

// Non-blocking read
bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;

    if (state->busy) {
        return false;
    }

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 0;
    state->reading = 1;
    state->write_p = buf;
    state->read_p = buf;
    state->bytes = len;
    state->busy = true;
    state->error = false;

    if (reg_ == 0xFF) {
        // No register address, directly start read
        LL_I2C_HandleTransfer(I2Cx, state->addr, LL_I2C_ADDRSLAVE_7BIT,
                              len, LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_START_READ);

        // Enable RX interrupts
        LL_I2C_EnableIT_RX(I2Cx);
    } else {
        // First send register address with SOFTEND, TC interrupt will trigger restart
        LL_I2C_HandleTransfer(I2Cx, state->addr, LL_I2C_ADDRSLAVE_7BIT,
                              1, LL_I2C_MODE_SOFTEND,
                              LL_I2C_GENERATE_START_WRITE);

        // Wait for TXIS to send the register address byte synchronously
        timeUs_t timeoutStartUs = microsISR();
        while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
            if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
                LL_I2C_ClearFlag_NACK(I2Cx);
                state->error = true;
                state->busy = false;
                return i2cHandleHardwareFailure(device);
            }
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                state->busy = false;
                return i2cHandleHardwareFailure(device);
            }
        }
        LL_I2C_TransmitData8(I2Cx, reg_);

        // Enable TC interrupt - ISR will handle the restart and switch to RX
        LL_I2C_EnableIT_TC(I2Cx);
    }

    LL_I2C_EnableIT_NACK(I2Cx);
    LL_I2C_EnableIT_STOP(I2Cx);
    LL_I2C_EnableIT_ERR(I2Cx);

    return true;
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    i2cState_t *state = &i2cDevice[device].state;

    if (error) {
        *error = state->error;
    }

    return state->busy;
}

#endif
