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
#include "drivers/nvic.h"

// Ensure Pico-specific types are defined
#include "platform/platform.h"

#define DEF_I2C_INTR (I2C_IC_INTR_STAT_R_STOP_DET_BITS | \
                      I2C_IC_INTR_STAT_R_TX_ABRT_BITS | \
                      I2C_IC_INTR_STAT_R_TX_OVER_BITS | \
                      I2C_IC_INTR_STAT_R_RX_OVER_BITS)

#define I2C_FIFO_BUFFER_DEPTH 16

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

typedef enum {
    I2C_STATE_IDLE,         // Idle
    I2C_STATE_ACTIVE,       // Transfer in progress
    I2C_STATE_READ_DATA,    // Multi-batch read in progress
} i2c_state_t;

typedef volatile struct {
    i2c_inst_t *i2c;
    i2c_state_t state;
    uint32_t intr_error_stat;
    bool read;
    uint8_t *data;
    uint8_t len;
    uint8_t bytes_transfered;
    uint8_t remaining_batches;
} i2c_context_t;

i2c_context_t i2c_contexts[I2CDEV_COUNT];

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

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

bool i2cWrite(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t data)
{
    // Start non-blocking write
    if (!i2cWriteBuffer(device, addr, reg, 1, &data)) {
        return false;
    }

    // Wait for completion
    while (i2cBusy(device, NULL)) {
        // Wait until transfer is complete
    }

    return true;
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    // We don't ever write more bytes than will fit in the FIFO, but, just in case, validate args
    if (device == I2CINVALID || device >= I2CDEV_COUNT || len == 0 || len >= I2C_FIFO_BUFFER_DEPTH) {
        return false;
    }

    i2c_inst_t *port = I2C_INST(i2cHardware[device].reg);

    if (!port) {
        return false;
    }

    // Check if I2C is busy
    if (i2cBusy(device, NULL)) {
        return false;
    }

    i2c_context_t *context = &i2c_contexts[device];
    i2c_hw_t *hw = port->hw;

    // Set up transfer parameters
    context->read = false;
    context->data = data;
    context->len = len;
    context->bytes_transfered = 0;
    context->state = I2C_STATE_ACTIVE;

    // Set up I2C for transfer
    hw->enable = 0;
    hw->tar = addr;
    hw->enable = 1;

    // Preload TX FIFO with all commands for write sequence
    // 1. Register address (normal write, no restart needed)
    hw->data_cmd = reg;

    // 2. Data bytes
    for (uint8_t i = 0; i < len; i++) {
        uint32_t cmd = data[i];
        if (i == len - 1) {
            // Last byte - add stop
            cmd |= I2C_IC_DATA_CMD_STOP_BITS;
        }
        hw->data_cmd = cmd;
    }

    // Enable only STOP and error interrupts
    hw->intr_mask = DEF_I2C_INTR;

    return true;
}

bool i2cRead(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    // Start non-blocking read
    if (!i2cReadBuffer(device, addr, reg, len, buf)) {
        return false;
    }

    // Wait for completion
    while (i2cBusy(device, NULL)) {
        // Wait until transfer is complete
    }

    return true;
}

static void i2c_load_read_commands(i2c_hw_t *hw, uint8_t len, bool final_batch)
{
    // Load read commands into TX FIFO
    for (uint8_t i = 0; i < len; i++) {
        uint32_t cmd = I2C_IC_DATA_CMD_CMD_BITS;
        if (i == len - 1 && final_batch) {
            // Last command of final batch - add stop
            cmd |= I2C_IC_DATA_CMD_STOP_BITS;
        }
        hw->data_cmd = cmd;
    }
}

bool i2cReadBuffer(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT || len == 0) {
        return false;
    }

    i2c_inst_t *port = I2C_INST(i2cHardware[device].reg);

    if (!port) {
        return false;
    }

    // Check if I2C is busy
    if (i2cBusy(device, NULL)) {
        return false;
    }

    i2c_context_t *context = &i2c_contexts[device];
    i2c_hw_t *hw = port->hw;

    // Set up transfer parameters
    context->read = true;
    context->data = buf;
    context->len = len;
    context->bytes_transfered = 0;

    // Set up I2C for transfer
    hw->enable = 0;
    hw->tar = addr;
    hw->enable = 1;

    // Preload TX FIFO with commands for read sequence
    // 1. Register address (normal write)
    hw->data_cmd = reg;

    if (len <= I2C_FIFO_BUFFER_DEPTH - 1) {
        // Single batch - load all read commands
        context->state = I2C_STATE_ACTIVE;

        // 2. Restart + read commands (all in one batch)
        for (uint8_t i = 0; i < len; i++) {
            uint32_t cmd = I2C_IC_DATA_CMD_CMD_BITS;
            if (i == 0) {
                // First command needs restart
                cmd |= I2C_IC_DATA_CMD_RESTART_BITS;
            }
            if (i == len - 1) {
                // Last byte - add stop
                cmd |= I2C_IC_DATA_CMD_STOP_BITS;
            }
            hw->data_cmd = cmd;
        }

        hw->intr_mask = DEF_I2C_INTR;
    } else {
        // Multi-batch read
        context->state = I2C_STATE_READ_DATA;

        // Load first batch of read commands with restart
        for (uint8_t i = 0; i < I2C_FIFO_BUFFER_DEPTH - 1; i++) {
            uint32_t cmd = I2C_IC_DATA_CMD_CMD_BITS;
            if (i == 0) {
                // First command needs restart
                cmd |= I2C_IC_DATA_CMD_RESTART_BITS;
            }
            hw->data_cmd = cmd;
        }

        // Enable RX_FULL interrupt
        hw->intr_mask = DEF_I2C_INTR | I2C_IC_INTR_STAT_R_RX_FULL_BITS;
    }

    return true;
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

    // Check if we have a transfer in progress via our state machine
    i2c_context_t *context = &i2c_contexts[device];
    if (context->state != I2C_STATE_IDLE) {
        return true;
    }

    // Also check hardware status
    uint32_t status_reg = port->hw->status;
    return (status_reg & I2C_IC_STATUS_ACTIVITY_VALUE_ACTIVE) != 0;
}

static void i2c_handle_error(i2c_context_t *i2c_context, uint32_t intr_stat)
{
    i2c_hw_t *hw = i2c_context->i2c->hw;

    // Enable only default interrupts
    hw->intr_mask = DEF_I2C_INTR;
    i2c_context->state = I2C_STATE_IDLE;
    i2c_context->intr_error_stat = intr_stat & DEF_I2C_INTR;
    i2cErrorCount++;
}

// Common I2C IRQ handler
static void i2c_irq_handler(i2c_context_t *i2c_context)
{
    i2c_hw_t *hw = i2c_context->i2c->hw;
    uint32_t intr_stat = hw->intr_stat;

    // Handle error conditions first
    if (intr_stat & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        (void)hw->clr_tx_abrt;
        i2c_handle_error(i2c_context, intr_stat);
        return;
    }

    if (intr_stat & I2C_IC_INTR_STAT_R_TX_OVER_BITS) {
        (void)hw->clr_tx_over;
        i2c_handle_error(i2c_context, intr_stat);
        return;
    }

    if (intr_stat & I2C_IC_INTR_STAT_R_RX_OVER_BITS) {
        (void)hw->clr_rx_over;
        i2c_handle_error(i2c_context, intr_stat);
        return;
    }

    // Handle stop detection - transfer complete
    if (intr_stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        (void)hw->clr_stop_det;

        // Read any remaining data from RX FIFO
        if (i2c_context->read) {
            while (hw->rxflr > 0) {
                i2c_context->data[i2c_context->bytes_transfered++] = (uint8_t)(hw->data_cmd & I2C_IC_DATA_CMD_DAT_BITS);
            }
        }

        // Transfer complete
        hw->intr_mask = DEF_I2C_INTR;
        i2c_context->state = I2C_STATE_IDLE;
        return;
    }

    // Handle RX FIFO full for multi-batch reads
    if ((intr_stat & I2C_IC_INTR_STAT_R_RX_FULL_BITS) && i2c_context->state == I2C_STATE_READ_DATA) {
        // Read current batch from RX FIFO (should be <= I2C_FIFO_BUFFER_DEPTH bytes)
        while (hw->rxflr > 0) {
            i2c_context->data[i2c_context->bytes_transfered++] = (uint8_t)(hw->data_cmd & I2C_IC_DATA_CMD_DAT_BITS);
        }

        // Check if more batches are needed
        if (i2c_context->bytes_transfered < i2c_context->len) {
            // Calculate how many bytes are left to read
            uint8_t remaining_bytes = i2c_context->len - i2c_context->bytes_transfered;
            uint8_t batch_size = MIN(remaining_bytes, I2C_FIFO_BUFFER_DEPTH);
            bool final_batch = (remaining_bytes <= I2C_FIFO_BUFFER_DEPTH);

            // Load next batch of read commands
            i2c_load_read_commands(hw, batch_size, final_batch);

            if (final_batch) {
                // Last batch - disable RX_FULL interrupt, only wait for STOP
                hw->intr_mask = DEF_I2C_INTR;
            }
        }
   }
}

static void i2c_irq0_handler(void)
{
    i2c_irq_handler(&i2c_contexts[I2CDEV_0]);
}

static void i2c_irq1_handler(void)
{
    i2c_irq_handler(&i2c_contexts[I2CDEV_1]);
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

    if (!hardware || !scl || !sda) {
        return;
    }

    const uint8_t sclPin = IO_Pin(scl);
    const uint8_t sdaPin = IO_Pin(sda);

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

    i2c_contexts[device].i2c = I2C_INST(i2cHardware[device].reg);
    i2c_contexts[device].state = I2C_STATE_IDLE;

    // Enable the default interrupts
    hardware->reg->hw->intr_mask = DEF_I2C_INTR;

    // Set FIFO thresholds
    // The reg takes up one position in the output FIFO and if
    // I2C_FIFO_BUFFER_DEPTH bytes are written following this
    // then a race condition occurs where if that byte isn't
    // consumed quickly enough by the hardware a TX_ABORT occurs.
    // Thus the rx_tl must be set to interrupt with one byte less.
    hardware->reg->hw->rx_tl = I2C_FIFO_BUFFER_DEPTH - 2;

    // Register the I2C interrupt handler
    switch (device) {
    case I2CDEV_0:
        irq_set_exclusive_handler(I2C0_IRQ, i2c_irq0_handler);
        irq_set_priority(I2C0_IRQ, NVIC_PRIO_I2C_EV);
        irq_set_enabled(I2C0_IRQ, true);
        break;

    case I2CDEV_1:
        irq_set_exclusive_handler(I2C1_IRQ, i2c_irq1_handler);
        irq_set_priority(I2C1_IRQ, NVIC_PRIO_I2C_EV);
        irq_set_enabled(I2C1_IRQ, true);
        break;

    default:
        break;
    }
}

#endif // #if defined(USE_I2C) && !defined(SOFT_I2C)
