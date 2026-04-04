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
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "pg/bus_i2c.h"

// The ESP-IDF SOC headers define I2C0/I2C1 as extern i2c_dev_t structs,
// but our platform.h redefines them as esp32_peripheral_t pointers.
// Save our platform definitions and undefine them so the LL header
// picks up the real hardware register structs.
#undef I2C0
#undef I2C1

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/i2c_ll.h"
#include "hal/gpio_ll.h"
#pragma GCC diagnostic pop

#include "soc/i2c_periph.h"
#include "soc/gpio_struct.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

// Cache the real i2c_dev_t hardware register pointers before
// restoring our platform macros (which would shadow them).
static i2c_dev_t *const i2cHwRegs[] = {
    I2C_LL_GET_HW(0),
    I2C_LL_GET_HW(1),
};

// Restore our platform I2C instance macros
#define I2C0 (&esp32I2cDev0)
#define I2C1 (&esp32I2cDev1)

// ESP32-S3 XTAL clock frequency used as I2C source clock (40 MHz)
#define ESP32_I2C_SOURCE_CLK_FREQ  40000000

// Polling timeout for I2C operations (iterations)
#define I2C_POLL_TIMEOUT  10000

// Get the i2c_dev_t hardware register pointer from a port number
static i2c_dev_t *i2cGetHw(int portNum)
{
    return i2cHwRegs[portNum];
}

// Get the I2C port number (0 or 1) from our platform peripheral pointer
static int i2cGetPortNum(i2cResource_t *reg)
{
    return I2C_INST((I2C_TypeDef *)reg);
}

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
    {
        .device = I2CDEV_0,
        .reg = (i2cResource_t *)I2C0,
        .sclPins = { I2CPINDEF(PA9), I2CPINDEF(PA2), },
        .sdaPins = { I2CPINDEF(PA8), I2CPINDEF(PA1), },
    },
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *)I2C1,
        .sclPins = { I2CPINDEF(PA7), I2CPINDEF(PA4), },
        .sdaPins = { I2CPINDEF(PA6), I2CPINDEF(PA3), },
    },
};

i2cDevice_t i2cDevice[I2CDEV_COUNT] = { 0 };

void i2cInit(i2cDevice_e device)
{
    if (device >= I2CDEV_COUNT) {
        return;
    }

    i2cDevice_t *dev = &i2cDevice[device];
    if (!dev->hardware) {
        return;
    }

    int portNum = i2cGetPortNum(dev->reg);
    i2c_dev_t *hw = i2cGetHw(portNum);

    // Enable peripheral clock and reset
    int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));
    i2c_ll_enable_bus_clock(portNum, true);
    i2c_ll_reset_register(portNum);

    // Configure SCL pin via GPIO matrix (open-drain with pull-up)
    IO_t sclIO = dev->scl;
    if (sclIO) {
        uint32_t pin = IO_Pin(sclIO);
        esp_rom_gpio_pad_select_gpio(pin);
        gpio_ll_output_enable(&GPIO, pin);
        gpio_ll_input_enable(&GPIO, pin);
        gpio_ll_od_enable(&GPIO, pin);
        if (dev->pullUp) {
            gpio_ll_pullup_en(&GPIO, pin);
        }
        esp_rom_gpio_connect_out_signal(pin, i2c_periph_signal[portNum].scl_out_sig, false, false);
        esp_rom_gpio_connect_in_signal(pin, i2c_periph_signal[portNum].scl_in_sig, false);
    }

    // Configure SDA pin via GPIO matrix (open-drain with pull-up)
    IO_t sdaIO = dev->sda;
    if (sdaIO) {
        uint32_t pin = IO_Pin(sdaIO);
        esp_rom_gpio_pad_select_gpio(pin);
        gpio_ll_output_enable(&GPIO, pin);
        gpio_ll_input_enable(&GPIO, pin);
        gpio_ll_od_enable(&GPIO, pin);
        if (dev->pullUp) {
            gpio_ll_pullup_en(&GPIO, pin);
        }
        esp_rom_gpio_connect_out_signal(pin, i2c_periph_signal[portNum].sda_out_sig, false, false);
        esp_rom_gpio_connect_in_signal(pin, i2c_periph_signal[portNum].sda_in_sig, false);
    }

    // Set master mode with open-drain outputs
    i2c_ll_set_mode(hw, I2C_BUS_MODE_MASTER);
    i2c_ll_enable_pins_open_drain(hw, true);
    i2c_ll_enable_arbitration(hw, true);

    // Enable controller clock and select XTAL as source
    i2c_ll_set_source_clk(hw, I2C_CLK_SRC_XTAL);
    i2c_ll_enable_controller_clock(hw, true);

    // Configure bus timing for desired speed
    uint32_t busFreq = dev->clockSpeed ? ((uint32_t)dev->clockSpeed * 1000) : 400000;
    i2c_hal_clk_config_t clkConfig;
    i2c_ll_master_cal_bus_clk(ESP32_I2C_SOURCE_CLK_FREQ, busFreq, &clkConfig);
    i2c_ll_master_set_bus_timing(hw, &clkConfig);
    i2c_ll_master_set_fractional_divider(hw, 0, 0);

    // Set glitch filter (default of 7 cycles)
    i2c_ll_master_set_filter(hw, 7);

    // Disable all interrupts (we use polling)
    i2c_ll_disable_intr_mask(hw, 0xFFFFFFFF);
    i2c_ll_clear_intr_mask(hw, 0xFFFFFFFF);

    // Reset FIFOs
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);

    i2c_ll_update(hw);
}

// Wait for the bus to become free, returns true if bus is free
static bool i2cWaitBusFree(i2c_dev_t *hw)
{
    int timeout = I2C_POLL_TIMEOUT;
    while (i2c_ll_is_bus_busy(hw) && --timeout > 0) {
    }
    return timeout > 0;
}

// Wait for a specific command to complete, returns true on success
static bool i2cWaitCmdDone(i2c_dev_t *hw, int cmdIdx)
{
    int timeout = I2C_POLL_TIMEOUT;
    while (!i2c_ll_master_is_cmd_done(hw, cmdIdx) && --timeout > 0) {
        // Check for NACK or timeout errors
        uint32_t status;
        i2c_ll_get_intr_mask(hw, &status);
        if (status & (I2C_LL_INTR_NACK | I2C_LL_INTR_TIMEOUT | I2C_LL_INTR_ARBITRATION)) {
            i2c_ll_clear_intr_mask(hw, status);
            return false;
        }
    }
    return timeout > 0;
}

bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device >= I2CDEV_COUNT) {
        return false;
    }

    i2cDevice_t *dev = &i2cDevice[device];
    if (!dev->reg) {
        return false;
    }

    int portNum = i2cGetPortNum(dev->reg);
    i2c_dev_t *hw = i2cGetHw(portNum);

    if (!i2cWaitBusFree(hw)) {
        return false;
    }

    // Reset FIFOs
    i2c_ll_txfifo_rst(hw);

    // Clear any pending interrupts
    i2c_ll_clear_intr_mask(hw, 0xFFFFFFFF);

    // Load TX FIFO: address byte (write), register, data
    uint8_t addrByte = (addr_ << 1) | 0;  // Write bit = 0
    i2c_ll_write_txfifo(hw, &addrByte, 1);
    i2c_ll_write_txfifo(hw, &reg_, 1);
    if (len_ > 0 && data) {
        i2c_ll_write_txfifo(hw, data, len_);
    }

    int cmdIdx = 0;
    i2c_ll_hw_cmd_t cmd;

    // Command 0: START
    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmdIdx++);

    // Command 1: WRITE (addr + reg + data bytes)
    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 2 + len_;
    cmd.ack_en = 1;
    cmd.ack_exp = 0;  // Expect ACK
    i2c_ll_master_write_cmd_reg(hw, cmd, cmdIdx++);

    // Command 2: STOP
    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_STOP;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmdIdx++);

    // Start transaction
    i2c_ll_update(hw);
    i2c_ll_start_trans(hw);

    // Wait for STOP command to complete
    return i2cWaitCmdDone(hw, cmdIdx - 1);
}

bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    if (device >= I2CDEV_COUNT || len == 0 || !buf) {
        return false;
    }

    i2cDevice_t *dev = &i2cDevice[device];
    if (!dev->reg) {
        return false;
    }

    int portNum = i2cGetPortNum(dev->reg);
    i2c_dev_t *hw = i2cGetHw(portNum);

    if (!i2cWaitBusFree(hw)) {
        return false;
    }

    // Reset FIFOs
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);

    // Clear any pending interrupts
    i2c_ll_clear_intr_mask(hw, 0xFFFFFFFF);

    // Load TX FIFO: address (write), register, address (read)
    uint8_t addrW = (addr_ << 1) | 0;  // Write bit = 0
    i2c_ll_write_txfifo(hw, &addrW, 1);
    i2c_ll_write_txfifo(hw, &reg_, 1);
    uint8_t addrR = (addr_ << 1) | 1;  // Read bit = 1
    i2c_ll_write_txfifo(hw, &addrR, 1);

    int cmdIdx = 0;
    i2c_ll_hw_cmd_t cmd;

    // Command 0: START
    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmdIdx++);

    // Command 1: WRITE 2 bytes (addr+W, register)
    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 2;
    cmd.ack_en = 1;
    cmd.ack_exp = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmdIdx++);

    // Command 2: REPEATED START
    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmdIdx++);

    // Command 3: WRITE 1 byte (addr+R)
    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;
    cmd.ack_exp = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmdIdx++);

    // For multi-byte reads, read all but last byte with ACK
    if (len > 1) {
        // Command 4: READ (len-1) bytes, sending ACK
        cmd.val = 0;
        cmd.op_code = I2C_LL_CMD_READ;
        cmd.byte_num = len - 1;
        cmd.ack_val = 0;  // Send ACK
        i2c_ll_master_write_cmd_reg(hw, cmd, cmdIdx++);
    }

    // Read last byte with NACK
    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_READ;
    cmd.byte_num = 1;
    cmd.ack_val = 1;  // Send NACK for last byte
    i2c_ll_master_write_cmd_reg(hw, cmd, cmdIdx++);

    // Command: STOP
    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_STOP;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmdIdx++);

    // Start transaction
    i2c_ll_update(hw);
    i2c_ll_start_trans(hw);

    // Wait for STOP command to complete
    if (!i2cWaitCmdDone(hw, cmdIdx - 1)) {
        return false;
    }

    // Read received data from RX FIFO
    i2c_ll_read_rxfifo(hw, buf, len);

    return true;
}

bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf);
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    if (error) {
        *error = false;
    }

    if (device >= I2CDEV_COUNT) {
        return false;
    }

    i2cDevice_t *dev = &i2cDevice[device];
    if (!dev->reg) {
        return false;
    }

    int portNum = i2cGetPortNum(dev->reg);
    i2c_dev_t *hw = i2cGetHw(portNum);

    return i2c_ll_is_bus_busy(hw);
}

void i2cPinConfigure(const struct i2cConfig_s *i2cConfig)
{
    for (int devIdx = 0; devIdx < I2CDEV_COUNT; devIdx++) {
        const i2cHardware_t *hw = &i2cHardware[devIdx];
        i2cDevice_t *dev = &i2cDevice[devIdx];

        ioTag_t cfgScl = i2cConfig[devIdx].ioTagScl;
        ioTag_t cfgSda = i2cConfig[devIdx].ioTagSda;

        if (!cfgScl && !cfgSda) {
            continue;
        }

        for (int pIdx = 0; pIdx < I2C_PIN_SEL_MAX; pIdx++) {
            if (cfgScl && cfgScl == hw->sclPins[pIdx].ioTag) {
                dev->scl = IOGetByTag(cfgScl);
            }
            if (cfgSda && cfgSda == hw->sdaPins[pIdx].ioTag) {
                dev->sda = IOGetByTag(cfgSda);
            }
        }

        if (dev->scl && dev->sda) {
            dev->hardware = hw;
            dev->reg = hw->reg;
            dev->pullUp = i2cConfig[devIdx].pullUp;
            dev->clockSpeed = i2cConfig[devIdx].clockSpeed;
        }
    }
}

uint16_t i2cGetErrorCounter(void)
{
    return 0;
}

#endif // USE_I2C
