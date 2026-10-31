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

/*
 * i.MX 95 LPI2C master back-end for the common I2C driver: polled, no DMA.
 * I2CDEV_1 is LPI2C1 and I2CDEV_6 is LPI2C6.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

#include "pg/bus_i2c.h"

#include "navq95_scmi.h"

#define LPI2C_VERID_OFFSET    0x00u
#define LPI2C_PARAM_OFFSET    0x04u
#define LPI2C_MCR_OFFSET      0x10u
#define LPI2C_MSR_OFFSET      0x14u
#define LPI2C_MIER_OFFSET     0x18u
#define LPI2C_MCFGR1_OFFSET   0x24u
#define LPI2C_MCFGR2_OFFSET   0x28u
#define LPI2C_MCFGR3_OFFSET   0x2Cu
#define LPI2C_MCCR0_OFFSET    0x48u
#define LPI2C_MFCR_OFFSET     0x58u
#define LPI2C_MFSR_OFFSET     0x5Cu
#define LPI2C_MTDR_OFFSET     0x60u
#define LPI2C_MRDR_OFFSET     0x70u

#define LPI2C_MCR_MEN         (1u << 0)
#define LPI2C_MCR_RST         (1u << 1)
#define LPI2C_MCR_RTF         (1u << 8)
#define LPI2C_MCR_RRF         (1u << 9)

#define LPI2C_MSR_TDF         (1u << 0)
#define LPI2C_MSR_RDF         (1u << 1)
#define LPI2C_MSR_EPF         (1u << 8)
#define LPI2C_MSR_SDF         (1u << 9)
#define LPI2C_MSR_NDF         (1u << 10)
#define LPI2C_MSR_ALF         (1u << 11)
#define LPI2C_MSR_FEF         (1u << 12)
#define LPI2C_MSR_MBF         (1u << 24)
#define LPI2C_MSR_BBF         (1u << 25)
// NACK, arbitration lost and FIFO error abort a transfer.
#define LPI2C_MSR_ERROR_MASK  (LPI2C_MSR_NDF | LPI2C_MSR_ALF | LPI2C_MSR_FEF)
#define LPI2C_MSR_W1C         (LPI2C_MSR_EPF | LPI2C_MSR_SDF | LPI2C_MSR_NDF | \
                               LPI2C_MSR_ALF | LPI2C_MSR_FEF)

#define LPI2C_MCFGR1_PRESCALE(n)  ((uint32_t)(n) & 0x7u)

#define LPI2C_MCCR0_CLKLO(n)    ((uint32_t)(n) & 0x3Fu)
#define LPI2C_MCCR0_CLKHI(n)    (((uint32_t)(n) & 0x3Fu) << 8)
#define LPI2C_MCCR0_SETHOLD(n)  (((uint32_t)(n) & 0x3Fu) << 16)
#define LPI2C_MCCR0_DATAVD(n)   (((uint32_t)(n) & 0x3Fu) << 24)

// MTDR command field.
#define LPI2C_MTDR_CMD_TXD      (0u << 8)
#define LPI2C_MTDR_CMD_RXD      (1u << 8)
#define LPI2C_MTDR_CMD_STOP     (2u << 8)
#define LPI2C_MTDR_CMD_START    (4u << 8)

#define LPI2C_MRDR_RXEMPTY      (1u << 14)

#define NAVQ95_LPI2C1_CLK_ROOT  6u
#define NAVQ95_LPI2C6_CLK_ROOT  91u
#define NAVQ95_LPI2C_SRC_HZ     24000000u

// Bounds every wait on a controller status bit; stops use the shorter bound.
#define NAVQ95_LPI2C_SPIN       2000000u
#define NAVQ95_LPI2C_PROBE_SPIN 50000u

// LPI2C6 is on the shared GPIO_IO02 (SDA) and GPIO_IO03 (SCL) pads, with input select.
#define NAVQ95_I2C6_SDA_MUX     0x443C0018u
#define NAVQ95_I2C6_SDA_DSY     0x443C051Cu
#define NAVQ95_I2C6_SDA_CFG     0x443C021Cu
#define NAVQ95_I2C6_SCL_MUX     0x443C001Cu
#define NAVQ95_I2C6_SCL_DSY     0x443C0518u
#define NAVQ95_I2C6_SCL_CFG     0x443C0220u
#define NAVQ95_I2C6_ALT         6u
#define NAVQ95_I2C6_DSY_VAL     0u

// LPI2C1 has dedicated pads.
#define NAVQ95_I2C1_SCL_MUX     0x443C01C0u
#define NAVQ95_I2C1_SCL_CFG     0x443C03C4u
#define NAVQ95_I2C1_SDA_MUX     0x443C01C4u
#define NAVQ95_I2C1_SDA_CFG     0x443C03C8u
#define NAVQ95_I2C1_ALT         0u

/*
 * Pad control values, passed to the System Manager unshifted. Both are open
 * drain. LPI2C6 relies on external pull-ups; LPI2C1 needs the internal pull-up.
 */
#define NAVQ95_I2C6_PADVAL      0x097Eu
#define NAVQ95_I2C1_PADVAL      0x0B06u

/*
 * SCL timing with a 6 MHz tick (24 MHz, prescale 4). LPI2C6 is exactly
 * 100 kHz. LPI2C1 is set to 111 kHz because the slow rise through the internal
 * pull-up stretches each period, giving about 100 kHz on the bus.
 */
#define NAVQ95_I2C6_MCCR0  (LPI2C_MCCR0_CLKLO(29u)   | LPI2C_MCCR0_CLKHI(29u) \
                          | LPI2C_MCCR0_SETHOLD(29u) | LPI2C_MCCR0_DATAVD(14u))

#define NAVQ95_I2C1_MCCR0  (LPI2C_MCCR0_CLKLO(26u)   | LPI2C_MCCR0_CLKHI(26u) \
                          | LPI2C_MCCR0_SETHOLD(26u) | LPI2C_MCCR0_DATAVD(13u))

static volatile uint16_t i2cErrorCount = 0;

i2cDevice_t i2cDevice[I2CDEV_COUNT];

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    [I2CDEV_1] = {
        .device = I2CDEV_1,
        .reg    = (i2cResource_t *)I2C1,
    },
#endif
#ifdef USE_I2C_DEVICE_6
    [I2CDEV_6] = {
        .device = I2CDEV_6,
        .reg    = (i2cResource_t *)I2C6,
    },
#endif
};

static inline volatile uint32_t *i2c_reg(uint32_t base, uint32_t off)
{
    return (volatile uint32_t *)(base + off);
}

static int navq95_lpi2c_padmux(uint32_t base)
{
    int sclS, sdaS, rateS, clkS;

    // The clock root rate must be set before enabling it; the boot firmware leaves it unset.
    if (base == (uint32_t)I2C6) {
        sclS = navq95_scmi_pinmux_daisy(NAVQ95_I2C6_SCL_MUX, NAVQ95_I2C6_ALT, 1u,
                                        NAVQ95_I2C6_SCL_DSY, NAVQ95_I2C6_DSY_VAL,
                                        NAVQ95_I2C6_PADVAL);
        sdaS = navq95_scmi_pinmux_daisy(NAVQ95_I2C6_SDA_MUX, NAVQ95_I2C6_ALT, 1u,
                                        NAVQ95_I2C6_SDA_DSY, NAVQ95_I2C6_DSY_VAL,
                                        NAVQ95_I2C6_PADVAL);

        rateS = navq95_scmi_clock_rate_set(NAVQ95_LPI2C6_CLK_ROOT, NAVQ95_LPI2C_SRC_HZ);
        clkS  = navq95_scmi_clock_enable(NAVQ95_LPI2C6_CLK_ROOT);
    } else {
        sclS = navq95_scmi_pinmux(NAVQ95_I2C1_SCL_MUX, NAVQ95_I2C1_ALT, 1u,
                                  NAVQ95_I2C1_SCL_CFG, NAVQ95_I2C1_PADVAL);
        sdaS = navq95_scmi_pinmux(NAVQ95_I2C1_SDA_MUX, NAVQ95_I2C1_ALT, 1u,
                                  NAVQ95_I2C1_SDA_CFG, NAVQ95_I2C1_PADVAL);
        rateS = navq95_scmi_clock_rate_set(NAVQ95_LPI2C1_CLK_ROOT, NAVQ95_LPI2C_SRC_HZ);
        clkS  = navq95_scmi_clock_enable(NAVQ95_LPI2C1_CLK_ROOT);
    }

    return sclS | sdaS | rateS | clkS;
}

static void navq95_lpi2c_configure(uint32_t base)
{
    volatile uint32_t *mcr    = i2c_reg(base, LPI2C_MCR_OFFSET);
    volatile uint32_t *mcfgr1 = i2c_reg(base, LPI2C_MCFGR1_OFFSET);
    volatile uint32_t *mcfgr2 = i2c_reg(base, LPI2C_MCFGR2_OFFSET);
    volatile uint32_t *mcfgr3 = i2c_reg(base, LPI2C_MCFGR3_OFFSET);
    volatile uint32_t *mccr0  = i2c_reg(base, LPI2C_MCCR0_OFFSET);
    volatile uint32_t *mfcr   = i2c_reg(base, LPI2C_MFCR_OFFSET);

    *mcr = 0u;
    *mcr = LPI2C_MCR_RST | LPI2C_MCR_RTF | LPI2C_MCR_RRF;
    *mcr = 0u;

    *mcfgr1 = LPI2C_MCFGR1_PRESCALE(2u);

    *mcfgr2 = 0u;
    *mcfgr3 = 0u;

    *mccr0 = (base == (uint32_t)I2C1) ? NAVQ95_I2C1_MCCR0 : NAVQ95_I2C6_MCCR0;

    *mfcr = 0u;

    *mcr = LPI2C_MCR_MEN;
    (void)*mcr;
}

static inline void navq95_lpi2c_clear_status(uint32_t base)
{
    volatile uint32_t *msr = i2c_reg(base, LPI2C_MSR_OFFSET);
    *msr = LPI2C_MSR_W1C;
}

static bool navq95_lpi2c_wait_txready(uint32_t base)
{
    volatile uint32_t *msr = i2c_reg(base, LPI2C_MSR_OFFSET);
    uint32_t spin = NAVQ95_LPI2C_SPIN;

    while (spin--) {
        uint32_t s = *msr;
        if (s & LPI2C_MSR_ERROR_MASK) {
            return false;
        }
        if (s & LPI2C_MSR_TDF) {
            return true;
        }
    }
    return false;
}

static bool navq95_lpi2c_cmd(uint32_t base, uint32_t cmdWord)
{
    if (!navq95_lpi2c_wait_txready(base)) {
        return false;
    }
    *i2c_reg(base, LPI2C_MTDR_OFFSET) = cmdWord;
    return true;
}

static bool navq95_lpi2c_start(uint32_t base, uint8_t addr7, uint8_t rw)
{
    navq95_lpi2c_clear_status(base);
    uint32_t data = (uint32_t)((addr7 << 1) | (rw & 1u)) & 0xFFu;
    return navq95_lpi2c_cmd(base, LPI2C_MTDR_CMD_START | data);
}

// Issue STOP and flush the FIFOs so the next transfer starts clean.
static bool navq95_lpi2c_stop(uint32_t base)
{
    volatile uint32_t *msr = i2c_reg(base, LPI2C_MSR_OFFSET);
    volatile uint32_t *mcr = i2c_reg(base, LPI2C_MCR_OFFSET);

    navq95_lpi2c_clear_status(base);

    *i2c_reg(base, LPI2C_MTDR_OFFSET) = LPI2C_MTDR_CMD_STOP;

    uint32_t spin = NAVQ95_LPI2C_PROBE_SPIN;
    bool done = false;
    while (spin--) {
        uint32_t s = *msr;
        if (s & LPI2C_MSR_SDF) {
            done = true;
            break;
        }
        if (!(s & LPI2C_MSR_MBF)) {
            done = true;
            break;
        }
    }

    navq95_lpi2c_clear_status(base);
    *mcr = LPI2C_MCR_MEN | LPI2C_MCR_RTF | LPI2C_MCR_RRF;
    *mcr = LPI2C_MCR_MEN;

    return done;
}

static uint32_t i2cBase(i2cDevice_e device)
{
    if ((unsigned)device >= I2CDEV_COUNT) {
        return 0;
    }
    return (uint32_t)i2cDevice[device].reg;
}

void i2cPinConfigure(const i2cConfig_t *i2cConfig)
{
    for (int index = 0; index < I2CDEV_COUNT; index++) {
        const i2cHardware_t *hw = &i2cHardware[index];
        if (!hw->reg) {
            continue;
        }
        i2cDevice_t *pDev = &i2cDevice[hw->device];
        memset(pDev, 0, sizeof(*pDev));
        pDev->hardware   = hw;
        pDev->reg        = hw->reg;
        pDev->pullUp     = i2cConfig[hw->device].pullUp;
        pDev->clockSpeed = i2cConfig[hw->device].clockSpeed;
    }
}

void i2cInit(i2cDevice_e device)
{
    if ((unsigned)device >= I2CDEV_COUNT) {
        return;
    }

    uint32_t base = (uint32_t)i2cDevice[device].reg;
    if (base) {
        (void)navq95_lpi2c_padmux(base);
        navq95_lpi2c_configure(base);
    }

// fc/init.c only initialises I2CDEV_0..I2CDEV_4, so bring LPI2C6 up with I2CDEV_1.
#if defined(USE_I2C_DEVICE_1) && defined(USE_I2C_DEVICE_6)
    if (device == I2CDEV_1) {
        uint32_t base6 = (uint32_t)i2cDevice[I2CDEV_6].reg;
        if (base6) {
            (void)navq95_lpi2c_padmux(base6);
            navq95_lpi2c_configure(base6);
        }
    }
#endif
}

bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    uint32_t base = i2cBase(device);
    if (!base) {
        return false;
    }

    bool ok = navq95_lpi2c_start(base, addr_, 0u);

    // reg 0xFF with no data is an address-only probe.
    if (ok && !(reg_ == 0xFFu && len_ == 0u && data == NULL)) {
        ok = navq95_lpi2c_cmd(base, LPI2C_MTDR_CMD_TXD | reg_);
        for (uint8_t i = 0; ok && i < len_; i++) {
            ok = navq95_lpi2c_cmd(base, LPI2C_MTDR_CMD_TXD | data[i]);
        }
    }

    bool stopped = navq95_lpi2c_stop(base);

    if (!ok || !stopped) {
        i2cErrorCount++;
        return false;
    }
    return true;
}

bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint32_t base = i2cBase(device);
    if (!base || len == 0) {
        return false;
    }

    volatile uint32_t *msr = i2c_reg(base, LPI2C_MSR_OFFSET);
    volatile uint32_t *mrdr = i2c_reg(base, LPI2C_MRDR_OFFSET);

    bool ok = navq95_lpi2c_start(base, addr_, 0u);
    if (ok) {
        ok = navq95_lpi2c_cmd(base, LPI2C_MTDR_CMD_TXD | reg);
    }
    if (ok) {
        ok = navq95_lpi2c_start(base, addr_, 1u);
    }
    if (ok) {
        // The RXD command takes the byte count minus one.
        ok = navq95_lpi2c_cmd(base, LPI2C_MTDR_CMD_RXD | (uint32_t)(len - 1));
    }

    for (uint8_t i = 0; ok && i < len; i++) {
        uint32_t spin = NAVQ95_LPI2C_SPIN;
        bool got = false;
        while (spin--) {
            uint32_t s = *msr;
            if (s & LPI2C_MSR_ERROR_MASK) {
                ok = false;
                break;
            }
            uint32_t rd = *mrdr;
            if (!(rd & LPI2C_MRDR_RXEMPTY)) {
                buf[i] = (uint8_t)(rd & 0xFFu);
                got = true;
                break;
            }
        }
        if (!got) {
            ok = false;
        }
    }

    bool stopped = navq95_lpi2c_stop(base);

    if (!ok || !stopped) {
        i2cErrorCount++;
        return false;
    }
    return true;
}

bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return i2cReadBuffer(device, addr_, reg, len, buf);
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    uint32_t base = i2cBase(device);
    if (error) {
        *error = false;
    }
    if (!base) {
        return false;
    }
    return (*i2c_reg(base, LPI2C_MSR_OFFSET) & LPI2C_MSR_MBF) != 0u;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

#endif // USE_I2C && !USE_SOFT_I2C
