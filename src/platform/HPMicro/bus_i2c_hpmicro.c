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

// I2C bus implementation on HPM SoCs: pin/hardware mapping, blocking
// master read/write transfers, and bus recovery on failure.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "pg/bus_i2c.h"

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_utils.h"
#include "hpm_i2c_drv.h"
#include "hpm_soc.h"
#include "hpm_clock_drv.h"
#include "hpm_batt_iomux.h"
#include "hpm_pmic_iomux.h"
#include "hpm_iomux.h"
#include "io_hpmicro.h"

#define IOCFG_I2C   IOCFG_AF_OD

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_0
#ifdef HPM6360
    {
        .device = I2CDEV_0,
        .reg = (i2cResource_t *) HPM_I2C0,
        .sclPins = {
            I2CPINDEF(PA6, IOC_PA06_FUNC_CTL_I2C0_SCL),
            I2CPINDEF(PA23, IOC_PA23_FUNC_CTL_I2C0_SCL),
            I2CPINDEF(PB22, IOC_PB22_FUNC_CTL_I2C0_SCL),
            I2CPINDEF(PC13, IOC_PC13_FUNC_CTL_I2C0_SCL),
            I2CPINDEF(PY4, IOC_PY04_FUNC_CTL_I2C0_SCL),
        },
        .sdaPins = {
            I2CPINDEF(PA7, IOC_PA07_FUNC_CTL_I2C0_SDA),
            I2CPINDEF(PA24, IOC_PA24_FUNC_CTL_I2C0_SDA),
            I2CPINDEF(PB23, IOC_PB23_FUNC_CTL_I2C0_SDA),
            I2CPINDEF(PC14, IOC_PC14_FUNC_CTL_I2C0_SDA),
            I2CPINDEF(PY5, IOC_PY05_FUNC_CTL_I2C0_SDA),
        },
        .rcc = clock_i2c0,
    },
#endif
#ifdef HPM6750
    {
        .device = I2CDEV_0,
        .reg = (i2cResource_t *) HPM_I2C0,
        .sclPins = {
            I2CPINDEF(PA6, IOC_PA06_FUNC_CTL_I2C0_SCL),
            I2CPINDEF(PB11, IOC_PB11_FUNC_CTL_I2C0_SCL),
            I2CPINDEF(PD1, IOC_PD01_FUNC_CTL_I2C0_SCL),
            I2CPINDEF(PE15, IOC_PE15_FUNC_CTL_I2C0_SCL),
            I2CPINDEF(PF5, IOC_PF05_FUNC_CTL_I2C0_SCL),
        },
        .sdaPins = {
            I2CPINDEF(PA5, IOC_PA05_FUNC_CTL_I2C0_SDA),
            I2CPINDEF(PB10, IOC_PB10_FUNC_CTL_I2C0_SDA),
            I2CPINDEF(PD0, IOC_PD00_FUNC_CTL_I2C0_SDA),
            I2CPINDEF(PE14, IOC_PE14_FUNC_CTL_I2C0_SDA),
            I2CPINDEF(PF8, IOC_PF08_FUNC_CTL_I2C0_SDA),
        },
        .rcc = clock_i2c0,
    },
#endif
#endif
#ifdef USE_I2C_DEVICE_1
#ifdef HPM6360
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *) HPM_I2C1,
        .sclPins = {
            I2CPINDEF(PA8, IOC_PA08_FUNC_CTL_I2C1_SCL),
            I2CPINDEF(PA25, IOC_PA25_FUNC_CTL_I2C1_SCL),
            I2CPINDEF(PB24, IOC_PB24_FUNC_CTL_I2C1_SCL),
            I2CPINDEF(PC15, IOC_PC15_FUNC_CTL_I2C1_SCL),
            I2CPINDEF(PY6, IOC_PY06_FUNC_CTL_I2C1_SCL),
        },
        .sdaPins = {
            I2CPINDEF(PA9, IOC_PA09_FUNC_CTL_I2C1_SDA),
            I2CPINDEF(PA26, IOC_PA26_FUNC_CTL_I2C1_SDA),
            I2CPINDEF(PB25, IOC_PB25_FUNC_CTL_I2C1_SDA),
            I2CPINDEF(PC16, IOC_PC16_FUNC_CTL_I2C1_SDA),
            I2CPINDEF(PY7, IOC_PY07_FUNC_CTL_I2C1_SDA),
        },
        .rcc = clock_i2c1,
    },
#endif
#ifdef HPM6750
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *) HPM_I2C1,
        .sclPins = {
            I2CPINDEF(PA11, IOC_PA11_FUNC_CTL_I2C1_SCL),
            I2CPINDEF(PB0, IOC_PB00_FUNC_CTL_I2C1_SCL),
            I2CPINDEF(PD5, IOC_PD05_FUNC_CTL_I2C1_SCL),
            I2CPINDEF(PE11, IOC_PE11_FUNC_CTL_I2C1_SCL),
            I2CPINDEF(PF1, IOC_PF01_FUNC_CTL_I2C1_SCL),
        },
        .sdaPins = {
            I2CPINDEF(PA10, IOC_PA10_FUNC_CTL_I2C1_SDA),
            I2CPINDEF(PA31, IOC_PA31_FUNC_CTL_I2C1_SDA),
            I2CPINDEF(PD8, IOC_PD08_FUNC_CTL_I2C1_SDA),
            I2CPINDEF(PE10, IOC_PE10_FUNC_CTL_I2C1_SDA),
            I2CPINDEF(PF3, IOC_PF03_FUNC_CTL_I2C1_SDA),
        },
        .rcc = clock_i2c1,
    },
#endif
#endif
#ifdef USE_I2C_DEVICE_2
#ifdef HPM6360
    {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *) HPM_I2C2,
        .sclPins = {
            I2CPINDEF(PA19, IOC_PA19_FUNC_CTL_I2C2_SCL),
            I2CPINDEF(PB18, IOC_PB18_FUNC_CTL_I2C2_SCL),
            I2CPINDEF(PC9, IOC_PC09_FUNC_CTL_I2C2_SCL),
            I2CPINDEF(PZ2, IOC_PZ02_FUNC_CTL_I2C2_SCL),
        },
        .sdaPins = {
            I2CPINDEF(PA20, IOC_PA20_FUNC_CTL_I2C2_SDA),
            I2CPINDEF(PB19, IOC_PB19_FUNC_CTL_I2C2_SDA),
            I2CPINDEF(PC10, IOC_PC10_FUNC_CTL_I2C2_SDA),
            I2CPINDEF(PZ3, IOC_PZ03_FUNC_CTL_I2C2_SDA),
        },
        .rcc = clock_i2c2,
    },
#endif
#ifdef HPM6750
    {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *) HPM_I2C2,
        .sclPins = {
            I2CPINDEF(PA15, IOC_PA15_FUNC_CTL_I2C2_SCL),
            I2CPINDEF(PB4, IOC_PB04_FUNC_CTL_I2C2_SCL),
            I2CPINDEF(PD10, IOC_PD10_FUNC_CTL_I2C2_SCL),
            I2CPINDEF(PE13, IOC_PE13_FUNC_CTL_I2C2_SCL),
        },
        .sdaPins = {
            I2CPINDEF(PA14, IOC_PA14_FUNC_CTL_I2C2_SDA),
            I2CPINDEF(PB3, IOC_PB03_FUNC_CTL_I2C2_SDA),
            I2CPINDEF(PD9, IOC_PD09_FUNC_CTL_I2C2_SDA),
            I2CPINDEF(PE12, IOC_PE12_FUNC_CTL_I2C2_SDA),
        },
        .rcc = clock_i2c2,
    },
#endif
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

static volatile uint16_t i2cErrorCount = 0;

/*
 * i2cHandleHardwareFailure:
 * - For NACK (transient) and invalid arguments (caller error): only increment
 *   the error counter and set the error flag.  Neither condition indicates a
 *   stuck bus, so leave the peripheral and pins untouched.
 * - For bus errors / arbitration loss / timeout: attempt bus recovery via
 *   i2cUnstick (bit-bang clock pulses + STOP to free stuck slaves), then
 *   reinitialise the peripheral from scratch.
 */
static bool i2cHandleHardwareFailure(i2cDevice_e device, hpm_stat_t status)
{
    i2cState_t *state = &i2cDevice[device].state;

    i2cErrorCount++;
    state->error = true;
    state->busy = false;

    // These failures do not indicate a stuck bus — no reinit needed
    if (status == status_invalid_argument || status == status_i2c_no_ack || status == status_i2c_no_addr_hit) {
        return false;
    }
    // For bus errors, arbitration loss, or timeout, recover the bus and
    // peripheral hardware.
    i2cInit(device);
    // i2cInit() resets the transfer state while rebuilding the peripheral;
    // retain the failure indication for i2cBusy() and diagnostics.
    state->error = true;

    return false;
}

bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *) i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;
    if (state->busy) {
        return false;
    }
    // The HPM SDK rejects zero-length transfers.  Reject invalid buffers at
    // the API boundary too, before claiming or touching the hardware bus.
    if (len_ == 0 || !data) {
        return false;
    }

    state->busy = true;
    state->error = false;

    // Blocking transfer: the bus is idle again when this call returns.
    // reg_ == 0xFF means that the device protocol has no register-address
    // phase, matching the STM32 I2C API contract.
    const hpm_stat_t status = (reg_ == 0xFF) ? i2c_master_write(I2Cx, addr_, data, len_)
                              : i2c_master_address_write(I2Cx, addr_, (uint8_t *) &reg_, 1, data, len_);

    state->busy = false;

    if (status_success != status) {
        return i2cHandleHardwareFailure(device, status);
    }
    return true;
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        if (error) {
            *error = true;
        }
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;

    if (error) {
        *error = state->error;
    }

    return state->busy;
}

bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *) i2cDevice[device].reg;
    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;
    if (state->busy) {
        return false;
    }
    // The HPM SDK rejects zero-length transfers.  Reject invalid buffers at
    // the API boundary too, before claiming or touching the hardware bus.
    if (len == 0 || !buf) {
        return false;
    }

    state->busy = true;
    state->error = false;

    // Blocking transfer: the bus is idle again when this call returns.
    // Without a register-address phase, start the read directly.
    const hpm_stat_t status = (reg_ == 0xFF) ? i2c_master_read(I2Cx, addr_, buf, len)
                              : i2c_master_address_read(I2Cx, addr_, (uint8_t *) &reg_, 1, buf, len);

    state->busy = false;

    if (status_success != status) {
        return i2cHandleHardwareFailure(device, status);
    }
    return true;
}

bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf);
}

static void i2cPinLoopbackEnable(IO_t io)
{
    if (!io) {
        return;
    }
    HPM_IOC->PAD[IOCIndex(io)].FUNC_CTL |= IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
}

static hpm_stat_t i2cInitMaster(I2C_TypeDef *I2Cx, uint32_t srcClockHz, uint16_t clockSpeedKhz)
{
    if (!srcClockHz || clockSpeedKhz < I2C_CLOCKSPEED_MIN_KHZ || clockSpeedKhz > I2C_CLOCKSPEED_MAX_KHZ) {
        return status_invalid_argument;
    }

    i2c_config_t config = {
        .is_10bit_addressing = false,
    };

    // Select the I2C specification timing class first.  The SDK configures
    // the spike suppression and data setup/hold fields for that class.
    if (clockSpeedKhz <= 100U) {
        config.i2c_mode = i2c_mode_normal;
    } else if (clockSpeedKhz <= 400U) {
        config.i2c_mode = i2c_mode_fast;
    } else {
        config.i2c_mode = i2c_mode_fast_plus;
    }

    hpm_stat_t status = i2c_init_master(I2Cx, srcClockHz, &config);

    if (status != status_success) {
        return status;
    }
    // The SDK modes have fixed target rates (100/400/1000 kHz).  Recalculate
    // T_SCLHI from Betaflight's configured rate while retaining the selected
    // mode's electrical timing fields.  Round up so the resulting bus clock
    // never exceeds the requested clockSpeed.
    const uint32_t targetHz = (uint32_t) clockSpeedKhz * 1000U;
    const uint32_t targetPeriodCycles = ((uint64_t) srcClockHz + targetHz - 1U) / targetHz;
    const uint32_t setup = I2Cx->SETUP;
    const uint32_t spikeCycles = I2C_SETUP_T_SP_GET(setup);
    const uint32_t sclRatio = I2C_SETUP_T_SCLRADIO_GET(setup) + 1U;
    const uint32_t tpmCycles = I2C_TPM_TPM_GET(I2Cx->TPM) + 1U;
    const uint32_t fixedCycles = 4U + (4U + 2U * spikeCycles) * tpmCycles;
    const uint32_t sclhiDivisor = tpmCycles * (1U + sclRatio);

    if (targetPeriodCycles <= fixedCycles) {
        return status_invalid_argument;
    }

    const uint32_t sclhi = (targetPeriodCycles - fixedCycles + sclhiDivisor - 1U) / sclhiDivisor;
    const uint32_t sclhiMax = I2C_SETUP_T_SCLHI_MASK >> I2C_SETUP_T_SCLHI_SHIFT;

    if (sclhi > sclhiMax) {
        return status_invalid_argument;
    }

    I2Cx->SETUP = (setup & ~I2C_SETUP_T_SCLHI_MASK) | I2C_SETUP_T_SCLHI_SET(sclhi);
    return status_success;
}

void i2cInit(i2cDevice_e device)
{
    hpm_stat_t stat;
    uint32_t freq;

    if (device == I2CINVALID) {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];
    const i2cHardware_t *hw = pDev->hardware;
    const IO_t scl = pDev->scl;
    const IO_t sda = pDev->sda;

    // Recovery re-enters i2cInit() after this device has claimed its pins.
    // Accept those I2C claims, but do not take a pin owned by another function.
    if (!hw ||
        (IOGetOwner(scl) != OWNER_FREE && IOGetOwner(scl) != OWNER_I2C_SCL) ||
        (IOGetOwner(sda) != OWNER_FREE && IOGetOwner(sda) != OWNER_I2C_SDA)) {
        return;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *) hw->reg;

    const bool hadError = pDev->state.error;
    memset(&pDev->state, 0, sizeof(pDev->state));
    pDev->state.error = hadError;

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable RCC
    if (hw->rcc) {
        clock_add_to_group(hw->rcc, 0);
    }
    // Recover the pins selected by the Betaflight configuration rather than
    // the board SDK's fixed default I2C pin route.
    if (!i2cUnstick(scl, sda)) {
        pDev->state.error = true;
        return;
    }
    // Init pins
    IOConfigGPIOAF(scl, (pDev->pullUp) ? IOCFG_AF_OD_UP : IOCFG_AF_OD, pDev->sclAF);
    IOConfigGPIOAF(sda, (pDev->pullUp) ? IOCFG_AF_OD_UP : IOCFG_AF_OD, pDev->sdaAF);

    // Enable I2C loopback on SCL/SDA pads (required for I2C peripheral to
    // correctly drive the open-drain lines and detect ACK)
    i2cPinLoopbackEnable(scl);
    i2cPinLoopbackEnable(sda);

    freq = clock_get_frequency(hw->rcc);
    stat = i2cInitMaster(I2Cx, freq, pDev->clockSpeed);
    if (stat != status_success) {
        // Do not publish an uninitialised peripheral to the transfer API.
        // i2cBusy() exposes the failure and subsequent transfers fail before
        // accessing the hardware.
        pDev->reg = NULL;
        pDev->state.error = true;
        i2cErrorCount++;
        return;
    }

    // A previous failed initialization clears reg.  Publish the peripheral
    // again only after the retry has completed successfully.
    pDev->reg = hw->reg;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

void i2cPinConfigure(const struct i2cConfig_s *i2cConfig)
{
    for (int index = 0; index < I2CDEV_COUNT; index++) {
        const i2cHardware_t *hardware = &i2cHardware[index];

        if (!hardware->reg) {
            continue;
        }

        const i2cDevice_e device = hardware->device;
        i2cDevice_t *pDev = &i2cDevice[device];

        memset(pDev, 0, sizeof(*pDev));

        for (int pindex = 0; pindex < I2C_PIN_SEL_MAX; pindex++) {
            if (i2cConfig[device].ioTagScl == hardware->sclPins[pindex].ioTag) {
                pDev->scl = IOGetByTag(i2cConfig[device].ioTagScl);
#if I2C_TRAIT_AF_PIN
                pDev->sclAF = hardware->sclPins[pindex].af;
#endif
            }
            if (i2cConfig[device].ioTagSda == hardware->sdaPins[pindex].ioTag) {
                pDev->sda = IOGetByTag(i2cConfig[device].ioTagSda);
#if I2C_TRAIT_AF_PIN
                pDev->sdaAF = hardware->sdaPins[pindex].af;
#endif
            }
        }

        if (pDev->scl && pDev->sda) {
            pDev->hardware = hardware;
            pDev->reg = hardware->reg;
            pDev->pullUp = i2cConfig[device].pullUp;
            pDev->clockSpeed = i2cConfig[device].clockSpeed;
        }
    }
}
#endif
