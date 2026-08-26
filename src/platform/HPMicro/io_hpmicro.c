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

// Low-level IO pin abstraction for HPMicro.
// Maps IO tags to IOC pad control registers and implements the pad control-word
// programming (IOConfigGPIOAF) used by all other platform drivers.

#include "platform.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "io_def.h"
#include "io_hpmicro.h"
#include "hpm_ioc_regs.h"
#include "hpm_gpio_drv.h"

// Default PAD_CTL drive strength for output pins (DS = 3: moderate drive,
// roughly half of the maximum).  Peripheral drivers (SPI, I2C) override this
// with higher per-signal values.  The DS field is at bits 0–2 on both the
// HPM6300 and HPM6700 families.
#define HPM_GPIO_OUTPUT_PAD_DEFAULTS IOC_PAD_PAD_CTL_DS_SET(3)

static const uint32_t ioPortUsedMasks[DEFIO_PORT_USED_COUNT] = {
    DEFIO_PORT_A_USED_MASK,
    DEFIO_PORT_B_USED_MASK,
    DEFIO_PORT_C_USED_MASK,
#ifdef HPM6750
    DEFIO_PORT_D_USED_MASK,
    DEFIO_PORT_E_USED_MASK,
    DEFIO_PORT_F_USED_MASK,
#elif defined(HPM6360)
    0,
    0,
    0,
#endif
    DEFIO_PORT_XYZ_USED_MASK,
};

static uint16_t iocIndexFromLogicalIndex(unsigned index)
{
    const unsigned gpioId = index / DEFIO_PORT_PINS;
    const unsigned pin = index % DEFIO_PORT_PINS;

    if (gpioId == DEFIO_GPIOID__XYZ) {
        if (pin < 8) {
            return HPM_X_IOC_BASE + pin;
        }
        if (pin < 16) {
            return HPM_Y_IOC_BASE + pin - 8;
        }
        return HPM_Z_IOC_BASE + pin - 16;
    }

    return index;
}

void IOInitGlobal(void)
{
    ioRec_t *ioRec = ioRecs;

    for (unsigned pin = 0; pin < DEFIO_PIN_USED_COUNT; pin++) {
        ioRec->pin = iocIndexFromLogicalIndex(pin);
        ioRec++;
    }
}

void unusedPinsInit(void)
{
    /* Deliberately left empty for HPMicro: board-specific pinmux already
     * configures the pads, and a blanket input-pull-up here could conflict
     * with special-domain (PY/PZ) or analog pins. */
}

// zero based pin index
int IO_GPIOPinIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
    uint32_t iocIdx = IOCIndex(io);

    return GPIO_GET_PIN_INDEX(iocIdx);
}

int IO_GPIOPortIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
    uint32_t iocIdx = IOCIndex(io);

    return GPIO_GET_PORT_INDEX(iocIdx);
}

// Stub implementations for the io_impl.h compatibility layer.
// Current generic code does not call these on HPMicro, but providing
// them avoids silent link failures if future code does.
int IO_GPIO_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

int IO_GPIO_PortSource(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

uint32_t IO_EXTI_Line(IO_t io)
{
    if (!io) {
        return 0;
    }
    return 1U << IO_GPIOPinIdx(io);
}

FAST_CODE bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }
    uint32_t iocIdx = IOCIndex(io);

    if (gpio_read_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(iocIdx), GPIO_GET_PIN_INDEX(iocIdx))) {
        return true;
    } else {
        return false;
    }
}

FAST_CODE void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }
    uint32_t iocIdx = IOCIndex(io);

    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(iocIdx), GPIO_GET_PIN_INDEX(iocIdx), hi);
}

// Direct SET/CLEAR register writes: saves the branch inside gpio_write_pin()
// and makes the intent explicit even without compiler optimisations.
FAST_CODE void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
    uint32_t iocIdx = IOCIndex(io);

    HPM_GPIO0->DO[GPIO_GET_PORT_INDEX(iocIdx)].SET = 1U << GPIO_GET_PIN_INDEX(iocIdx);
}

FAST_CODE void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
    uint32_t iocIdx = IOCIndex(io);

    HPM_GPIO0->DO[GPIO_GET_PORT_INDEX(iocIdx)].CLEAR = 1U << GPIO_GET_PIN_INDEX(iocIdx);
}

FAST_CODE void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }
    uint32_t iocIdx = IOCIndex(io);

    gpio_toggle_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(iocIdx), GPIO_GET_PIN_INDEX(iocIdx));
}

// IOConfigGPIO delegates to IOConfigGPIOAF with af = 0 (ALT0 = GPIO function).
// This mirrors the STM32 pattern and guarantees that all IOCFG_* constants are
// handled by a single, well-tested switch statement.
void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    IOConfigGPIOAF(io, cfg, 0);
}

// IOConfigBPIOC routes a PY/PZ power/battery-domain pad into the SoC domain
// so that the main IOC (HPM_IOC) can drive it.  Callers that configure PY/PZ
// pins directly (e.g. board pinmux) must call this before writing HPM_IOC.
void IOConfigBPIOC(IO_t io)
{
    if (!io) {
        return;
    }
    uint32_t iocIdx = IOCIndex(io);

    if (iocIdx >= HPM_Y_IOC_BASE && iocIdx <= HPM_Y_IOC_BASE + 7) {
        HPM_PIOC->PAD[(uint32_t) iocIdx].FUNC_CTL = 3;
    }
    if (iocIdx >= HPM_Z_IOC_BASE && iocIdx <= HPM_Z_IOC_BASE + 7) {
        HPM_BIOC->PAD[(uint32_t) iocIdx].FUNC_CTL = 3;
    }
}

IO_t IOGetByTag(ioTag_t tag)
{
    const int portIdx = DEFIO_TAG_GPIOID_REL(tag);
    const int pin = DEFIO_TAG_PIN_REL(tag);

    if (portIdx < 0 || portIdx >= DEFIO_PORT_USED_COUNT) {
        return NULL;
    }
    if (!(ioPortUsedMasks[portIdx] & (1U << pin))) {
        return NULL;
    }

    const int pinIdx = pin + portIdx * DEFIO_PORT_PINS;
    if (pinIdx >= DEFIO_PIN_USED_COUNT) {
        return NULL;
    }

    return &ioRecs[pinIdx];
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint32_t af)
{
    if (!io) {
        return;
    }
    uint32_t iocIdx = IOCIndex(io);
    const uint32_t port = GPIO_GET_PORT_INDEX(iocIdx);
    const uint32_t pin = GPIO_GET_PIN_INDEX(iocIdx);

    /* Route PY/PZ to the SoC domain before programming the main IOC. */
    IOConfigBPIOC(io);
    if (cfg == IOCFG_ANALOG) {
        af = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    }
    HPM_IOC->PAD[(uint32_t) iocIdx].FUNC_CTL = af;

    // Fail safe for unknown ioConfig_t values: leave the pad high impedance
    // unless the configuration is explicitly recognised as an output.
    bool input = true;
    uint32_t padCtl = 0;

    switch (cfg) {
    case IOCFG_AF_PP:
        input = false;
        padCtl = HPM_GPIO_OUTPUT_PAD_DEFAULTS;
        break;
    case IOCFG_AF_PP_UP:
        input = false;
        padCtl = HPM_GPIO_OUTPUT_PAD_DEFAULTS | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
        break;
    case IOCFG_AF_PP_PD:
        input = false;
        padCtl = HPM_GPIO_OUTPUT_PAD_DEFAULTS | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);
        break;
    case IOCFG_AF_OD:
        input = false;
        padCtl = HPM_GPIO_OUTPUT_PAD_DEFAULTS | IOC_PAD_PAD_CTL_OD_SET(1);
        break;
    case IOCFG_AF_OD_UP:
        input = false;
        padCtl =
            HPM_GPIO_OUTPUT_PAD_DEFAULTS | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) |
            IOC_PAD_PAD_CTL_OD_SET(1);
        break;
    case IOCFG_OUT_PP:
        input = false;
        padCtl = HPM_GPIO_OUTPUT_PAD_DEFAULTS;
        break;
    case IOCFG_OUT_PP_UP:
        input = false;
        padCtl = HPM_GPIO_OUTPUT_PAD_DEFAULTS | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
        break;
    case IOCFG_OUT_OD:
        input = false;
        padCtl = HPM_GPIO_OUTPUT_PAD_DEFAULTS | IOC_PAD_PAD_CTL_OD_SET(1);
        break;
    case IOCFG_IPD:
        padCtl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);
        break;
    case IOCFG_IPU:
        padCtl = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
        break;
    case IOCFG_IN_FLOATING:
    case IOCFG_ANALOG:
        break;
    default:
        break;
    }

    // Once a peripheral function owns the pad, its output-enable signal is
    // selected by the IOC mux; the GPIO OE bit is no longer needed.  Keep it
    // cleared so switching FUNC_CTL back to ALT0 cannot momentarily drive an
    // AF input such as UART RX or SPI MISO.
    if (af != 0) {
        input = true;
    }

    /* Program PAD_CTL before enabling the output driver so the pin is never
     * driven with an unconfigured (zero) pad-control word. */
    HPM_IOC->PAD[(uint32_t) iocIdx].PAD_CTL = padCtl;

    if (input) {
        gpio_set_pin_input(HPM_GPIO0, port, pin);
    } else {
        gpio_set_pin_output(HPM_GPIO0, port, pin);
    }
}
