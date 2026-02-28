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

#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "platform/rcc.h"

#include "common/utils.h"

// io ports defs are stored in array by index now
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

#if defined(GD32F4)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(PA) },
    { RCC_AHB1(PB) },
    { RCC_AHB1(PC) },
    { RCC_AHB1(PD) },
    { RCC_AHB1(PE) },
    { RCC_AHB1(PF) },
};
#else
# error "IO PortDefs not defined for MCU"
#endif

uint32_t IO_EXTI_Line(IO_t io)
{
    if (!io) {
        return 0;
    }
#if defined(GD32F4)
    return 1 << IO_GPIOPinIdx(io);
#elif defined(SIMULATOR_BUILD)
    return 0;
#else
# error "Unknown target type"
#endif
    return 0;
}

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }

    return (GPIO_ISTAT(PERIPH_INT(IO_GPIO(io))) & IO_Pin(io));
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }

    if (hi) {
        GPIO_BOP(PERIPH_INT(IO_GPIO(io))) = IO_Pin(io);
    } else {
        GPIO_BC(PERIPH_INT(IO_GPIO(io))) = IO_Pin(io);
    }
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }

    GPIO_BOP(PERIPH_INT(IO_GPIO(io))) = IO_Pin(io);
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
#if defined(GD32F4)
    GPIO_BC(PERIPH_INT(IO_GPIO(io))) = IO_Pin(io);
#endif
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }

    uint32_t mask = IO_Pin(io);
    // For GD32F4,use toggle register to toggle GPIO pin status
#if defined(GD32F4)
    GPIO_TG(PERIPH_INT(IO_GPIO(io))) = mask;
#endif
}

#if defined(GD32F4)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    gpio_mode_set(PERIPH_INT(IO_GPIO(io)), ((cfg >> 0) & 0x03), ((cfg >> 5) & 0x03), IO_Pin(io));
    gpio_output_options_set(PERIPH_INT(IO_GPIO(io)), ((cfg >> 4) & 0x01), ((cfg >> 2) & 0x03), IO_Pin(io));
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    gpio_af_set(PERIPH_INT(IO_GPIO(io)), af, IO_Pin(io));
    gpio_mode_set(PERIPH_INT(IO_GPIO(io)), ((cfg >> 0) & 0x03), ((cfg >> 5) & 0x03), IO_Pin(io));
    gpio_output_options_set(PERIPH_INT(IO_GPIO(io)), ((cfg >> 4) & 0x01), ((cfg >> 2) & 0x03), IO_Pin(io));
}

#else
# warning MCU not set
#endif
