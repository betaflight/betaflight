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
#include "drivers/rcc.h"

#include "common/utils.h"

// io ports defs are stored in array by index now
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(GPIOA) },
    { RCC_AHB1(GPIOB) },
    { RCC_AHB1(GPIOC) },
    { RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },
    { RCC_AHB1(GPIOF) },
    { RCC_AHB1(GPIOG) },
    { RCC_AHB1(GPIOH) }
};

uint32_t IO_EXTI_Line(IO_t io)
{
    if (!io) {
        return 0;
    }
    return 1 << IO_GPIOPinIdx(io);
}

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }
    return (IO_GPIO(io)->idt & IO_Pin(io));
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->scr = IO_Pin(io) << (hi ? 0 : 16);
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->scr = IO_Pin(io);
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->clr = IO_Pin(io);
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }

    uint32_t mask = IO_Pin(io);

    if (IO_GPIO(io)->odt & mask) {
        mask <<= 16; // bit is set, shift mask to reset half
    }
    IO_GPIO(io)->scr = mask;
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    gpio_init_type init = {
        .gpio_pins = IO_Pin(io),
        .gpio_mode = (cfg >> 0) & 0x03,
        .gpio_drive_strength = (cfg >> 2) & 0x03,
        .gpio_out_type = (cfg >> 4) & 0x01,
        .gpio_pull = (cfg >> 5) & 0x03,
    };
    gpio_init(IO_GPIO(io), &init);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    gpio_init_type init = {
        .gpio_pins = IO_Pin(io),
        .gpio_mode = (cfg >> 0) & 0x03,
        .gpio_drive_strength = (cfg >> 2) & 0x03,
        .gpio_out_type = (cfg >> 4) & 0x01,
        .gpio_pull = (cfg >> 5) & 0x03,
    };
    gpio_init(IO_GPIO(io), &init);
    gpio_pin_mux_config(IO_GPIO(io), IO_GPIO_PinSource(io), af);
}
