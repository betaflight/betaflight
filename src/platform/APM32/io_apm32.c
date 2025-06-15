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

const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(PA) },
    { RCC_AHB1(PB) },
    { RCC_AHB1(PC) },
    { RCC_AHB1(PD) },
    { RCC_AHB1(PE) },
    { RCC_AHB1(PF) },
};

// mask on stm32f103, bit index on stm32f303
uint32_t IO_EXTI_Line(IO_t io)
{
    if (!io) {
        return 0;
    }
#if defined(APM32F4)
    return 1 << IO_GPIOPinIdx(io);
#elif defined(SIMULATOR_BUILD)
    return 0;
#else
# error "Unknown target type"
#endif
}

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }
#if defined(USE_FULL_DDL_DRIVER)
    return (DDL_GPIO_ReadInputPort(IO_GPIO(io)) & IO_Pin(io));
#elif defined(USE_DAL_DRIVER)
    return !! DAL_GPIO_ReadPin(IO_GPIO(io), IO_Pin(io));
#endif
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }
#if defined(USE_FULL_DDL_DRIVER)
    DDL_GPIO_SetOutputPin(IO_GPIO(io), IO_Pin(io) << (hi ? 0 : 16));
#elif defined(USE_DAL_DRIVER)
    DAL_GPIO_WritePin(IO_GPIO(io), IO_Pin(io), hi ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
#if defined(USE_FULL_DDL_DRIVER)
    DDL_GPIO_SetOutputPin(IO_GPIO(io), IO_Pin(io));
#elif defined(USE_DAL_DRIVER)
    DAL_GPIO_WritePin(IO_GPIO(io), IO_Pin(io), GPIO_PIN_SET);
#endif
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
#if defined(USE_FULL_DDL_DRIVER)
    DDL_GPIO_ResetOutputPin(IO_GPIO(io), IO_Pin(io));
#elif defined(USE_DAL_DRIVER)
    DAL_GPIO_WritePin(IO_GPIO(io), IO_Pin(io), GPIO_PIN_RESET);
#endif
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }

    uint32_t mask = IO_Pin(io);
    // Read pin state from ODR but write to BSRR because it only changes the pins
    // high in the mask value rather than all pins. XORing ODR directly risks
    // setting other pins incorrectly because it change all pins' state.
#if defined(USE_FULL_DDL_DRIVER)
    if (DDL_GPIO_ReadOutputPort(IO_GPIO(io)) & mask) {
        mask <<= 16;   // bit is set, shift mask to reset half
    }
    DDL_GPIO_SetOutputPin(IO_GPIO(io), mask);
#elif defined(USE_DAL_DRIVER)
    UNUSED(mask);
    DAL_GPIO_TogglePin(IO_GPIO(io), IO_Pin(io));
#endif
}

#if defined(USE_DAL_DRIVER)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    IOConfigGPIOAF(io, cfg, 0);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .Pin = IO_Pin(io),
        .Mode = (cfg >> 0) & 0x13,
        .Speed = (cfg >> 2) & 0x03,
        .Pull = (cfg >> 5) & 0x03,
        .Alternate = af
    };

    DAL_GPIO_Init(IO_GPIO(io), &init);
}

#elif defined(USE_FULL_DDL_DRIVER)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    IOConfigGPIOAF(io, cfg, 0);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    DDL_GPIO_InitTypeDef init = {
        .Pin = IO_Pin(io),
        .Mode = (cfg >> 0) & 0x03,
        .Speed = (cfg >> 2) & 0x03,
        .OutputType = (cfg >> 4) & 0x01,
        .Pull = (cfg >> 5) & 0x03,
        .Alternate = af
    };

    DDL_GPIO_Init(IO_GPIO(io), &init);
}
#else
# warning MCU not set
#endif
