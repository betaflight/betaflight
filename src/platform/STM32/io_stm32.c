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

#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "platform/io_impl.h"
#include "platform/rcc.h"

#include "common/utils.h"

// io ports defs are stored in array by index now
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

#if defined(STM32F4)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(GPIOA) },
    { RCC_AHB1(GPIOB) },
    { RCC_AHB1(GPIOC) },
    { RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },
    { RCC_AHB1(GPIOF) },
    { RCC_AHB1(GPIOG) },
};
#elif defined(STM32F7)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(GPIOA) },
    { RCC_AHB1(GPIOB) },
    { RCC_AHB1(GPIOC) },
    { RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },
    { RCC_AHB1(GPIOF) },
    { RCC_AHB1(GPIOG) },
};
#elif defined(STM32H7)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB4(GPIOA) },
    { RCC_AHB4(GPIOB) },
    { RCC_AHB4(GPIOC) },
    { RCC_AHB4(GPIOD) },
    { RCC_AHB4(GPIOE) },
    { RCC_AHB4(GPIOF) },
    { RCC_AHB4(GPIOG) },
    { RCC_AHB4(GPIOH) },
#if !(defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx) || defined(STM32H735xx))
    { RCC_AHB4(GPIOI) },
#endif
};
#elif defined(STM32G4)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB2(GPIOA) },
    { RCC_AHB2(GPIOB) },
    { RCC_AHB2(GPIOC) },
    { RCC_AHB2(GPIOD) },
    { RCC_AHB2(GPIOE) },
    { RCC_AHB2(GPIOF) },
};
#elif defined(STM32C5)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB2(GPIOA) },
    { RCC_AHB2(GPIOB) },
    { RCC_AHB2(GPIOC) },
    { RCC_AHB2(GPIOD) },
    { RCC_AHB2(GPIOE) },
    // STM32C562 lacks GPIOF
#if !defined(STM32C562xx)
    { RCC_AHB2(GPIOF) },
#endif
};
#elif defined(STM32H5)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB2(GPIOA) },
    { RCC_AHB2(GPIOB) },
    { RCC_AHB2(GPIOC) },
    { RCC_AHB2(GPIOD) },
    { RCC_AHB2(GPIOE) },
    { RCC_AHB2(GPIOF) },
    { RCC_AHB2(GPIOG) },
    { RCC_AHB2(GPIOH) },
    { RCC_AHB2(GPIOI) },
};
#elif defined(STM32N6)
// Index by IO_GPIOPortIdx() -- (GPIO_BASE - GPIOA_BASE) >> 10. The N6 GPIO
// blocks are still on a 1 KiB stride, but ports I..M don't exist (the bank
// jumps from H@idx 7 straight to N@idx 13), so those slots stay zero and
// are never reached because no DEFIO_TAG__PI..PM are produced (target.h
// doesn't enable TARGET_IO_PORTI..M).
//
// Ports P (idx 15) and Q (idx 16) overflow the 4-bit port nibble in
// ioTag_t and are intentionally not exposed: flight-controller boards
// almost never reach for the largest pin-count packages, so widening
// ioTag_t to uint16_t isn't worth the cross-cutting churn.
const struct ioPortDef_s ioPortDefs[] = {
    [0]  = { RCC_AHB4(GPIOA) },
    [1]  = { RCC_AHB4(GPIOB) },
    [2]  = { RCC_AHB4(GPIOC) },
    [3]  = { RCC_AHB4(GPIOD) },
    [4]  = { RCC_AHB4(GPIOE) },
    [5]  = { RCC_AHB4(GPIOF) },
    [6]  = { RCC_AHB4(GPIOG) },
    [7]  = { RCC_AHB4(GPIOH) },
    // [8..12] reserved (I..M absent on N6)
    [13] = { RCC_AHB4(GPION) },
    [14] = { RCC_AHB4(GPIOO) },
};
#else
# error "IO PortDefs not defined for MCU"
#endif

// mask on stm32f103, bit index on stm32f303
uint32_t IO_EXTI_Line(IO_t io)
{
    if (!io) {
        return 0;
    }
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(STM32H5) || defined(STM32C5) || defined(STM32N6)
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
#if defined(USE_FULL_LL_DRIVER)
    return (LL_GPIO_ReadInputPort(IO_GPIO(io)) & IO_Pin(io));
#elif defined(USE_HAL_DRIVER)
    return !! HAL_GPIO_ReadPin(IO_GPIO(io), IO_Pin(io));
#else
    return (IO_GPIO(io)->IDR & IO_Pin(io));
#endif
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }
#if defined(USE_FULL_LL_DRIVER)
    LL_GPIO_SetOutputPin(IO_GPIO(io), IO_Pin(io) << (hi ? 0 : 16));
#elif defined(USE_HAL_DRIVER)
    HAL_GPIO_WritePin(IO_GPIO(io), IO_Pin(io), hi ? GPIO_PIN_SET : GPIO_PIN_RESET);
#elif defined(STM32F4)
    if (hi) {
        IO_GPIO(io)->BSRRL = IO_Pin(io);
    } else {
        IO_GPIO(io)->BSRRH = IO_Pin(io);
    }
#else
    IO_GPIO(io)->BSRR = IO_Pin(io) << (hi ? 0 : 16);
#endif
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
#if defined(USE_FULL_LL_DRIVER)
    LL_GPIO_SetOutputPin(IO_GPIO(io), IO_Pin(io));
#elif defined(USE_HAL_DRIVER)
    HAL_GPIO_WritePin(IO_GPIO(io), IO_Pin(io), GPIO_PIN_SET);
#elif defined(STM32F4)
    IO_GPIO(io)->BSRRL = IO_Pin(io);
#else
    IO_GPIO(io)->BSRR = IO_Pin(io);
#endif
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
#if defined(USE_FULL_LL_DRIVER)
    LL_GPIO_ResetOutputPin(IO_GPIO(io), IO_Pin(io));
#elif defined(USE_HAL_DRIVER)
    HAL_GPIO_WritePin(IO_GPIO(io), IO_Pin(io), GPIO_PIN_RESET);
#elif defined(STM32F4)
    IO_GPIO(io)->BSRRH = IO_Pin(io);
#else
    IO_GPIO(io)->BRR = IO_Pin(io);
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
#if defined(USE_FULL_LL_DRIVER)
    if (LL_GPIO_ReadOutputPort(IO_GPIO(io)) & mask) {
        mask <<= 16;   // bit is set, shift mask to reset half
    }
    LL_GPIO_SetOutputPin(IO_GPIO(io), mask);
#elif defined(USE_HAL_DRIVER)
    UNUSED(mask);
    HAL_GPIO_TogglePin(IO_GPIO(io), IO_Pin(io));
#elif defined(STM32F4)
    if (IO_GPIO(io)->ODR & mask) {
        IO_GPIO(io)->BSRRH = mask;
    } else {
        IO_GPIO(io)->BSRRL = mask;
    }
#else
    if (IO_GPIO(io)->ODR & mask) {
        mask <<= 16; // bit is set, shift mask to reset half
    }
    IO_GPIO(io)->BSRR = mask;
#endif
}

#if defined(STM32H7) || defined(STM32H5) || defined(STM32G4) || defined(STM32N6)

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

    HAL_GPIO_Init(IO_GPIO(io), &init);
}

#elif defined(STM32C5)

// HAL2: no HAL_GPIO_Init or LL_GPIO_Init. Configure via direct LL register access.
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

    GPIO_TypeDef *gpio = IO_GPIO(io);
    uint16_t pin = IO_Pin(io);
    uint32_t pinPos = IO_GPIOPinIdx(io);

    uint32_t mode = (cfg >> 0) & 0x03;
    uint32_t speed = (cfg >> 2) & 0x03;
    uint32_t otype = (cfg >> 4) & 0x01;
    uint32_t pull = (cfg >> 5) & 0x03;

    LL_GPIO_SetPinMode(gpio, pin, mode);
    LL_GPIO_SetPinSpeed(gpio, pin, speed);
    LL_GPIO_SetPinOutputType(gpio, pin, otype);
    LL_GPIO_SetPinPull(gpio, pin, pull);

    if (mode == LL_GPIO_MODE_ALTERNATE) {
        if (pinPos < 8) {
            LL_GPIO_SetAFPin_0_7(gpio, pin, af);
        } else {
            LL_GPIO_SetAFPin_8_15(gpio, pin, af);
        }
    }
}

#elif defined(STM32F7)

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

    LL_GPIO_InitTypeDef init = {
        .Pin = IO_Pin(io),
        .Mode = (cfg >> 0) & 0x03,
        .Speed = (cfg >> 2) & 0x03,
        .OutputType = (cfg >> 4) & 0x01,
        .Pull = (cfg >> 5) & 0x03,
        .Alternate = af
    };

    LL_GPIO_Init(IO_GPIO(io), &init);
}

#elif defined(STM32F4)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),
        .GPIO_Mode = (cfg >> 0) & 0x03,
        .GPIO_Speed = (cfg >> 2) & 0x03,
        .GPIO_OType = (cfg >> 4) & 0x01,
        .GPIO_PuPd = (cfg >> 5) & 0x03,
    };
    GPIO_Init(IO_GPIO(io), &init);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);
    GPIO_PinAFConfig(IO_GPIO(io), IO_GPIO_PinSource(io), af);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),
        .GPIO_Mode = (cfg >> 0) & 0x03,
        .GPIO_Speed = (cfg >> 2) & 0x03,
        .GPIO_OType = (cfg >> 4) & 0x01,
        .GPIO_PuPd = (cfg >> 5) & 0x03,
    };
    GPIO_Init(IO_GPIO(io), &init);
}

#else
# warning MCU not set
#endif
