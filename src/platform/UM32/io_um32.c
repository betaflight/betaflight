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

#include "build/atomic.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "platform/io_impl.h"
#include "platform/rcc.h"

#include "common/utils.h"

// io ports defs are stored in array by index now

struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

#if defined(UM324xF)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(GPIOA) },
    { RCC_AHB1(GPIOB) },
    { RCC_AHB1(GPIOC) },
    { RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },
    [(GPIOH_BASE - GPIOA_BASE) >> 10] = { RCC_AHB1(GPIOH) },
};
#else
# error "IO PortDefs not defined for MCU"
#endif

static volatile uint16_t openDrainPins[ARRAYLEN(ioPortDefs)];

static bool ioIsOpenDrain(IO_t io)
{
    return (openDrainPins[IO_GPIOPortIdx(io)] & IO_Pin(io)) != 0;
}

static void ioWriteOpenDrain(IO_t io, bool release)
{
    GPIO_TypeDef *gpio = IO_GPIO(io);
    const unsigned shift = 2 * IO_GPIOPinIdx(io);
    const uint32_t mode = release ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT;

    if (!release) {
        // Set the output latch low before enabling the output driver.
        gpio->CLR = IO_Pin(io);
    }
    MODIFY_REG(gpio->MODE, GPIO_MODE << shift, mode << shift);
}
uint32_t IO_EXTI_Line(IO_t io)
{
    if (!io) {
        return 0;
    }
#if defined(UM324xF)
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
    return (IO_GPIO(io)->IDATA & IO_Pin(io));
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }
    if (ioIsOpenDrain(io)) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            ioWriteOpenDrain(io, hi);
        }
    } else if (hi) {
        IO_GPIO(io)->SET = IO_Pin(io);
    } else {
        IO_GPIO(io)->CLR = IO_Pin(io);
    }
}

void IOHi(IO_t io)
{
    IOWrite(io, true);
}

void IOLo(IO_t io)
{
    IOWrite(io, false);
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        GPIO_TypeDef *gpio = IO_GPIO(io);
        if (ioIsOpenDrain(io)) {
            const unsigned shift = 2 * IO_GPIOPinIdx(io);
            const bool drivingLow = ((gpio->MODE >> shift) & GPIO_MODE) == GPIO_MODE_OUTPUT;
            ioWriteOpenDrain(io, drivingLow);
        } else {
            gpio->ODATA ^= IO_Pin(io);
        }
    }
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    IOConfigGPIOAF(io, cfg, 0);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    const int port = IO_GPIOPortIdx(io);
    const uint16_t pin = IO_Pin(io);

    GPIO_InitTypeDef init = {
        .Pin = pin,
        .Mode = (cfg >> 0) & 0x03,
        .Speed = (cfg >> 2) & 0x03,
        .Pull = ioConfigGetPull(cfg),
        .Driving_Ability = GPIO_DS_14MA,
        .Alternate = af
    };

    // GPIO mode fields share a register with pins switched by the UART IRQ.
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        RCC_ClockCmd(ioPortDefs[port].rcc, ENABLE);
        HAL_GPIO_Init(IO_GPIO(io), &init);
        if (cfg == IOCFG_OUT_OD) {
            openDrainPins[port] |= pin;
        } else {
            openDrainPins[port] &= ~pin;
        }
    }
}
