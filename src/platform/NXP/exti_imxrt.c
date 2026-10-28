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
 * External interrupts for i.MXRT GPIO blocks. Each block has one vector per
 * half port (GPIO13 on the 11xx has one for all 32 pins), so pins in a half
 * share a priority.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"

// Ports A..E are GPIO1..GPIO5; the 11xx adds GPIO6 as port F and GPIO13 as port G.
#define EXTI_PORT_COUNT     NXP_GPIO_PORT_COUNT

// Two ICR bits per pin select the trigger; ICR1 covers pins 0..15, ICR2 the rest.
#define EXTI_ICR_RISING_EDGE    2
#define EXTI_ICR_FALLING_EDGE   3
#define EXTI_ICR_PINS_PER_REG   16

#define EXTI_HALVES_PER_PORT    2

static extiCallbackRec_t *extiHandlers[EXTI_PORT_COUNT][DEFIO_PORT_PINS];

// Copy of each port's IMR; nothing outside this file writes IMR.
static uint32_t extiMask[EXTI_PORT_COUNT];
static uint8_t extiGroupPriority[EXTI_PORT_COUNT][EXTI_HALVES_PER_PORT];

static const IRQn_Type extiIrqLow[] = {
    GPIO1_Combined_0_15_IRQn,
    GPIO2_Combined_0_15_IRQn,
    GPIO3_Combined_0_15_IRQn,
    GPIO4_Combined_0_15_IRQn,
    GPIO5_Combined_0_15_IRQn,
#if defined(IMXRT11XX)
    GPIO6_Combined_0_15_IRQn,
    GPIO13_Combined_0_31_IRQn,
#endif
};

static const IRQn_Type extiIrqHigh[] = {
    GPIO1_Combined_16_31_IRQn,
    GPIO2_Combined_16_31_IRQn,
    GPIO3_Combined_16_31_IRQn,
    GPIO4_Combined_16_31_IRQn,
    GPIO5_Combined_16_31_IRQn,
#if defined(IMXRT11XX)
    GPIO6_Combined_16_31_IRQn,
    GPIO13_Combined_0_31_IRQn,
#endif
};

STATIC_ASSERT(ARRAYLEN(extiIrqLow) == EXTI_PORT_COUNT, "a port is missing from extiIrqLow");
STATIC_ASSERT(ARRAYLEN(extiIrqHigh) == EXTI_PORT_COUNT, "a port is missing from extiIrqHigh");

static GPIO_Type *extiGpio(IO_t io)
{
    return (GPIO_Type *)IO_Rec(io)->gpio;
}

// Priority entry for a pin; a port with one vector for both halves uses entry 0.
static unsigned extiHalf(unsigned port, int pin)
{
    if (extiIrqLow[port] == extiIrqHigh[port]) {
        return 0;
    }

    return (pin < EXTI_ICR_PINS_PER_REG) ? 0 : 1;
}

void EXTIInit(void)
{
    memset(extiHandlers, 0, sizeof(extiHandlers));
    memset(extiMask, 0, sizeof(extiMask));
    memset(extiGroupPriority, 0xff, sizeof(extiGroupPriority));
}

void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn)
{
    self->fn = fn;
}

void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger)
{
    const int port = IO_GPIOPortIdx(io);
    const int pin = IO_GPIOPinIdx(io);

    if (port < 0 || port >= EXTI_PORT_COUNT || pin < 0 || pin >= DEFIO_PORT_PINS) {
        return;
    }

    GPIO_Type *gpio = extiGpio(io);
    const uint32_t mask = 1u << pin;

    // Stay masked while the trigger changes, so a half set up pin cannot fire.
    extiMask[port] &= ~mask;
    gpio->IMR = extiMask[port];

    extiHandlers[port][pin] = cb;

    IOConfigGPIO(io, config);

    if (trigger == BETAFLIGHT_EXTI_TRIGGER_BOTH) {
        gpio->EDGE_SEL |= mask;
    } else {
        gpio->EDGE_SEL &= ~mask;

        const uint32_t icr = (trigger == BETAFLIGHT_EXTI_TRIGGER_FALLING)
                           ? EXTI_ICR_FALLING_EDGE : EXTI_ICR_RISING_EDGE;
        const unsigned shift = (pin % EXTI_ICR_PINS_PER_REG) * 2;
        volatile uint32_t *icrReg = (pin < EXTI_ICR_PINS_PER_REG) ? &gpio->ICR1 : &gpio->ICR2;

        *icrReg = (*icrReg & ~(3u << shift)) | (icr << shift);
    }

    // Drop any edge seen during setup; ISR is write one to clear.
    gpio->ISR = mask;

    // Every pin on a vector shares its priority, so keep the most urgent request.
    const unsigned half = extiHalf(port, pin);
    if (extiGroupPriority[port][half] > irqPriority) {
        extiGroupPriority[port][half] = irqPriority;

        const IRQn_Type irqn = half ? extiIrqHigh[port] : extiIrqLow[port];
        NVIC_SetPriority(irqn, NVIC_PRIORITY_LEVEL(irqPriority));
        NVIC_EnableIRQ(irqn);
    }
}

void EXTIRelease(IO_t io)
{
    const int port = IO_GPIOPortIdx(io);
    const int pin = IO_GPIOPinIdx(io);

    if (port < 0 || port >= EXTI_PORT_COUNT || pin < 0 || pin >= DEFIO_PORT_PINS) {
        return;
    }

    EXTIDisable(io);
    extiHandlers[port][pin] = NULL;
}

void EXTIEnable(IO_t io)
{
    const int port = IO_GPIOPortIdx(io);

    if (!io || port < 0 || port >= EXTI_PORT_COUNT) {
        return;
    }

    GPIO_Type *gpio = extiGpio(io);
    const uint32_t mask = 1u << IO_Pin(io);

    gpio->ISR = mask;
    extiMask[port] |= mask;
    gpio->IMR = extiMask[port];
}

void EXTIDisable(IO_t io)
{
    const int port = IO_GPIOPortIdx(io);

    if (!io || port < 0 || port >= EXTI_PORT_COUNT) {
        return;
    }

    extiMask[port] &= ~(1u << IO_Pin(io));
    extiGpio(io)->IMR = extiMask[port];
}

static FAST_IRQ_HANDLER void extiHandlePort(unsigned port, GPIO_Type *gpio, uint32_t half)
{
    // A masked pin can still set its status bit.
    uint32_t pending = gpio->ISR & extiMask[port] & half;

    // Clear before the callbacks so an edge during a callback is not lost.
    gpio->ISR = pending;

    while (pending) {
        const unsigned pin = 31 - __CLZ(pending);
        pending &= ~(1u << pin);

        extiCallbackRec_t *cb = extiHandlers[port][pin];
        if (cb) {
            cb->fn(cb);
        }
    }
}

// Handlers named by the vector table in the family's startup file.
#define EXTI_PORT_HANDLERS(portIdx, block)                          \
    FAST_IRQ_HANDLER void block ## _Combined_0_15_IRQHandler(void)   \
    {                                                               \
        extiHandlePort(portIdx, block, 0x0000ffffu);                \
    }                                                               \
    FAST_IRQ_HANDLER void block ## _Combined_16_31_IRQHandler(void)  \
    {                                                               \
        extiHandlePort(portIdx, block, 0xffff0000u);                \
    }

EXTI_PORT_HANDLERS(0, GPIO1)
EXTI_PORT_HANDLERS(1, GPIO2)
EXTI_PORT_HANDLERS(2, GPIO3)
EXTI_PORT_HANDLERS(3, GPIO4)
EXTI_PORT_HANDLERS(4, GPIO5)
#if defined(IMXRT11XX)
EXTI_PORT_HANDLERS(5, GPIO6)

// GPIO13 has one vector for all 32 pins instead of one per half.
FAST_IRQ_HANDLER void GPIO13_Combined_0_31_IRQHandler(void)
{
    extiHandlePort(6, GPIO13, 0xffffffffu);
}
#endif
