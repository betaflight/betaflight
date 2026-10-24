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
 * RGPIO external interrupts. Each block has two vectors and nonsecure Linux uses the first,
 * so every pin here takes the second and the pins of a block share its priority.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"

#define EXTI_PORT_COUNT         NXP_GPIO_PORT_COUNT

// ICR IRQC values that raise an interrupt; zero leaves the flag off as well.
#define EXTI_IRQC_RISING_EDGE   0x9
#define EXTI_IRQC_FALLING_EDGE  0xa
#define EXTI_IRQC_BOTH_EDGES    0xb

// The block's second interrupt line, which the M7 claims as secure.
#define EXTI_IRQ_LINE           1

static extiCallbackRec_t *extiHandlers[EXTI_PORT_COUNT][DEFIO_PORT_PINS];

// Pins with their interrupt enabled, and the IRQC each pin gets when enabled.
static uint32_t extiMask[EXTI_PORT_COUNT];
static uint8_t extiIrqc[EXTI_PORT_COUNT][DEFIO_PORT_PINS];
static uint8_t extiPortPriority[EXTI_PORT_COUNT];

static const IRQn_Type extiIrq[EXTI_PORT_COUNT] = {
    GPIO1_1_IRQn, GPIO2_1_IRQn, GPIO3_1_IRQn, GPIO4_1_IRQn, GPIO5_1_IRQn,
};

static RGPIO_Type *extiGpio(IO_t io)
{
    return (RGPIO_Type *)IO_Rec(io)->gpio;
}

// The ICR value for the given IRQC; ISF is write one to clear.
static uint32_t extiIcr(uint32_t irqc)
{
    return RGPIO_ICR_ISF_MASK | RGPIO_ICR_IRQS(EXTI_IRQ_LINE) | RGPIO_ICR_IRQC(irqc);
}

void EXTIInit(void)
{
    memset(extiHandlers, 0, sizeof(extiHandlers));
    memset(extiMask, 0, sizeof(extiMask));
    memset(extiIrqc, 0, sizeof(extiIrqc));
    memset(extiPortPriority, 0xff, sizeof(extiPortPriority));
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

    RGPIO_Type *gpio = extiGpio(io);

    // A secure line and pin, or the M7's ICR writes are ignored.
    // ICNS is read-modify-write, so a board puts M7 pins only in RGPIO blocks Linux does not drive.
    gpio->ICNS &= ~(1u << EXTI_IRQ_LINE);
    IOConfigGPIO(io, config);

    // Stay disabled while the trigger changes, so a partly set up pin cannot fire.
    extiMask[port] &= ~(1u << pin);
    gpio->ICR[pin] = extiIcr(0);

    extiHandlers[port][pin] = cb;

    extiIrqc[port][pin] = (trigger == BETAFLIGHT_EXTI_TRIGGER_BOTH) ? EXTI_IRQC_BOTH_EDGES
                        : (trigger == BETAFLIGHT_EXTI_TRIGGER_FALLING) ? EXTI_IRQC_FALLING_EDGE
                        : EXTI_IRQC_RISING_EDGE;

    // Every pin of a block shares its vector, so keep the most urgent request.
    if (extiPortPriority[port] > irqPriority) {
        extiPortPriority[port] = irqPriority;

        NVIC_SetPriority(extiIrq[port], NVIC_PRIORITY_LEVEL(irqPriority));
        NVIC_EnableIRQ(extiIrq[port]);
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

    const int pin = IO_Pin(io);

    extiMask[port] |= 1u << pin;
    extiGpio(io)->ICR[pin] = extiIcr(extiIrqc[port][pin]);
}

void EXTIDisable(IO_t io)
{
    const int port = IO_GPIOPortIdx(io);

    if (!io || port < 0 || port >= EXTI_PORT_COUNT) {
        return;
    }

    const int pin = IO_Pin(io);

    extiMask[port] &= ~(1u << pin);
    extiGpio(io)->ICR[pin] = extiIcr(0);
}

static FAST_IRQ_HANDLER void extiHandlePort(unsigned port, RGPIO_Type *gpio)
{
    // Another core may own pins on this block, so only the enabled pins are read and cleared.
    uint32_t pending = gpio->ISFR[EXTI_IRQ_LINE] & extiMask[port];

    // Clear before the callbacks so an edge during a callback is not lost.
    gpio->ISFR[EXTI_IRQ_LINE] = pending;

    while (pending) {
        const unsigned pin = 31 - __CLZ(pending);
        pending &= ~(1u << pin);

        extiCallbackRec_t *cb = extiHandlers[port][pin];
        if (cb) {
            cb->fn(cb);
        }
    }
}

// Handlers named by the vector table in the startup file.
#define EXTI_PORT_HANDLERS(portIdx, block)                  \
    FAST_IRQ_HANDLER void block ## _1_IRQHandler(void)      \
    {                                                       \
        extiHandlePort(portIdx, block);                     \
    }

EXTI_PORT_HANDLERS(0, GPIO1)
EXTI_PORT_HANDLERS(1, GPIO2)
EXTI_PORT_HANDLERS(2, GPIO3)
EXTI_PORT_HANDLERS(3, GPIO4)
EXTI_PORT_HANDLERS(4, GPIO5)
