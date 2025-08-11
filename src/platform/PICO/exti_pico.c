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
#include <string.h>
#include "drivers/exti.h"
#include "common/utils.h"
#include "drivers/io_impl.h"

#include "hardware/gpio.h"

typedef struct {
    extiCallbackRec_t* handler;
} extiChannelRec_t;

static extiChannelRec_t extiChannelRecs[DEFIO_USED_COUNT];
static uint32_t extiEventMask[DEFIO_USED_COUNT];

void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger)
{
    if (!io) {
        return;
    }

    uint32_t gpio = IO_Pin(io);

    UNUSED(irqPriority); // Just stick with default GPIO irq priority for now
    UNUSED(config); // TODO consider pullup/pulldown etc. Needs fixing first in platform.h

    // Ensure the GPIO is initialised and not being used for some other function
    gpio_init(gpio);

    extiChannelRec_t *rec = &extiChannelRecs[gpio];
    rec->handler = cb;

    switch(trigger) {
    case BETAFLIGHT_EXTI_TRIGGER_RISING:
    default:
        extiEventMask[gpio] = GPIO_IRQ_EDGE_RISE;
        break;

    case BETAFLIGHT_EXTI_TRIGGER_FALLING:
        extiEventMask[gpio] = GPIO_IRQ_EDGE_FALL;
        break;

    case BETAFLIGHT_EXTI_TRIGGER_BOTH:
        extiEventMask[gpio] = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
        break;
    }
}

void EXTIRelease(IO_t io)
{
    if (!io) {
        return;
    }

    EXTIDisable(io);
    extiChannelRec_t *rec = &extiChannelRecs[IO_Pin(io)];
    rec->handler = NULL;
}

static void EXTI_IRQHandler(uint gpio, uint32_t event_mask)
{
    // Call the registered handler for this GPIO
    if (extiChannelRecs[gpio].handler) {
        extiChannelRecs[gpio].handler->fn(extiChannelRecs[gpio].handler);
    }

    // Acknowledge the interrupt
    gpio_acknowledge_irq(gpio, event_mask);
}

void EXTIInit(void)
{
    // Clear all the callbacks
    memset(extiChannelRecs, 0, sizeof(extiChannelRecs));

    // Register the shared handler for GPIO interrupts
    gpio_set_irq_callback(EXTI_IRQHandler);

    // Enable the interrupt
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn)
{
    self->fn = fn;
}

void EXTIEnable(IO_t io)
{
    if (!io) {
        return;
    }

    gpio_set_irq_enabled(IO_Pin(io), extiEventMask[IO_Pin(io)], true);
}

void EXTIDisable(IO_t io)
{
    if (!io) {
        return;
    }

    gpio_set_irq_enabled(IO_Pin(io), 0, false);
}
