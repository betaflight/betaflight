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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/gpio_ll.h"
#pragma GCC diagnostic pop
#include "soc/gpio_struct.h"
#include "esp_rom_gpio.h"
#include "soc/interrupts.h"

#include "platform/interrupt.h"

typedef struct {
    extiCallbackRec_t* handler;
} extiChannelRec_t;

static extiChannelRec_t extiChannelRecs[DEFIO_USED_COUNT];
static uint32_t extiTriggerMask[DEFIO_USED_COUNT];

// Shared GPIO ISR - reads interrupt status and dispatches to per-pin callbacks.
// Handles pins 0-31 via the primary status register and pins 32-48 via the
// high status register.
static void EXTI_IRQHandler(void *arg)
{
    UNUSED(arg);

    uint32_t status = 0;

    // Process pins 0-31
    gpio_ll_get_intr_status(&GPIO, 0, &status);
    while (status) {
        uint32_t pin = __builtin_ctz(status);
        status &= ~(1U << pin);
        gpio_ll_clear_intr_status(&GPIO, (1U << pin));
        if (pin < DEFIO_USED_COUNT && extiChannelRecs[pin].handler && extiChannelRecs[pin].handler->fn) {
            extiChannelRecs[pin].handler->fn(extiChannelRecs[pin].handler);
        }
    }

    // Process pins 32-48
    gpio_ll_get_intr_status_high(&GPIO, 0, &status);
    while (status) {
        uint32_t bit = __builtin_ctz(status);
        uint32_t pin = bit + 32;
        status &= ~(1U << bit);
        gpio_ll_clear_intr_status_high(&GPIO, (1U << bit));
        if (pin < DEFIO_USED_COUNT && extiChannelRecs[pin].handler && extiChannelRecs[pin].handler->fn) {
            extiChannelRecs[pin].handler->fn(extiChannelRecs[pin].handler);
        }
    }
}

void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger)
{
    if (!io) {
        return;
    }

    UNUSED(irqPriority);
    UNUSED(config);

    uint32_t gpio = IO_Pin(io);
    extiChannelRec_t *rec = &extiChannelRecs[gpio];
    rec->handler = cb;

    // Map Betaflight trigger constants to ESP32 GPIO interrupt types
    gpio_int_type_t intrType;
    switch (trigger) {
    case BETAFLIGHT_EXTI_TRIGGER_RISING:
    default:
        intrType = GPIO_INTR_POSEDGE;
        break;
    case BETAFLIGHT_EXTI_TRIGGER_FALLING:
        intrType = GPIO_INTR_NEGEDGE;
        break;
    case BETAFLIGHT_EXTI_TRIGGER_BOTH:
        intrType = GPIO_INTR_ANYEDGE;
        break;
    }

    extiTriggerMask[gpio] = intrType;

    // Select GPIO function and enable input
    esp_rom_gpio_pad_select_gpio(gpio);
    gpio_ll_input_enable(&GPIO, gpio);

    // Configure the interrupt trigger type (but do not enable yet)
    gpio_ll_set_intr_type(&GPIO, gpio, intrType);
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

void EXTIInit(void)
{
    memset(extiChannelRecs, 0, sizeof(extiChannelRecs));
    memset(extiTriggerMask, 0, sizeof(extiTriggerMask));

    // Route GPIO peripheral interrupt to CPU interrupt line and register handler
    esp32IntrRoute(ESP32_CPU_INTR_GPIO, ETS_GPIO_INTR_SOURCE);
    esp32IntrRegister(ESP32_CPU_INTR_GPIO, EXTI_IRQHandler, NULL);
    esp32IntrEnable(ESP32_CPU_INTR_GPIO);
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

    gpio_ll_intr_enable_on_core(&GPIO, 0, IO_Pin(io));
}

void EXTIDisable(IO_t io)
{
    if (!io) {
        return;
    }

    gpio_ll_intr_disable(&GPIO, IO_Pin(io));
}
