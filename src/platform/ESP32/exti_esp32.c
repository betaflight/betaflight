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

typedef struct {
    extiCallbackRec_t* handler;
} extiChannelRec_t;

static extiChannelRec_t extiChannelRecs[DEFIO_USED_COUNT];

void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger)
{
    if (!io) {
        return;
    }

    UNUSED(irqPriority);
    UNUSED(config);
    UNUSED(trigger);

    uint32_t gpio = IO_Pin(io);
    extiChannelRec_t *rec = &extiChannelRecs[gpio];
    rec->handler = cb;

    // TODO: configure GPIO interrupt via gpio_isr_handler_add()
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
    // TODO: install GPIO ISR service via gpio_install_isr_service()
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
    // TODO: gpio_intr_enable(IO_Pin(io))
}

void EXTIDisable(IO_t io)
{
    if (!io) {
        return;
    }
    // TODO: gpio_intr_disable(IO_Pin(io))
}
