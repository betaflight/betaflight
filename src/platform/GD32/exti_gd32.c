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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_EXTI

#include "drivers/nvic.h"
#include "drivers/io_impl.h"
#include "drivers/exti.h"
#include "platform/rcc.h"

typedef struct {
    extiCallbackRec_t* handler;
} extiChannelRec_t;

extiChannelRec_t extiChannelRecs[16];

// IRQ grouping
#define EXTI_IRQ_GROUPS 7
//                                      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
static const uint8_t extiGroups[16] = { 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6 };
static uint8_t extiGroupPriority[EXTI_IRQ_GROUPS];

#if defined(GD32F4)
static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
    EXTI0_IRQn,
    EXTI1_IRQn,
    EXTI2_IRQn,
    EXTI3_IRQn,
    EXTI4_IRQn,
    EXTI5_9_IRQn,
    EXTI10_15_IRQn
};
#else
# warning "Unknown MCU"
#endif

static uint32_t triggerLookupTable[] = {
    [BETAFLIGHT_EXTI_TRIGGER_RISING]    = EXTI_TRIG_RISING,
    [BETAFLIGHT_EXTI_TRIGGER_FALLING]   = EXTI_TRIG_FALLING,
    [BETAFLIGHT_EXTI_TRIGGER_BOTH]      = EXTI_TRIG_BOTH
};

void EXTIInit(void)
{
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_ClockCmd(RCC_APB2(SYSCFG), ENABLE);

    memset(extiChannelRecs, 0, sizeof(extiChannelRecs));
    memset(extiGroupPriority, 0xff, sizeof(extiGroupPriority));
}

void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn)
{
    self->fn = fn;
}

void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger)
{
    int chIdx = IO_GPIOPinIdx(io);

    if (chIdx < 0) {
        return;
    }

    int group = extiGroups[chIdx];

    extiChannelRec_t *rec = &extiChannelRecs[chIdx];
    rec->handler = cb;

    EXTIDisable(io);

    IOConfigGPIO(io, config);

    syscfg_exti_line_config(IO_GPIO_PortSource(io), IO_GPIO_PinSource(io));

    uint32_t extiLine = IO_EXTI_Line(io);

    exti_init(extiLine, EXTI_INTERRUPT, triggerLookupTable[trigger]);

    if (extiGroupPriority[group] > irqPriority) {
        extiGroupPriority[group] = irqPriority;
        nvic_irq_enable(extiGroupIRQn[group], NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));
    }
}

void EXTIRelease(IO_t io)
{
    // don't forget to match cleanup with config
    EXTIDisable(io);

    const int chIdx = IO_GPIOPinIdx(io);

    if (chIdx < 0) {
        return;
    }

    extiChannelRec_t *rec = &extiChannelRecs[chIdx];
    rec->handler = NULL;
}

void EXTIEnable(IO_t io)
{
#if defined(GD32F4)
    uint32_t extiLine = IO_EXTI_Line(io);

    if (!extiLine) {
        return;
    }

    EXTI_INTEN |= extiLine;
#else
# error "Unknown MCU"
#endif
}

void EXTIDisable(IO_t io)
{
    (void) io;
#if defined(GD32F4)
    uint32_t extiLine = IO_EXTI_Line(io);

    if (!extiLine) {
        return;
    }

    EXTI_INTEN &= ~extiLine;
    EXTI_PD = extiLine;
#else
# error "Unknown MCU"
#endif
}

#define EXTI_EVENT_MASK 0xFFFF // first 16 bits only, see also definition of extiChannelRecs.

static void EXTI_IRQHandler(uint32_t mask)
{
    uint32_t exti_active = (EXTI_INTEN & EXTI_PD) & mask;

    EXTI_PD = exti_active;  // clear pending mask (by writing 1)

    while (exti_active) {
        unsigned idx = 31 - __builtin_clz(exti_active);
        uint32_t mask = 1 << idx;
        extiChannelRecs[idx].handler->fn(extiChannelRecs[idx].handler);
        exti_active &= ~mask;
    }
}

#define _EXTI_IRQ_HANDLER(name, mask)            \
    void name(void) {                            \
        EXTI_IRQHandler(mask & EXTI_EVENT_MASK); \
    }                                            \
    struct dummy                                 \
    /**/

_EXTI_IRQ_HANDLER(EXTI0_IRQHandler, 0x0001);
_EXTI_IRQ_HANDLER(EXTI1_IRQHandler, 0x0002);
_EXTI_IRQ_HANDLER(EXTI2_IRQHandler, 0x0004);
_EXTI_IRQ_HANDLER(EXTI3_IRQHandler, 0x0008);
_EXTI_IRQ_HANDLER(EXTI4_IRQHandler, 0x0010);
_EXTI_IRQ_HANDLER(EXTI5_9_IRQHandler, 0x03e0);
_EXTI_IRQ_HANDLER(EXTI10_15_IRQHandler, 0xfc00);

#endif
