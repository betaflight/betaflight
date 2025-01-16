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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_EXTI

#include "drivers/nvic.h"
#include "drivers/io_impl.h"
#include "drivers/exti.h"

typedef struct {
    extiCallbackRec_t* handler;
} extiChannelRec_t;

extiChannelRec_t extiChannelRecs[16];

#define EXTI_IRQ_GROUPS 7
//                                      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
static const uint8_t extiGroups[16] = { 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6 };
static uint8_t extiGroupPriority[EXTI_IRQ_GROUPS];

static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
    EXINT0_IRQn,
    EXINT1_IRQn,
    EXINT2_IRQn,
    EXINT3_IRQn,
    EXINT4_IRQn,
    EXINT9_5_IRQn,
    EXINT15_10_IRQn
};

static uint32_t triggerLookupTable[] = {
    [BETAFLIGHT_EXTI_TRIGGER_RISING]    = EXINT_TRIGGER_RISING_EDGE,
    [BETAFLIGHT_EXTI_TRIGGER_FALLING]   = EXINT_TRIGGER_FALLING_EDGE,
    [BETAFLIGHT_EXTI_TRIGGER_BOTH]      = EXINT_TRIGGER_BOTH_EDGE
};

#define EXTI_REG_IMR (EXINT->inten)
#define EXTI_REG_PR  (EXINT->intsts)

void EXTIInit(void)
{
    crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);

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

    scfg_exint_line_config(IO_GPIO_PortSource(io), IO_GPIO_PinSource(io));

    uint32_t extiLine = IO_EXTI_Line(io);

    exint_flag_clear(extiLine);

    exint_init_type exint_init_struct;
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
    exint_init_struct.line_select = extiLine;
    exint_init_struct.line_polarity = triggerLookupTable[trigger];
    exint_init(&exint_init_struct);

    if (extiGroupPriority[group] > irqPriority) {
        extiGroupPriority[group] = irqPriority;

        nvic_priority_group_config(NVIC_PRIORITY_GROUPING);
        nvic_irq_enable(extiGroupIRQn[group],
            NVIC_PRIORITY_BASE(irqPriority),
            NVIC_PRIORITY_SUB(irqPriority));

    }
}

void EXTIRelease(IO_t io)
{
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
    uint32_t extiLine = IO_EXTI_Line(io);

    if (!extiLine) {
        return;
    }

    EXTI_REG_IMR |= extiLine;
}

void EXTIDisable(IO_t io)
{
    uint32_t extiLine = IO_EXTI_Line(io);

    if (!extiLine) {
        return;
    }

    EXTI_REG_IMR &= ~extiLine;
    EXTI_REG_PR = extiLine;
}

#define EXTI_EVENT_MASK 0xFFFF

static void EXTI_IRQHandler(uint32_t mask)
{
    uint32_t exti_active = (EXTI_REG_IMR & EXTI_REG_PR) & mask;

    EXTI_REG_PR = exti_active;

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

_EXTI_IRQ_HANDLER(EXINT0_IRQHandler, 0x0001);
_EXTI_IRQ_HANDLER(EXINT1_IRQHandler, 0x0002);
_EXTI_IRQ_HANDLER(EXINT2_IRQHandler, 0x0004);
_EXTI_IRQ_HANDLER(EXINT3_IRQHandler, 0x0008);
_EXTI_IRQ_HANDLER(EXINT4_IRQHandler, 0x0010);
_EXTI_IRQ_HANDLER(EXINT9_5_IRQHandler, 0x03e0);
_EXTI_IRQ_HANDLER(EXINT15_10_IRQHandler, 0xfc00);

#endif
