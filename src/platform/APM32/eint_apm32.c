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

#include "common/irq_all.h"

typedef struct {
    extiCallbackRec_t* handler;
} extiChannelRec_t;

extiChannelRec_t extiChannelRecs[16];

// IRQ grouping
#define EXTI_IRQ_GROUPS 7
//                                      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
static const uint8_t extiGroups[16] = { 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6 };
static uint8_t extiGroupPriority[EXTI_IRQ_GROUPS];

#if defined(APM32F4)
static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
    EINT0_IRQn,
    EINT1_IRQn,
    EINT2_IRQn,
    EINT3_IRQn,
    EINT4_IRQn,
    EINT9_5_IRQn,
    EINT15_10_IRQn
};
#else
# warning "Unknown CPU"
#endif

static uint32_t triggerLookupTable[] = {
#if defined(APM32F4)
    [BETAFLIGHT_EXTI_TRIGGER_RISING]    = GPIO_MODE_IT_RISING,
    [BETAFLIGHT_EXTI_TRIGGER_FALLING]   = GPIO_MODE_IT_FALLING,
    [BETAFLIGHT_EXTI_TRIGGER_BOTH]      = GPIO_MODE_IT_RISING_FALLING
#else
# warning "Unknown CPU"
#endif
};

// Absorb the difference in IMR and PR assignments to registers

#define EXTI_REG_IMR (EINT->IMASK)
#define EXTI_REG_PR  (EINT->IPEND)

void EXTIInit(void)
{
#if defined(APM32F4)
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    __DAL_RCM_SYSCFG_CLK_ENABLE();
#endif
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

    GPIO_InitTypeDef init = {
        .Pin = IO_Pin(io),
        .Mode = GPIO_MODE_INPUT | IO_CONFIG_GET_MODE(config) | triggerLookupTable[trigger],
        .Speed = IO_CONFIG_GET_SPEED(config),
        .Pull = IO_CONFIG_GET_PULL(config),
    };
    DAL_GPIO_Init(IO_GPIO(io), &init);

    if (extiGroupPriority[group] > irqPriority) {
        extiGroupPriority[group] = irqPriority;
        DAL_NVIC_SetPriority(extiGroupIRQn[group], NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));
        DAL_NVIC_EnableIRQ(extiGroupIRQn[group]);
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
#if defined(APM32F4)
    uint32_t extiLine = IO_EXTI_Line(io);

    if (!extiLine) {
        return;
    }

    EXTI_REG_IMR |= extiLine;
#else
# error "Unknown CPU"
#endif
}

void EXTIDisable(IO_t io)
{
#if defined(APM32F4)
    uint32_t extiLine = IO_EXTI_Line(io);

    if (!extiLine) {
        return;
    }

    EXTI_REG_IMR &= ~extiLine;
    EXTI_REG_PR = extiLine;
#else
# error "Unknown CPU"
#endif
}

#define EXTI_EVENT_MASK 0xFFFF // first 16 bits only, see also definition of extiChannelRecs.

static void EXTI_IRQHandler(uint32_t mask)
{
    uint32_t exti_active = (EXTI_REG_IMR & EXTI_REG_PR) & mask;

    EXTI_REG_PR = exti_active;  // clear pending mask (by writing 1)

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
// TODO - no version for GCC ? 
_EXTI_IRQ_HANDLER(EINT0_IRQHandler, 0x0001);
_EXTI_IRQ_HANDLER(EINT1_IRQHandler, 0x0002);
#if defined(APM32F4)
_EXTI_IRQ_HANDLER(EINT2_IRQHandler, 0x0004);
#else
# warning "Unknown CPU"
#endif
_EXTI_IRQ_HANDLER(EINT3_IRQHandler, 0x0008);
_EXTI_IRQ_HANDLER(EINT4_IRQHandler, 0x0010);
_EXTI_IRQ_HANDLER(EINT9_5_IRQHandler, 0x03e0);
_EXTI_IRQ_HANDLER(EINT15_10_IRQHandler, 0xfc00);

#endif
