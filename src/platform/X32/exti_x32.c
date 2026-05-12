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
#include "platform/rcc.h"

typedef struct {
    extiCallbackRec_t *handler;
} extiChannelRec_t;

extiChannelRec_t extiChannelRecs[16];

// IRQ grouping
#define EXTI_IRQ_GROUPS 7
//                                      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
static const uint8_t extiGroups[16] = { 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6 };
static uint8_t extiGroupPriority[EXTI_IRQ_GROUPS];

#if defined(X32M7)
static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
    EXTI0_IRQn,
    EXTI1_IRQn,
    EXTI2_IRQn,
    EXTI3_IRQn,
    EXTI4_IRQn,
    EXTI9_5_IRQn,
    EXTI15_10_IRQn
};
#else
# warning "Unknown CPU"
#endif

static uint32_t triggerLookupTable[] = {
#if defined(X32M7)
    [BETAFLIGHT_EXTI_TRIGGER_RISING]    = (uint32_t)EXTI_Trigger_Rising,
    [BETAFLIGHT_EXTI_TRIGGER_FALLING]   = (uint32_t)EXTI_Trigger_Falling,
    [BETAFLIGHT_EXTI_TRIGGER_BOTH]      = (uint32_t)EXTI_Trigger_Rising_Falling
#else
# warning "Unknown CPU"
#endif
};


#if defined(X32M7)
#define EXTI_REG_IMASK (EXTI->M7IMASK[0])
#define EXTI_REG_PEND  (EXTI->M7PEND[0])
#endif

static uint8_t extiPinSource(IO_t io, int chIdx)
{
    return (uint8_t)(IO_GPIO_PortSource(io) + (11 * chIdx));
}

void EXTIInit(void)
{
#if defined(X32M7)
    /* Enable AFIO clock to config gpio remap to exto line */
    RCC_ClockCmd(RCC_AHB5_2(AFIO), ENABLE);
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

    IOConfigGPIO(io, config);

#if defined(X32M7)
    /* Clear AFIO_EXTI_CFG bits */
    AFIO->EXTI_CFG[chIdx / 4] &= ~(0xffU << (8 * (chIdx % 4)));
    /* Config the GPIO pin used as EXTI Line */
    GPIO_ConfigEXTILine((uint8_t)chIdx, extiPinSource(io, chIdx));

    /* Clear EXTI line interrupt pend bit */
    EXTI_ClrITPendBit((uint32_t)chIdx);

    EXTI_InitType EXTI_InitStructure;
    EXTI_InitStruct(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line    = (uint32_t)chIdx;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = triggerLookupTable[trigger];
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    if (extiGroupPriority[group] > irqPriority) {
        extiGroupPriority[group] = irqPriority;

        NVIC_InitType NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel                   = (uint8_t)extiGroupIRQn[group];
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = NVIC_PRIORITY_SUB(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
#endif
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
#if defined(X32M7)
    uint32_t extiLine = IO_EXTI_Line(io);

    if (!extiLine) {
        return;
    }

    EXTI_REG_PEND   = extiLine;
    EXTI_REG_IMASK |= extiLine;
#else
# error "Unknown MCU"
#endif
}

void EXTIDisable(IO_t io)
{
#if defined(X32M7)
    uint32_t extiLine = IO_EXTI_Line(io);

    if (!extiLine) {
        return;
    }

    EXTI_REG_IMASK &= ~extiLine;
    EXTI_REG_PEND   = extiLine;
#else
# error "Unknown CPU"
#endif
}

#define EXTI_EVENT_MASK 0xFFFF // first 16 bits only, see also definition of extiChannelRecs.

static void EXTI_IRQHandler(uint32_t mask)
{
    uint32_t exti_active = (EXTI_REG_IMASK & EXTI_REG_PEND) & mask;

    /* Clear EXTI line interrupt pend bit */
    EXTI_REG_PEND = exti_active;  // clear pending mask (by writing 1)

    while (exti_active) {
        unsigned idx = 31 - __builtin_clz(exti_active);
        uint32_t activeMask = 1 << idx;

        extiCallbackRec_t *handler = extiChannelRecs[idx].handler;
        if (handler && handler->fn) {
            handler->fn(handler);
        }

        exti_active &= ~activeMask;
    }
}


#define EXTI_IRQ_HANDLER(name, mask)            \
    void name(void)                             \
    {                                           \
        EXTI_IRQHandler((mask) & EXTI_EVENT_MASK); \
    }

EXTI_IRQ_HANDLER(EXTI0_IRQHandler, 0x0001U)
EXTI_IRQ_HANDLER(EXTI1_IRQHandler, 0x0002U)
EXTI_IRQ_HANDLER(EXTI2_IRQHandler, 0x0004U)
EXTI_IRQ_HANDLER(EXTI3_IRQHandler, 0x0008U)
EXTI_IRQ_HANDLER(EXTI4_IRQHandler, 0x0010U)
EXTI_IRQ_HANDLER(EXTI9_5_IRQHandler, 0x03E0U)
EXTI_IRQ_HANDLER(EXTI15_10_IRQHandler, 0xFC00U)

#endif
