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
#include "platform/io_impl.h"
#include "drivers/exti.h"

typedef struct {
    extiCallbackRec_t* handler;
} extiChannelRec_t;

extiChannelRec_t extiChannelRecs[16];

// IRQ grouping, same on F40x, F7xx, H7xx and G4xx.
#define EXTI_IRQ_GROUPS 7
//                                      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
static const uint8_t extiGroups[16] = { 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6 };
static uint8_t extiGroupPriority[EXTI_IRQ_GROUPS];
static GPIO_TypeDef* extiGpio[EXTI_IRQ_GROUPS];
static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
    EXTI0_IRQn,
    EXTI1_IRQn,
    EXTI2_IRQn,
    EXTI3_IRQn,
    EXTI4_IRQn,
    EXTI5TO9_IRQn,
    EXTI10TO15_IRQn
};

static uint32_t triggerLookupTable[] = {
    [BETAFLIGHT_EXTI_TRIGGER_RISING]    = GPIO_MODE_IT_EDGE_RISE,
    [BETAFLIGHT_EXTI_TRIGGER_FALLING]   = GPIO_MODE_IT_EDGE_FALL,
    [BETAFLIGHT_EXTI_TRIGGER_BOTH]      = GPIO_MODE_IT_EDGE_FALL_RISE
};

void EXTIInit(void)
{
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
    GPIO_TypeDef *GPIOx = IO_GPIO(io);
    extiGpio[group] = GPIOx;


    extiChannelRec_t *rec = &extiChannelRecs[chIdx];
    rec->handler = cb;

    EXTIDisable(io);

    GPIO_InitTypeDef init = {
        .Pin = IO_Pin(io),
        .Mode = triggerLookupTable[trigger],
        .Speed = IO_CONFIG_GET_SPEED(config),
        .Pull = IO_CONFIG_GET_PULL(config),
    };
    HAL_GPIO_Init(IO_GPIO(io), &init);

    if (extiGroupPriority[group] > irqPriority) {
        extiGroupPriority[group] = irqPriority;
        HAL_NVIC_SetPriority(extiGroupIRQn[group], NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));
        HAL_NVIC_EnableIRQ(extiGroupIRQn[group]);
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
    uint32_t extiLine = IO_EXTI_Line(io);

    if (!extiLine) {
        return;
    }

    IO_GPIO(io)->IEN |= extiLine;
}


void EXTIDisable(IO_t io)
{
    uint32_t extiLine = IO_EXTI_Line(io);

    if (!extiLine) {
        return;
    }

    IO_GPIO(io)->IEN &= ~extiLine;
    IO_GPIO(io)->IC = extiLine;
}

#define EXTI_EVENT_MASK 0xFFFF // first 16 bits only, see also definition of extiChannelRecs.

FAST_IRQ_HANDLER void EXTI_IRQHandler(uint32_t mask)
{
    GPIO_TypeDef* GPIOx = extiGpio[mask];
    uint32_t exti_active = GPIOx->MIS;

    GPIOx->IC = exti_active;  // clear pending mask (by writing 1) 

    while (exti_active) {
        unsigned idx = 31 - __builtin_clz(exti_active);
        uint32_t mask = 1 << idx;
        extiChannelRecs[idx].handler->fn(extiChannelRecs[idx].handler);
        exti_active &= ~mask;
    }
}

#define _EXTI_IRQ_HANDLER(name, mask)            \
    FAST_IRQ_HANDLER void name(void) {           \
        EXTI_IRQHandler(mask & EXTI_EVENT_MASK); \
    }                                            \
    struct dummy                                 \
    /**/

_EXTI_IRQ_HANDLER(EXTI0_IRQHandler, 0);
_EXTI_IRQ_HANDLER(EXTI1_IRQHandler, 1);
_EXTI_IRQ_HANDLER(EXTI2_IRQHandler, 2);
_EXTI_IRQ_HANDLER(EXTI3_IRQHandler, 3);
_EXTI_IRQ_HANDLER(EXTI4_IRQHandler, 4);
_EXTI_IRQ_HANDLER(EXTI5TO9_IRQHandler, 5);
_EXTI_IRQ_HANDLER(EXTI10TO15_IRQHandler, 6);

#endif
