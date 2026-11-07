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

// The pending/enable state (IEN/MIS/IC) lives in EACH GPIO controller, while
// the SYSCFG EXTICR port mux selects a port per LINE — so two pins in the
// same IRQ group may sit on different ports. Track every port used per
// group; the IRQ handler must service all of them or the other port's MIS
// never clears and the shared NVIC line storms.
#define EXTI_PORTS_PER_GROUP 6 // GPIOA..E plus headroom
static GPIO_TypeDef* extiGroupPorts[EXTI_IRQ_GROUPS][EXTI_PORTS_PER_GROUP];
static uint8_t extiGroupPortCount[EXTI_IRQ_GROUPS];
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
    memset(extiGroupPorts, 0, sizeof(extiGroupPorts));
    memset(extiGroupPortCount, 0, sizeof(extiGroupPortCount));
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

    // Register this line's port with its IRQ group (deduplicated). Every
    // port in the group gets serviced by the group's IRQ handler.
    uint8_t portCount = extiGroupPortCount[group];
    bool known = false;
    for (uint8_t i = 0; i < portCount; i++) {
        if (extiGroupPorts[group][i] == GPIOx) {
            known = true;
            break;
        }
    }
    if (!known && portCount < EXTI_PORTS_PER_GROUP) {
        extiGroupPorts[group][portCount] = GPIOx;
        extiGroupPortCount[group] = portCount + 1;
    }

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

    /* Clear any stale pending bit BEFORE enabling — otherwise the enable
     * itself would fire a spurious callback for an edge that arrived while
     * the line was disabled (X32 exti_x32.c does the same). */
    IO_GPIO(io)->IC = extiLine;
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

// Upstream-shaped: mask is the IRQ group's LINE BITMASK, passed by the
// vector wrapper with the same literals as the STM32 exti.c wrappers
// (0x0001..0x0010, 0x03E0, 0xFC00). Two UM324 deviations from the STM32
// core, both forced by the hardware model:
//   1. Pending/enable state (IEN/MIS/IC) lives per GPIO controller, so the
//      handler drains every port registered for the group instead of one
//      centralized EXTI block. The group index for the port list is the
//      mask's top line (lines 0-4 -> groups 0-4, 9 -> 5, 15 -> 6).
//   2. Each port's MIS is masked down to the group's lines: MIS shows every
//      enabled line of that PORT, so without the mask a port with EXTI pins
//      in two groups would get cross-dispatched from the wrong IRQ.
FAST_IRQ_HANDLER void EXTI_IRQHandler(uint32_t mask)
{
    const unsigned top = 31 - __builtin_clz(mask);
    const uint8_t group = (top <= 4) ? top : ((top <= 9) ? 5 : 6);
    const uint8_t portCount = extiGroupPortCount[group];

    for (uint8_t p = 0; p < portCount; p++) {
        GPIO_TypeDef *GPIOx = extiGroupPorts[group][p];
        uint32_t exti_active = GPIOx->MIS & mask;

        GPIOx->IC = exti_active;  // clear pending mask (by writing 1)

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
}

#define _EXTI_IRQ_HANDLER(name, mask)            \
    FAST_IRQ_HANDLER void name(void) {           \
        EXTI_IRQHandler(mask & EXTI_EVENT_MASK); \
    }                                            \
    struct dummy                                 \
    /**/

_EXTI_IRQ_HANDLER(EXTI0_IRQHandler, 0x0001);
_EXTI_IRQ_HANDLER(EXTI1_IRQHandler, 0x0002);
_EXTI_IRQ_HANDLER(EXTI2_IRQHandler, 0x0004);
_EXTI_IRQ_HANDLER(EXTI3_IRQHandler, 0x0008);
_EXTI_IRQ_HANDLER(EXTI4_IRQHandler, 0x0010);
_EXTI_IRQ_HANDLER(EXTI5TO9_IRQHandler, 0x03E0);
_EXTI_IRQ_HANDLER(EXTI10TO15_IRQHandler, 0xFC00);

#endif
