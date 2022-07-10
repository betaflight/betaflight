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

#if !defined(SIMULATOR_BUILD)

#include "drivers/nvic.h"
#include "io_impl.h"
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

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
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
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    [BETAFLIGHT_EXTI_TRIGGER_RISING] = GPIO_MODE_IT_RISING,
    [BETAFLIGHT_EXTI_TRIGGER_FALLING] = GPIO_MODE_IT_FALLING,
    [BETAFLIGHT_EXTI_TRIGGER_BOTH] = GPIO_MODE_IT_RISING_FALLING
#elif defined(STM32F4)
    [BETAFLIGHT_EXTI_TRIGGER_RISING] = EXTI_Trigger_Rising,
    [BETAFLIGHT_EXTI_TRIGGER_FALLING] = EXTI_Trigger_Falling,
    [BETAFLIGHT_EXTI_TRIGGER_BOTH] = EXTI_Trigger_Rising_Falling
#else
# warning "Unknown CPU"
#endif
};

// Absorb the difference in IMR and PR assignments to registers

#if defined(STM32H7)
#define EXTI_REG_IMR (EXTI_D1->IMR1)
#define EXTI_REG_PR  (EXTI_D1->PR1)
#elif defined(STM32G4)
#define EXTI_REG_IMR (EXTI->IMR1)
#define EXTI_REG_PR  (EXTI->PR1)
#else
#define EXTI_REG_IMR (EXTI->IMR)
#define EXTI_REG_PR  (EXTI->PR)
#endif

void EXTIInit(void)
{
#if defined(STM32F4)
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#ifdef REMAP_TIM16_DMA
    SYSCFG_DMAChannelRemapConfig(SYSCFG_DMARemap_TIM16, ENABLE);
#endif
#ifdef REMAP_TIM17_DMA
    SYSCFG_DMAChannelRemapConfig(SYSCFG_DMARemap_TIM17, ENABLE);
#endif
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

#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    GPIO_InitTypeDef init = {
        .Pin = IO_Pin(io),
        .Mode = GPIO_MODE_INPUT | IO_CONFIG_GET_MODE(config) | triggerLookupTable[trigger],
        .Speed = IO_CONFIG_GET_SPEED(config),
        .Pull = IO_CONFIG_GET_PULL(config),
    };
    HAL_GPIO_Init(IO_GPIO(io), &init);

    if (extiGroupPriority[group] > irqPriority) {
        extiGroupPriority[group] = irqPriority;
        HAL_NVIC_SetPriority(extiGroupIRQn[group], NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));
        HAL_NVIC_EnableIRQ(extiGroupIRQn[group]);
    }
#else
    IOConfigGPIO(io, config);

#if defined(STM32F4)
    SYSCFG_EXTILineConfig(IO_EXTI_PortSourceGPIO(io), IO_EXTI_PinSource(io));
#else
# warning "Unknown CPU"
#endif
    uint32_t extiLine = IO_EXTI_Line(io);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = extiLine;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = triggerLookupTable[trigger];
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    if (extiGroupPriority[group] > irqPriority) {
        extiGroupPriority[group] = irqPriority;

        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = extiGroupIRQn[group];
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
#endif
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
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
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
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
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

void EXTI_IRQHandler(uint32_t mask)
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


_EXTI_IRQ_HANDLER(EXTI0_IRQHandler, 0x0001);
_EXTI_IRQ_HANDLER(EXTI1_IRQHandler, 0x0002);
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
_EXTI_IRQ_HANDLER(EXTI2_IRQHandler, 0x0004);
#else
# warning "Unknown CPU"
#endif
_EXTI_IRQ_HANDLER(EXTI3_IRQHandler, 0x0008);
_EXTI_IRQ_HANDLER(EXTI4_IRQHandler, 0x0010);
_EXTI_IRQ_HANDLER(EXTI9_5_IRQHandler, 0x03e0);
_EXTI_IRQ_HANDLER(EXTI15_10_IRQHandler, 0xfc00);

#endif

