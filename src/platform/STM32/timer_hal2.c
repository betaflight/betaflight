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

/*
 * Timer driver for STM32 HAL2 (Cube 2.0) families.
 *
 * Uses LL register-level functions instead of the HAL2 handle-based API.
 * Fork of timer_hal.c for families that ship with HAL2
 * (e.g. STM32C5 and future HAL2 parts).
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_TIMER

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/nvic.h"

#include "drivers/io.h"
#include "drivers/dma.h"

#include "platform/rcc.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"
#include "platform/timer.h"

#define TIM_N(n) (1 << (n))

#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)

#define TIM_IT_CCx(ch) (TIM_IT_CC1 << ((ch) / 4))

typedef struct timerConfig_s {
    timerOvrHandlerRec_t *updateCallback;
    timerEdgeHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallbackActive;
    uint32_t forcedOverflowTimerValue;
} timerConfig_t;
timerConfig_t timerConfig[USED_TIMER_COUNT + 1];

typedef struct {
    channelType_t type;
} timerChannelInfo_t;
timerChannelInfo_t timerChannelInfo[TIMER_CHANNEL_COUNT];

typedef struct {
    uint8_t priority;
} timerInfo_t;
timerInfo_t timerInfo[USED_TIMER_COUNT + 1];

// Minimal handle for timerFindTimerHandle() compatibility.
// No HAL2 API is called through this — just stores Instance pointer.
typedef struct {
    TIM_HandleTypeDef Handle;
} timerHandle_t;
timerHandle_t timerHandle[USED_TIMER_COUNT + 1];

// --------------------------------------------------------------------------
// Timer index lookup
// --------------------------------------------------------------------------

#define TIMER_INDEX(i) BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

static uint8_t lookupTimerIndex(const timerResource_t *tim)
{
    const TIM_TypeDef *tim_ptr = (const TIM_TypeDef *)tim;
#define _CASE_SHF 10
#define _CASE_(tim, index) case ((unsigned)tim >> _CASE_SHF): return index; break
#define _CASE(i) _CASE_(TIM##i##_BASE, TIMER_INDEX(i))

    switch ((unsigned)tim_ptr >> _CASE_SHF) {
#if USED_TIMERS & TIM_N(1)
        _CASE(1);
#endif
#if USED_TIMERS & TIM_N(2)
        _CASE(2);
#endif
#if USED_TIMERS & TIM_N(3)
        _CASE(3);
#endif
#if USED_TIMERS & TIM_N(4)
        _CASE(4);
#endif
#if USED_TIMERS & TIM_N(5)
        _CASE(5);
#endif
#if USED_TIMERS & TIM_N(6)
        _CASE(6);
#endif
#if USED_TIMERS & TIM_N(7)
        _CASE(7);
#endif
#if USED_TIMERS & TIM_N(8)
        _CASE(8);
#endif
#if USED_TIMERS & TIM_N(12)
        _CASE(12);
#endif
#if USED_TIMERS & TIM_N(15)
        _CASE(15);
#endif
#if USED_TIMERS & TIM_N(16)
        _CASE(16);
#endif
#if USED_TIMERS & TIM_N(17)
        _CASE(17);
#endif
    default:  return ~1;
    }
#undef _CASE
#undef _CASE_
}

TIM_TypeDef * const usedTimers[USED_TIMER_COUNT] = {
#define _DEF(i) TIM##i

#if USED_TIMERS & TIM_N(1)
    _DEF(1),
#endif
#if USED_TIMERS & TIM_N(2)
    _DEF(2),
#endif
#if USED_TIMERS & TIM_N(3)
    _DEF(3),
#endif
#if USED_TIMERS & TIM_N(4)
    _DEF(4),
#endif
#if USED_TIMERS & TIM_N(5)
    _DEF(5),
#endif
#if USED_TIMERS & TIM_N(6)
    _DEF(6),
#endif
#if USED_TIMERS & TIM_N(7)
    _DEF(7),
#endif
#if USED_TIMERS & TIM_N(8)
    _DEF(8),
#endif
#if USED_TIMERS & TIM_N(12)
    _DEF(12),
#endif
#if USED_TIMERS & TIM_N(15)
    _DEF(15),
#endif
#if USED_TIMERS & TIM_N(16)
    _DEF(16),
#endif
#if USED_TIMERS & TIM_N(17)
    _DEF(17),
#endif
#undef _DEF
};

const int8_t timerNumbers[USED_TIMER_COUNT] = {
#define _DEF(i) i

#if USED_TIMERS & TIM_N(1)
    _DEF(1),
#endif
#if USED_TIMERS & TIM_N(2)
    _DEF(2),
#endif
#if USED_TIMERS & TIM_N(3)
    _DEF(3),
#endif
#if USED_TIMERS & TIM_N(4)
    _DEF(4),
#endif
#if USED_TIMERS & TIM_N(5)
    _DEF(5),
#endif
#if USED_TIMERS & TIM_N(6)
    _DEF(6),
#endif
#if USED_TIMERS & TIM_N(7)
    _DEF(7),
#endif
#if USED_TIMERS & TIM_N(8)
    _DEF(8),
#endif
#if USED_TIMERS & TIM_N(12)
    _DEF(12),
#endif
#if USED_TIMERS & TIM_N(15)
    _DEF(15),
#endif
#if USED_TIMERS & TIM_N(16)
    _DEF(16),
#endif
#if USED_TIMERS & TIM_N(17)
    _DEF(17),
#endif
#undef _DEF
};

// --------------------------------------------------------------------------
// Utility functions
// --------------------------------------------------------------------------

int8_t timerGetNumberByIndex(uint8_t index)
{
    if (index < USED_TIMER_COUNT) {
        return timerNumbers[index];
    } else {
        return 0;
    }
}

int8_t timerGetIndexByNumber(uint8_t number)
{
    return TIM_N(number) & USED_TIMERS ? popcount((TIM_N(number) - 1) & USED_TIMERS) : -1;
}

int8_t timerGetTIMNumber(const timerHardware_t *timHw)
{
    uint8_t index = lookupTimerIndex(timHw->tim);
    return timerGetNumberByIndex(index);
}

static inline uint8_t lookupChannelIndex(const uint16_t channel)
{
    return channel >> 2;
}

uint8_t timerLookupChannelIndex(const uint16_t channel)
{
    return lookupChannelIndex(channel);
}

rccPeriphTag_t timerRCC(const timerResource_t *tim)
{
    const TIM_TypeDef *tim_ptr = (const TIM_TypeDef *)tim;
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim_ptr) {
            return timerDefinitions[i].rcc;
        }
    }
    return 0;
}

uint8_t timerInputInterrupt(const timerHardware_t *timHw)
{
    const TIM_TypeDef *tim_ptr = (const TIM_TypeDef *)timHw->tim;
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim_ptr) {
            return timerDefinitions[i].inputIrq;
        }
    }
    return 0;
}

static void timerNVICConfigure(uint8_t irq)
{
    HAL_NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
    HAL_NVIC_EnableIRQ(irq);
}

void* timerFindTimerHandle(timerResource_t *tim)
{
    uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex >= USED_TIMER_COUNT)
        return NULL;

    return &timerHandle[timerIndex].Handle;
}

// --------------------------------------------------------------------------
// Timer base configuration — uses LL instead of HAL_TIM_Base_Init
// --------------------------------------------------------------------------

void timerReconfigureTimeBase(const timerHardware_t *timHw, uint16_t period, uint32_t hz)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;

    LL_TIM_SetPrescaler(tim, (timerClock(timHw) / hz) - 1);
    LL_TIM_SetAutoReload(tim, (period - 1) & 0xffff);
    LL_TIM_SetCounterMode(tim, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(tim, LL_TIM_CLOCKDIVISION_DIV1);
    LL_TIM_SetRepetitionCounter(tim, 0);
    LL_TIM_EnableARRPreload(tim);
    LL_TIM_GenerateEvent_UPDATE(tim);

    // Track Instance for timerFindTimerHandle compatibility
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex < USED_TIMER_COUNT)
        timerHandle[timerIndex].Handle.Instance = tim;
}

// --------------------------------------------------------------------------
// Timer configure / channel init
// --------------------------------------------------------------------------

void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint32_t hz)
{
    uint8_t timerIndex = lookupTimerIndex(timerHardwarePtr->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    timerReconfigureTimeBase(timerHardwarePtr, period, hz);
    LL_TIM_EnableCounter((TIM_TypeDef *)timerHardwarePtr->tim);

    uint8_t irq = timerInputInterrupt(timerHardwarePtr);
    timerNVICConfigure(irq);

    switch (irq) {
    case TIM1_CC_IRQn:
        timerNVICConfigure(TIM1_UP_IRQn);
        break;
    case TIM8_CC_IRQn:
        timerNVICConfigure(TIM8_UP_IRQn);
        break;
    }
}

void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    unsigned channel = timHw - TIMER_HARDWARE;
    if (channel >= TIMER_CHANNEL_COUNT)
        return;

    timerChannelInfo[channel].type = type;
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;
    if (irqPriority < timerInfo[timer].priority) {
        timerReconfigureTimeBase(timHw, 0, 1);
        LL_TIM_EnableCounter((TIM_TypeDef *)timHw->tim);

        HAL_NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));
        HAL_NVIC_EnableIRQ(irq);

        timerInfo[timer].priority = irqPriority;
    }
}

// --------------------------------------------------------------------------
// Callback handler init
// --------------------------------------------------------------------------

void timerChannelEdgeHandlerInit(timerEdgeHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn = fn;
}

void timerChannelOverflowHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn = fn;
    self->next = NULL;
}

// --------------------------------------------------------------------------
// Overflow callback chain management
// --------------------------------------------------------------------------

static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, const timerResource_t *tim)
{
    TIM_TypeDef *timReg = (TIM_TypeDef *)tim;

    timerOvrHandlerRec_t **chain = &cfg->overflowCallbackActive;
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {

        if (cfg->updateCallback) {
            *chain = cfg->updateCallback;
            chain = &cfg->updateCallback->next;
        }

        for (int i = 0; i < CC_CHANNELS_PER_TIMER; i++)
            if (cfg->overflowCallback[i]) {
                *chain = cfg->overflowCallback[i];
                chain = &cfg->overflowCallback[i]->next;
            }
        *chain = NULL;
    }
    if (cfg->overflowCallbackActive)
        SET_BIT(timReg->DIER, TIM_IT_UPDATE);
    else
        CLEAR_BIT(timReg->DIER, TIM_IT_UPDATE);
}

void timerChannelConfigCallbacks(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint8_t channelIndex = lookupChannelIndex(timHw->channel);
    if (edgeCallback == NULL)
        CLEAR_BIT(tim->DIER, TIM_IT_CCx(timHw->channel));

    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;

    if (edgeCallback)
        SET_BIT(tim->DIER, TIM_IT_CCx(timHw->channel));

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

void timerConfigUpdateCallback(const timerHardware_t *timHw, timerOvrHandlerRec_t *updateCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    timerConfig[timerIndex].updateCallback = updateCallback;
    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

void timerChConfigCallbacksDual(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edgeCallbackLo, timerEdgeHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint16_t chLo = timHw->channel & ~TIM_CHANNEL_2;
    uint16_t chHi = timHw->channel | TIM_CHANNEL_2;
    uint8_t channelIndex = lookupChannelIndex(chLo);

    if (edgeCallbackLo == NULL)
        CLEAR_BIT(tim->DIER, TIM_IT_CCx(chLo));
    if (edgeCallbackHi == NULL)
        CLEAR_BIT(tim->DIER, TIM_IT_CCx(chHi));

    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallbackLo;
    timerConfig[timerIndex].edgeCallback[channelIndex + 1] = edgeCallbackHi;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex + 1] = NULL;

    if (edgeCallbackLo) {
        WRITE_REG(tim->SR, ~TIM_IT_CCx(chLo));
        SET_BIT(tim->DIER, TIM_IT_CCx(chLo));
    }
    if (edgeCallbackHi) {
        WRITE_REG(tim->SR, ~TIM_IT_CCx(chHi));
        SET_BIT(tim->DIER, TIM_IT_CCx(chHi));
    }

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

// --------------------------------------------------------------------------
// Channel interrupt configuration
// --------------------------------------------------------------------------

void timerChannelConfigInterruptDualLo(const timerHardware_t *timHw, FunctionalState newState)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    if (newState)
        SET_BIT(tim->DIER, TIM_IT_CCx(timHw->channel & ~TIM_CHANNEL_2));
    else
        CLEAR_BIT(tim->DIER, TIM_IT_CCx(timHw->channel & ~TIM_CHANNEL_2));
}

void timerChannelConfigInterrupt(const timerHardware_t *timHw, FunctionalState newState)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    if (newState)
        SET_BIT(tim->DIER, TIM_IT_CCx(timHw->channel));
    else
        CLEAR_BIT(tim->DIER, TIM_IT_CCx(timHw->channel));
}

void timerChannelClearFlag(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    WRITE_REG(tim->SR, ~TIM_IT_CCx(timHw->channel));
}

// --------------------------------------------------------------------------
// GPIO configuration
// --------------------------------------------------------------------------

void timerChannelConfigGPIO(const timerHardware_t* timHw, ioConfig_t mode)
{
    IOInit(IOGetByTag(timHw->tag), OWNER_TIMER, 0);
    IOConfigGPIO(IOGetByTag(timHw->tag), mode);
}

// --------------------------------------------------------------------------
// Input capture filter lookup
// --------------------------------------------------------------------------

static unsigned getFilter(unsigned ticks)
{
    static const unsigned ftab[16] = {
        1*1,
        1*2, 1*4, 1*8,
        2*6, 2*8,
        4*6, 4*8,
        8*6, 8*8,
        16*5, 16*6, 16*8,
        32*5, 32*6, 32*8
    };
    for (unsigned i = 1; i < ARRAYLEN(ftab); i++)
        if (ftab[i] > ticks)
            return i - 1;
    return 0x0f;
}

// --------------------------------------------------------------------------
// Input capture configuration — direct register access
// --------------------------------------------------------------------------

void timerChannelConfigInput(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;

    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    uint8_t chIdx = timHw->channel >> 2;
    unsigned filter = getFilter(inputFilterTicks);

    CLEAR_BIT(tim->CCER, TIM_CCER_CC1E << (chIdx * 4));

    __IO uint32_t *ccmr = (chIdx < 2) ? &tim->CCMR1 : &tim->CCMR2;
    uint32_t shift = (chIdx & 1) * 8;
    uint32_t mask = (TIM_CCMR1_CC1S | TIM_CCMR1_IC1PSC | TIM_CCMR1_IC1F) << shift;
    uint32_t value = (TIM_CCMR1_CC1S_0 | (filter << 4)) << shift;
    MODIFY_REG(*ccmr, mask, value);

    uint32_t polMask = (TIM_CCER_CC1P | TIM_CCER_CC1NP) << (chIdx * 4);
    uint32_t polVal = polarityRising ? 0 : (TIM_CCER_CC1P << (chIdx * 4));
    MODIFY_REG(tim->CCER, polMask, polVal);
}

void timerChannelConfigInputDual(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;

    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    bool directRising = (timHw->channel & TIM_CHANNEL_2) ? !polarityRising : polarityRising;
    unsigned filter = getFilter(inputFilterTicks);

    // Direct channel
    {
        uint8_t chIdx = timHw->channel >> 2;
        CLEAR_BIT(tim->CCER, TIM_CCER_CC1E << (chIdx * 4));

        __IO uint32_t *ccmr = (chIdx < 2) ? &tim->CCMR1 : &tim->CCMR2;
        uint32_t shift = (chIdx & 1) * 8;
        uint32_t mask = (TIM_CCMR1_CC1S | TIM_CCMR1_IC1PSC | TIM_CCMR1_IC1F) << shift;
        uint32_t value = (TIM_CCMR1_CC1S_0 | (filter << 4)) << shift;
        MODIFY_REG(*ccmr, mask, value);

        uint32_t polMask = (TIM_CCER_CC1P | TIM_CCER_CC1NP) << (chIdx * 4);
        uint32_t polVal = directRising ? 0 : (TIM_CCER_CC1P << (chIdx * 4));
        MODIFY_REG(tim->CCER, polMask, polVal);
    }

    // Indirect channel (paired channel)
    {
        uint16_t otherCh = timHw->channel ^ TIM_CHANNEL_2;
        uint8_t chIdx = otherCh >> 2;
        CLEAR_BIT(tim->CCER, TIM_CCER_CC1E << (chIdx * 4));

        __IO uint32_t *ccmr = (chIdx < 2) ? &tim->CCMR1 : &tim->CCMR2;
        uint32_t shift = (chIdx & 1) * 8;
        uint32_t mask = (TIM_CCMR1_CC1S | TIM_CCMR1_IC1PSC | TIM_CCMR1_IC1F) << shift;
        uint32_t value = (TIM_CCMR1_CC1S_1 | (filter << 4)) << shift;
        MODIFY_REG(*ccmr, mask, value);

        uint32_t polMask = (TIM_CCER_CC1P | TIM_CCER_CC1NP) << (chIdx * 4);
        uint32_t polVal = directRising ? (TIM_CCER_CC1P << (chIdx * 4)) : 0;
        MODIFY_REG(tim->CCER, polMask, polVal);
    }
}

void timerChannelInputPolarity(const timerHardware_t *timHw, bool polarityRising)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    timCCER_t tmpccer = tim->CCER;
    tmpccer &= ~(TIM_CCER_CC1P << timHw->channel);
    tmpccer |= polarityRising ? 0 : (TIM_CCER_CC1P << timHw->channel);
    tim->CCER = tmpccer;
}

// --------------------------------------------------------------------------
// CCR register access
// --------------------------------------------------------------------------

volatile timCCR_t* timerChCCRHi(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&((TIM_TypeDef *)timHw->tim)->CCR1 + (timHw->channel | TIM_CHANNEL_2));
}

volatile timCCR_t* timerChCCRLo(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&((TIM_TypeDef *)timHw->tim)->CCR1 + (timHw->channel & ~TIM_CHANNEL_2));
}

volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&((TIM_TypeDef *)timHw->tim)->CCR1 + timHw->channel);
}

// --------------------------------------------------------------------------
// Output compare configuration — direct register access
// --------------------------------------------------------------------------

void timerChannelConfigOutput(const timerHardware_t* timHw, bool outEnable, bool stateHigh)
{
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;

    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    uint8_t chIdx = timHw->channel >> 2;
    uint32_t ccerShift = chIdx * 4;

    CLEAR_BIT(tim->CCER, TIM_CCER_CC1E << ccerShift);

    // OC mode: INACTIVE_ON_MATCH for output, FROZEN for interrupt-only
    __IO uint32_t *ccmr = (chIdx < 2) ? &tim->CCMR1 : &tim->CCMR2;
    uint32_t shift = (chIdx & 1) * 8;
    uint32_t ocMode = outEnable ? TIM_CCMR1_OC1M_1 : 0U;
    MODIFY_REG(*ccmr, (TIM_CCMR1_OC1M | TIM_CCMR1_CC1S) << shift, ocMode << shift);

    // Pulse = 0
    volatile uint32_t *ccr = (volatile uint32_t *)((volatile char *)&tim->CCR1 + timHw->channel);
    *ccr = 0;

    // Polarity
    if (stateHigh)
        CLEAR_BIT(tim->CCER, TIM_CCER_CC1P << ccerShift);
    else
        SET_BIT(tim->CCER, TIM_CCER_CC1P << ccerShift);

    // Enable channel
    SET_BIT(tim->CCER, TIM_CCER_CC1E << ccerShift);

    if (!outEnable) {
        SET_BIT(tim->DIER, TIM_IT_CCx(timHw->channel));
    }
}

// --------------------------------------------------------------------------
// IRQ handlers — register-level, no HAL dependency
// --------------------------------------------------------------------------

static void timCCxHandler(timerResource_t *tim, timerConfig_t *timerConfig)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)tim;
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim_ptr->SR & tim_ptr->DIER;

    while (tim_status) {
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim_ptr->SR = mask;
        tim_status &= mask;
        switch (bit) {
        case __builtin_clz(TIM_IT_UPDATE): {

            if (timerConfig->forcedOverflowTimerValue != 0) {
                capture = timerConfig->forcedOverflowTimerValue - 1;
                timerConfig->forcedOverflowTimerValue = 0;
            } else {
                capture = tim_ptr->ARR;
            }

            timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
            while (cb) {
                cb->fn(cb, capture);
                cb = cb->next;
            }
            break;
        }
        case __builtin_clz(TIM_IT_CC1):
            if (timerConfig->edgeCallback[0]) {
                timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim_ptr->CCR1);
            }
            break;
        case __builtin_clz(TIM_IT_CC2):
            if (timerConfig->edgeCallback[1]) {
                timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim_ptr->CCR2);
            }
            break;
        case __builtin_clz(TIM_IT_CC3):
            if (timerConfig->edgeCallback[2]) {
                timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim_ptr->CCR3);
            }
            break;
        case __builtin_clz(TIM_IT_CC4):
            if (timerConfig->edgeCallback[3]) {
                timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim_ptr->CCR4);
            }
            break;
        }
    }
}

static inline void timUpdateHandler(timerResource_t *tim, timerConfig_t *timerConfig)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)tim;
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim_ptr->SR & tim_ptr->DIER;
    while (tim_status) {
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim_ptr->SR = mask;
        tim_status &= mask;
        switch (bit) {
        case __builtin_clz(TIM_IT_UPDATE): {

            if (timerConfig->forcedOverflowTimerValue != 0) {
                capture = timerConfig->forcedOverflowTimerValue - 1;
                timerConfig->forcedOverflowTimerValue = 0;
            } else {
                capture = tim_ptr->ARR;
            }

            timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
            while (cb) {
                cb->fn(cb, capture);
                cb = cb->next;
            }
            break;
        }
        }
    }
}

// --------------------------------------------------------------------------
// IRQ handler instances
// --------------------------------------------------------------------------

#define _TIM_IRQ_HANDLER2(name, i, j)                                   \
    void name(void)                                                     \
    {                                                                   \
        timCCxHandler((timerResource_t *)TIM ## i, &timerConfig[TIMER_INDEX(i)]);  \
        timCCxHandler((timerResource_t *)TIM ## j, &timerConfig[TIMER_INDEX(j)]);  \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        timCCxHandler((timerResource_t *)TIM ## i, &timerConfig[TIMER_INDEX(i)]);  \
    } struct dummy

#define _TIM_IRQ_HANDLER_UPDATE_ONLY(name, i)                           \
    void name(void)                                                     \
    {                                                                   \
        timUpdateHandler((timerResource_t *)TIM ## i, &timerConfig[TIMER_INDEX(i)]); \
    } struct dummy

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
_TIM_IRQ_HANDLER(TIM1_UP_IRQHandler, 1);
#endif

#if USED_TIMERS & TIM_N(2)
_TIM_IRQ_HANDLER(TIM2_IRQHandler, 2);
#endif
#if USED_TIMERS & TIM_N(3)
_TIM_IRQ_HANDLER(TIM3_IRQHandler, 3);
#endif
#if USED_TIMERS & TIM_N(4)
_TIM_IRQ_HANDLER(TIM4_IRQHandler, 4);
#endif
#if USED_TIMERS & TIM_N(5)
_TIM_IRQ_HANDLER(TIM5_IRQHandler, 5);
#endif

#if USED_TIMERS & TIM_N(6)
_TIM_IRQ_HANDLER_UPDATE_ONLY(TIM6_IRQHandler, 6);
#endif
#if USED_TIMERS & TIM_N(7)
#  if !(defined(USE_VCP) && defined(STM32C5))
_TIM_IRQ_HANDLER_UPDATE_ONLY(TIM7_IRQHandler, 7);
#  endif
#endif

#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8);
_TIM_IRQ_HANDLER(TIM8_UP_IRQHandler, 8);
#endif

#if USED_TIMERS & TIM_N(12)
_TIM_IRQ_HANDLER(TIM12_IRQHandler, 12);
#endif
#if USED_TIMERS & TIM_N(15)
_TIM_IRQ_HANDLER(TIM15_IRQHandler, 15);
#endif
#if USED_TIMERS & TIM_N(16)
_TIM_IRQ_HANDLER(TIM16_IRQHandler, 16);
#endif
#if USED_TIMERS & TIM_N(17)
_TIM_IRQ_HANDLER(TIM17_IRQHandler, 17);
#endif

// --------------------------------------------------------------------------
// Initialization
// --------------------------------------------------------------------------

void timerInit(void)
{
    memset(timerConfig, 0, sizeof(timerConfig));

    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        RCC_ClockCmd(timerRCC(TIMER_HARDWARE[i].tim), ENABLE);
    }

    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        timerChannelInfo[i].type = TYPE_FREE;
    }

    for (unsigned i = 0; i < USED_TIMER_COUNT; i++) {
        timerInfo[i].priority = ~0;
    }
}

// --------------------------------------------------------------------------
// Enable / disable / start
// --------------------------------------------------------------------------

void timerStart(const timerHardware_t *timHw)
{
    LL_TIM_EnableCounter((TIM_TypeDef *)timHw->tim);
}

void timerEnable(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    LL_TIM_EnableCounter(tim);
    LL_TIM_GenerateEvent_UPDATE(tim);
}

void timerDisable(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    LL_TIM_DisableIT_UPDATE(tim);
    LL_TIM_DisableCounter(tim);
}

void timerEnableInterrupt(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    LL_TIM_ClearFlag_UPDATE(tim);
    LL_TIM_EnableIT_UPDATE(tim);
}

void timerForceOverflow(timerResource_t *tim)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)tim;
    uint8_t timerIndex = lookupTimerIndex(tim);

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        timerConfig[timerIndex].forcedOverflowTimerValue = tim_ptr->CNT + 1;
        tim_ptr->EGR |= TIM_EGR_UG;
    }
}

// --------------------------------------------------------------------------
// DMA helpers
// --------------------------------------------------------------------------

uint16_t timerDmaIndex(uint8_t channel)
{
    return (channel >> 2) + 1;
}

uint16_t timerDmaSource(uint8_t channel)
{
    switch (channel) {
    case TIM_CHANNEL_1: return TIM_DMA_CC1;
    case TIM_CHANNEL_2: return TIM_DMA_CC2;
    case TIM_CHANNEL_3: return TIM_DMA_CC3;
    case TIM_CHANNEL_4: return TIM_DMA_CC4;
    }
    return 0;
}

// --------------------------------------------------------------------------
// Prescaler / period calculations
// --------------------------------------------------------------------------

uint16_t timerGetPrescalerByDesiredMhz(timerResource_t *tim, uint16_t mhz)
{
    if (mhz == 0) {
        return 0;
    }
    return timerGetPrescalerByDesiredHertz(tim, MHZ_TO_HZ(mhz));
}

uint16_t timerGetPeriodByPrescaler(timerResource_t *tim, uint16_t prescaler, uint32_t hz)
{
    if (hz == 0) {
        return 0;
    }
    return (uint16_t)((timerClockFromInstance(tim) / (prescaler + 1)) / hz);
}

uint16_t timerGetPrescalerByDesiredHertz(timerResource_t *tim, uint32_t hz)
{
    if (hz == 0) {
        return 0;
    }
    if (hz > timerClockFromInstance(tim)) {
        return 0;
    }
    return (uint16_t)((timerClockFromInstance(tim) + hz / 2) / hz) - 1;
}

// --------------------------------------------------------------------------
// Timer state access
// --------------------------------------------------------------------------

void timerReset(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    LL_TIM_DeInit(tim);
}

void timerSetPeriod(const timerHardware_t *timHw, uint32_t period)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    tim->ARR = period;
}

uint32_t timerGetPeriod(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    return tim->ARR;
}

void timerSetCounter(const timerHardware_t *timHw, uint32_t counter)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    tim->CNT = counter;
}

uint32_t timerGetPrescaler(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    return tim->PSC;
}

#endif // USE_TIMER
