/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#ifdef USE_TIMER

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"

#include "platform/rcc.h"
#include "platform/timer.h"

#define CC_CHANNELS_PER_TIMER 4

#define X32_TIMER_TABLE(X) \
    X(X32_TIMER_ATIM1,  ATIM1,  RCC_APB2_1(ATIM1),  ATIM1_CC_IRQn, ATIM1_UP_IRQn, true,  true)  \
    X(X32_TIMER_ATIM2,  ATIM2,  RCC_APB2_1(ATIM2),  ATIM2_CC_IRQn, ATIM2_UP_IRQn, true,  true)  \
    X(X32_TIMER_GTIMA1, GTIMA1, RCC_APB2_1(GTIMA1), GTIMA1_IRQn,   GTIMA1_IRQn,   false, false) \
    X(X32_TIMER_GTIMA2, GTIMA2, RCC_APB2_1(GTIMA2), GTIMA2_IRQn,   GTIMA2_IRQn,   false, false) \
    X(X32_TIMER_GTIMA3, GTIMA3, RCC_APB2_1(GTIMA3), GTIMA3_IRQn,   GTIMA3_IRQn,   false, false) \
    X(X32_TIMER_GTIMA4, GTIMA4, RCC_APB1_1(GTIMA4), GTIMA4_IRQn,   GTIMA4_IRQn,   false, false) \
    X(X32_TIMER_GTIMA5, GTIMA5, RCC_APB1_2(GTIMA5), GTIMA5_IRQn,   GTIMA5_IRQn,   false, false) \
    X(X32_TIMER_GTIMA6, GTIMA6, RCC_APB1_2(GTIMA6), GTIMA6_IRQn,   GTIMA6_IRQn,   false, false) \
    X(X32_TIMER_GTIMA7, GTIMA7, RCC_APB1_2(GTIMA7), GTIMA7_IRQn,   GTIMA7_IRQn,   false, false) \
    X(X32_TIMER_GTIMB1, GTIMB1, RCC_APB1_1(GTIMB1), GTIMB1_IRQn,   GTIMB1_IRQn,   false, false) \
    X(X32_TIMER_GTIMB2, GTIMB2, RCC_APB1_1(GTIMB2), GTIMB2_IRQn,   GTIMB2_IRQn,   false, false) \
    X(X32_TIMER_GTIMB3, GTIMB3, RCC_APB1_1(GTIMB3), GTIMB3_IRQn,   GTIMB3_IRQn,   false, false) \
    X(X32_TIMER_ATIM3,  ATIM3,  RCC_APB5_1(ATIM3),  ATIM3_CC_IRQn, ATIM3_UP_IRQn, true,  true)  \
    X(X32_TIMER_ATIM4,  ATIM4,  RCC_APB5_1(ATIM4),  ATIM4_CC_IRQn, ATIM4_UP_IRQn, true,  true)

#define X32_TIMER_DEF(num, instance, rccTag, ccIrq, upIrq, splitUpdate, isAdvanced) \
    { .TIMx = (void *)(instance), .rcc = (rccTag), .inputIrq = (uint8_t)(ccIrq) },
const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    X32_TIMER_TABLE(X32_TIMER_DEF)
};
#undef X32_TIMER_DEF

const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    { .tim = (timerResource_t *)GTIMA4, .tag = DEFIO_TAG_E(PA1), .channel = TIM_CH_1, .output = TIMER_OUTPUT_NONE, .alternateFunction = GPIO_AF2 },
    { .tim = (timerResource_t *)GTIMA4, .tag = DEFIO_TAG_E(PB7), .channel = TIM_CH_2, .output = TIMER_OUTPUT_NONE, .alternateFunction = GPIO_AF2 },
    { .tim = (timerResource_t *)GTIMA4, .tag = DEFIO_TAG_E(PB8), .channel = TIM_CH_3, .output = TIMER_OUTPUT_NONE, .alternateFunction = GPIO_AF2 },
    { .tim = (timerResource_t *)GTIMA4, .tag = DEFIO_TAG_E(PB9), .channel = TIM_CH_4, .output = TIMER_OUTPUT_NONE, .alternateFunction = GPIO_AF2 },
    { .tim = (timerResource_t *)GTIMA3, .tag = DEFIO_TAG_E(PC6), .channel = TIM_CH_1, .output = TIMER_OUTPUT_NONE, .alternateFunction = GPIO_AF2 },
    { .tim = (timerResource_t *)GTIMA3, .tag = DEFIO_TAG_E(PC7), .channel = TIM_CH_2, .output = TIMER_OUTPUT_NONE, .alternateFunction = GPIO_AF2 },
    { .tim = (timerResource_t *)GTIMA3, .tag = DEFIO_TAG_E(PC8), .channel = TIM_CH_3, .output = TIMER_OUTPUT_NONE, .alternateFunction = GPIO_AF2 },
    { .tim = (timerResource_t *)GTIMA3, .tag = DEFIO_TAG_E(PC9), .channel = TIM_CH_4, .output = TIMER_OUTPUT_NONE, .alternateFunction = GPIO_AF2 },
};

typedef struct X32TimerInfo_s {
    TIM_TypeDef *tim;
    rccPeriphTag_t rcc;
    IRQn_Type inputIrq;
    IRQn_Type updateIrq;
    bool separateUpdateIrq;
    bool advanced;
    int8_t number;
} X32TimerInfo_t;

#define X32_TIMER_INFO(num, instance, rccTag, ccIrq, upIrq, splitUpdate, isAdvanced) \
    { .tim = (instance), .rcc = (rccTag), .inputIrq = (ccIrq), .updateIrq = (upIrq), .separateUpdateIrq = (splitUpdate), .advanced = (isAdvanced), .number = (num) },
static const X32TimerInfo_t timerInstances[HARDWARE_TIMER_DEFINITION_COUNT] = {
    X32_TIMER_TABLE(X32_TIMER_INFO)
};
#undef X32_TIMER_INFO

typedef struct timerConfig_s {
    timerOvrHandlerRec_t *updateCallback;
    timerEdgeHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallbackActive;
    uint32_t forcedOverflowTimerValue;
} timerConfig_t;

typedef struct timerRuntimeInfo_s {
    uint8_t priority;
} timerRuntimeInfo_t;

static timerConfig_t timerConfig[HARDWARE_TIMER_DEFINITION_COUNT];
static timerRuntimeInfo_t timerInfo[HARDWARE_TIMER_DEFINITION_COUNT];

static uint8_t lookupTimerIndex(const timerResource_t *tim)
{
    for (uint8_t i = 0; i < ARRAYLEN(timerInstances); i++) {
        if (timerInstances[i].tim == (TIM_TypeDef *)tim) {
            return i;
        }
    }

    return ARRAYLEN(timerInstances);
}

static uint8_t lookupChannelIndex(const uint16_t channel)
{
    return CC_INDEX_FROM_CHANNEL(channel);
}

uint8_t timerLookupChannelIndex(const uint16_t channel)
{
    return lookupChannelIndex(channel);
}

static uint32_t channelToInterrupt(uint16_t channel)
{
    switch (channel) {
    case TIM_CH_1:
        return TIM_INT_CC1;
    case TIM_CH_2:
        return TIM_INT_CC2;
    case TIM_CH_3:
        return TIM_INT_CC3;
    case TIM_CH_4:
        return TIM_INT_CC4;
    default:
        return 0;
    }
}

static volatile timCCR_t *timerGetCCRPointer(TIM_TypeDef *tim, uint16_t channel)
{
    switch (channel) {
    case TIM_CH_1:
        return &tim->CCDAT1;
    case TIM_CH_2:
        return &tim->CCDAT2;
    case TIM_CH_3:
        return &tim->CCDAT3;
    case TIM_CH_4:
        return &tim->CCDAT4;
    default:
        return NULL;
    }
}

static uint16_t timerReadCapture(TIM_TypeDef *tim, uint16_t channel)
{
    volatile timCCR_t *ccr = timerGetCCRPointer(tim, channel);
    return ccr ? (uint16_t)(*ccr & 0xFFFFU) : 0;
}

static bool isAdvancedTimer(const TIM_TypeDef *tim)
{
    return (tim == ATIM1) || (tim == ATIM2) || (tim == ATIM3) || (tim == ATIM4);
}

static void timerEnableClock(const timerResource_t *tim)
{
    const uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex < ARRAYLEN(timerInstances)) {
        RCC_ClockCmd(timerInstances[timerIndex].rcc, ENABLE);
    }
}

static void timerNVICConfigure(IRQn_Type irq, uint8_t irqPriority)
{
    if ((int)irq < 0) {
        return;
    }

    NVIC_InitType nvicInit;
    nvicInit.NVIC_IRQChannel = (uint8_t)irq;
    nvicInit.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
    nvicInit.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);
}

static void timerConfigureInterrupts(const uint8_t timerIndex, uint8_t irqPriority)
{
    timerNVICConfigure(timerInstances[timerIndex].inputIrq, irqPriority);
    if (timerInstances[timerIndex].separateUpdateIrq) {
        timerNVICConfigure(timerInstances[timerIndex].updateIrq, irqPriority);
    }
}

static uint16_t timerPrescalerFromHz(uint32_t timerClockHz, uint32_t hz)
{
    if (hz == 0) {
        return 0;
    }

    uint32_t prescaler = (timerClockHz / hz);
    prescaler = (prescaler > 0) ? prescaler - 1U : 0U;
    return (uint16_t)((prescaler > 0xFFFFU) ? 0xFFFFU : prescaler);
}

static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, const timerResource_t *tim)
{
    timerOvrHandlerRec_t **chain = &cfg->overflowCallbackActive;

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        if (cfg->updateCallback) {
            *chain = cfg->updateCallback;
            chain = &cfg->updateCallback->next;
        }

        for (unsigned i = 0; i < CC_CHANNELS_PER_TIMER; i++) {
            if (cfg->overflowCallback[i]) {
                *chain = cfg->overflowCallback[i];
                chain = &cfg->overflowCallback[i]->next;
            }
        }

        *chain = NULL;
    }

    TIM_ConfigInt((TIM_TypeDef *)tim, TIM_INT_UPDATE, cfg->overflowCallbackActive ? ENABLE : DISABLE);
}

static void timUpdateHandler(TIM_TypeDef *tim, timerConfig_t *cfg)
{
    if ((tim->STS & tim->DINTEN & TIM_INT_UPDATE) == 0) {
        return;
    }

    TIM_ClrIntPendingBit(tim, TIM_INT_UPDATE);

    uint16_t capture;
    if (cfg->forcedOverflowTimerValue != 0) {
        capture = (uint16_t)(cfg->forcedOverflowTimerValue - 1U);
        cfg->forcedOverflowTimerValue = 0;
    } else {
        capture = (uint16_t)(tim->AR & 0xFFFFU);
    }

    timerOvrHandlerRec_t *cb = cfg->overflowCallbackActive;
    while (cb) {
        cb->fn(cb, capture);
        cb = cb->next;
    }
}

static void timCCHandler(TIM_TypeDef *tim, timerConfig_t *cfg)
{
    const uint32_t status = tim->STS & tim->DINTEN;

    if ((status & TIM_INT_CC1) && cfg->edgeCallback[0] && cfg->edgeCallback[0]->fn) {
        TIM_ClrIntPendingBit(tim, TIM_INT_CC1);
        cfg->edgeCallback[0]->fn(cfg->edgeCallback[0], timerReadCapture(tim, TIM_CH_1));
    }

    if ((status & TIM_INT_CC2) && cfg->edgeCallback[1] && cfg->edgeCallback[1]->fn) {
        TIM_ClrIntPendingBit(tim, TIM_INT_CC2);
        cfg->edgeCallback[1]->fn(cfg->edgeCallback[1], timerReadCapture(tim, TIM_CH_2));
    }

    if ((status & TIM_INT_CC3) && cfg->edgeCallback[2] && cfg->edgeCallback[2]->fn) {
        TIM_ClrIntPendingBit(tim, TIM_INT_CC3);
        cfg->edgeCallback[2]->fn(cfg->edgeCallback[2], timerReadCapture(tim, TIM_CH_3));
    }

    if ((status & TIM_INT_CC4) && cfg->edgeCallback[3] && cfg->edgeCallback[3]->fn) {
        TIM_ClrIntPendingBit(tim, TIM_INT_CC4);
        cfg->edgeCallback[3]->fn(cfg->edgeCallback[3], timerReadCapture(tim, TIM_CH_4));
    }
}

static void timCombinedHandler(TIM_TypeDef *tim, timerConfig_t *cfg)
{
    timCCHandler(tim, cfg);
    timUpdateHandler(tim, cfg);
}

#define X32_TIM_CC_HANDLER(handlerName, tim) \
    void handlerName(void); \
    void handlerName(void) \
    { \
        timCCHandler((tim), &timerConfig[lookupTimerIndex((timerResource_t *)(tim))]); \
    }

#define X32_TIM_UP_HANDLER(handlerName, tim) \
    void handlerName(void); \
    void handlerName(void) \
    { \
        timUpdateHandler((tim), &timerConfig[lookupTimerIndex((timerResource_t *)(tim))]); \
    }

#define X32_TIM_GLOBAL_HANDLER(handlerName, tim) \
    void handlerName(void); \
    void handlerName(void) \
    { \
        timCombinedHandler((tim), &timerConfig[lookupTimerIndex((timerResource_t *)(tim))]); \
    }

X32_TIM_CC_HANDLER(ATIM1_CC_IRQHandler, ATIM1)
X32_TIM_UP_HANDLER(ATIM1_UP_IRQHandler, ATIM1)
X32_TIM_CC_HANDLER(ATIM2_CC_IRQHandler, ATIM2)
X32_TIM_UP_HANDLER(ATIM2_UP_IRQHandler, ATIM2)
X32_TIM_GLOBAL_HANDLER(GTIMA1_IRQHandler, GTIMA1)
X32_TIM_GLOBAL_HANDLER(GTIMA2_IRQHandler, GTIMA2)
X32_TIM_GLOBAL_HANDLER(GTIMA3_IRQHandler, GTIMA3)
X32_TIM_GLOBAL_HANDLER(GTIMA4_IRQHandler, GTIMA4)
X32_TIM_GLOBAL_HANDLER(GTIMA5_IRQHandler, GTIMA5)
X32_TIM_GLOBAL_HANDLER(GTIMA6_IRQHandler, GTIMA6)
X32_TIM_GLOBAL_HANDLER(GTIMA7_IRQHandler, GTIMA7)
X32_TIM_GLOBAL_HANDLER(GTIMB1_IRQHandler, GTIMB1)
X32_TIM_GLOBAL_HANDLER(GTIMB2_IRQHandler, GTIMB2)
X32_TIM_GLOBAL_HANDLER(GTIMB3_IRQHandler, GTIMB3)
X32_TIM_CC_HANDLER(ATIM3_CC_IRQHandler, ATIM3)
X32_TIM_UP_HANDLER(ATIM3_UP_IRQHandler, ATIM3)
X32_TIM_CC_HANDLER(ATIM4_CC_IRQHandler, ATIM4)
X32_TIM_UP_HANDLER(ATIM4_UP_IRQHandler, ATIM4)

#undef X32_TIM_CC_HANDLER
#undef X32_TIM_UP_HANDLER
#undef X32_TIM_GLOBAL_HANDLER

void timerInitTarget(void)
{
}

int8_t timerGetNumberByIndex(uint8_t index)
{
    return (index < ARRAYLEN(timerInstances)) ? timerInstances[index].number : -1;
}

int8_t timerGetIndexByNumber(uint8_t number)
{
    for (uint8_t i = 0; i < ARRAYLEN(timerInstances); i++) {
        if (timerInstances[i].number == number) {
            return i;
        }
    }

    return -1;
}

int8_t timerGetTIMNumber(const timerHardware_t *timHw)
{
    const uint8_t index = lookupTimerIndex((timerResource_t *)timHw->tim);
    return timerGetNumberByIndex(index);
}

rccPeriphTag_t timerRCC(const timerResource_t *tim)
{
    const uint8_t timerIndex = lookupTimerIndex((timerResource_t *)tim);
    return (timerIndex < ARRAYLEN(timerInstances)) ? timerInstances[timerIndex].rcc : 0;
}

uint8_t timerInputInterrupt(const timerHardware_t *timHw)
{
    const uint8_t timerIndex = lookupTimerIndex((timerResource_t *)timHw->tim);
    return (timerIndex < ARRAYLEN(timerInstances)) ? (uint8_t)timerInstances[timerIndex].inputIrq : 0;
}

uint32_t timerClockFromInstance(const timerResource_t *tim)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);

    const TIM_TypeDef *timer = (const TIM_TypeDef *)tim;
    if ((timer == ATIM1) || (timer == ATIM2) || (timer == GTIMA1) || (timer == GTIMA2) || (timer == GTIMA3)) {
        return clocks.APB2ClkFreq;
    }

    if ((timer == ATIM3) || (timer == ATIM4)) {
        return clocks.APB5ClkFreq;
    }

    return clocks.APB1ClkFreq;
}

uint32_t timerClock(const timerHardware_t *timHw)
{
    return timerClockFromInstance(timHw->tim);
}

void timerReconfigureTimeBase(const timerHardware_t *timHw, uint16_t period, uint32_t hz)
{
    if (hz == 0) {
        return;
    }

    const uint8_t timerIndex = lookupTimerIndex((timerResource_t *)timHw->tim);
    if (timerIndex >= ARRAYLEN(timerInstances)) {
        return;
    }

    timerEnableClock(timHw->tim);

    TIM_TimeBaseInitType timeBaseInit;
    TIM_InitTimBaseStruct(&timeBaseInit);
    timeBaseInit.Period = (period > 0) ? ((period - 1U) & 0xFFFFU) : 0xFFFFU;
    timeBaseInit.Prescaler = timerPrescalerFromHz(timerClock(timHw), hz);
    timeBaseInit.CounterMode = TIM_CNT_MODE_UP;
    timeBaseInit.ClkDiv = TIM_CLK_DIV1;
    timeBaseInit.RepetCnt = 0;
    TIM_InitTimeBase((TIM_TypeDef *)timHw->tim, &timeBaseInit);
}

void timerConfigure(const timerHardware_t *timHw, uint16_t period, uint32_t hz)
{
    const uint8_t timerIndex = lookupTimerIndex((timerResource_t *)timHw->tim);
    if (timerIndex >= ARRAYLEN(timerInstances)) {
        return;
    }

    timerReconfigureTimeBase(timHw, period, hz);
    timerConfigureInterrupts(timerIndex, NVIC_PRIO_TIMER);
    if (isAdvancedTimer((TIM_TypeDef *)timHw->tim)) {
        TIM_EnableCtrlPwmOutputs((TIM_TypeDef *)timHw->tim, ENABLE);
    }
    TIM_Enable((TIM_TypeDef *)timHw->tim, ENABLE);
}

void timerStart(const timerHardware_t *timHw)
{
    timerEnableClock(timHw->tim);
    if (isAdvancedTimer((TIM_TypeDef *)timHw->tim)) {
        TIM_EnableCtrlPwmOutputs((TIM_TypeDef *)timHw->tim, ENABLE);
    }
    TIM_Enable((TIM_TypeDef *)timHw->tim, ENABLE);
}

void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq)
{
    UNUSED(type);
    UNUSED(irq);

    const uint8_t timerIndex = lookupTimerIndex((timerResource_t *)timHw->tim);
    if (timerIndex >= ARRAYLEN(timerInstances)) {
        return;
    }

    if (irqPriority < timerInfo[timerIndex].priority) {
        timerEnableClock(timHw->tim);
        timerReconfigureTimeBase(timHw, 0, 1);
        timerConfigureInterrupts(timerIndex, irqPriority);
        TIM_Enable((TIM_TypeDef *)timHw->tim, ENABLE);
        timerInfo[timerIndex].priority = irqPriority;
    }
}

void timerChannelEdgeHandlerInit(timerEdgeHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn = fn;
}

void timerChannelOverflowHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn = fn;
    self->next = NULL;
}

void timerChannelConfigCallbacks(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    const uint8_t timerIndex = lookupTimerIndex((timerResource_t *)timHw->tim);
    if (timerIndex >= ARRAYLEN(timerInstances)) {
        return;
    }

    const uint8_t channelIndex = lookupChannelIndex(timHw->channel);
    const uint32_t timerIt = channelToInterrupt(timHw->channel);

    if ((edgeCallback == NULL) && timerIt) {
        TIM_ConfigInt((TIM_TypeDef *)timHw->tim, timerIt, DISABLE);
    }

    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;

    if (edgeCallback && timerIt) {
        TIM_ClrIntPendingBit((TIM_TypeDef *)timHw->tim, timerIt);
        TIM_ConfigInt((TIM_TypeDef *)timHw->tim, timerIt, ENABLE);
    }

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

void timerChConfigCallbacksDual(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edgeCallbackLo, timerEdgeHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback)
{
    const uint8_t timerIndex = lookupTimerIndex((timerResource_t *)timHw->tim);
    if (timerIndex >= ARRAYLEN(timerInstances)) {
        return;
    }

    const uint16_t channelLo = timHw->channel & ~TIM_CH_2;
    const uint16_t channelHi = timHw->channel | TIM_CH_2;
    const uint8_t channelIndex = lookupChannelIndex(channelLo);
    const uint32_t timerItLo = channelToInterrupt(channelLo);
    const uint32_t timerItHi = channelToInterrupt(channelHi);

    if (edgeCallbackLo == NULL) {
        TIM_ConfigInt((TIM_TypeDef *)timHw->tim, timerItLo, DISABLE);
    }
    if (edgeCallbackHi == NULL) {
        TIM_ConfigInt((TIM_TypeDef *)timHw->tim, timerItHi, DISABLE);
    }

    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallbackLo;
    timerConfig[timerIndex].edgeCallback[channelIndex + 1] = edgeCallbackHi;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex + 1] = NULL;

    if (edgeCallbackLo) {
        TIM_ClrIntPendingBit((TIM_TypeDef *)timHw->tim, timerItLo);
        TIM_ConfigInt((TIM_TypeDef *)timHw->tim, timerItLo, ENABLE);
    }
    if (edgeCallbackHi) {
        TIM_ClrIntPendingBit((TIM_TypeDef *)timHw->tim, timerItHi);
        TIM_ConfigInt((TIM_TypeDef *)timHw->tim, timerItHi, ENABLE);
    }

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

void timerConfigUpdateCallback(const timerHardware_t *timHw, timerOvrHandlerRec_t *updateCallback)
{
    const uint8_t timerIndex = lookupTimerIndex((timerResource_t *)timHw->tim);
    if (timerIndex >= ARRAYLEN(timerInstances)) {
        return;
    }

    timerConfig[timerIndex].updateCallback = updateCallback;
    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

void timerChannelConfigInterruptDualLo(const timerHardware_t *timHw, FunctionalState newState)
{
    const uint16_t channelLo = timHw->channel & ~TIM_CH_2;
    const uint32_t timerItLo = channelToInterrupt(channelLo);
    if (timerItLo) {
        TIM_ConfigInt((TIM_TypeDef *)timHw->tim, timerItLo, newState);
    }
}

void timerChannelConfigInterrupt(const timerHardware_t *timHw, FunctionalState newState)
{
    const uint32_t timerIt = channelToInterrupt(timHw->channel);
    if (timerIt) {
        TIM_ConfigInt((TIM_TypeDef *)timHw->tim, timerIt, newState);
    }
}

void timerChannelClearFlag(const timerHardware_t *timHw)
{
    const uint32_t timerIt = channelToInterrupt(timHw->channel);
    if (timerIt) {
        TIM_ClrIntPendingBit((TIM_TypeDef *)timHw->tim, timerIt);
    }
}

void timerChannelConfigGPIO(const timerHardware_t *timHw, ioConfig_t mode)
{
    IOInit(IOGetByTag(timHw->tag), OWNER_TIMER, 0);
    IOConfigGPIO(IOGetByTag(timHw->tag), mode);
}

static unsigned getFilter(unsigned ticks)
{
    static const unsigned ftab[16] = {
        1U * 1U,
        1U * 2U, 1U * 4U, 1U * 8U,
        2U * 6U, 2U * 8U,
        4U * 6U, 4U * 8U,
        8U * 6U, 8U * 8U,
        16U * 5U, 16U * 6U, 16U * 8U,
        32U * 5U, 32U * 6U, 32U * 8U
    };

    for (unsigned i = 1; i < ARRAYLEN(ftab); i++) {
        if (ftab[i] > ticks) {
            return i - 1U;
        }
    }

    return 0x0FU;
}

void timerChannelConfigInput(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitType inputCaptureInit;
    TIM_InitIcStruct(&inputCaptureInit);
    inputCaptureInit.Channel = timHw->channel;
    inputCaptureInit.ICPolarity = polarityRising ? TIM_IC_POLARITY_RISING : TIM_IC_POLARITY_FALLING;
    inputCaptureInit.ICSelection = TIM_IC_SELECTION_DIRECTTI;
    inputCaptureInit.ICPrescaler = TIM_IC_PSC_DIV1;
    inputCaptureInit.ICFilter = getFilter(inputFilterTicks);
    TIM_ICInit((TIM_TypeDef *)timHw->tim, &inputCaptureInit);
}

void timerChannelConfigInputDual(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitType inputCaptureInit;
    bool directRising = (timHw->channel & TIM_CH_2) ? !polarityRising : polarityRising;

    TIM_InitIcStruct(&inputCaptureInit);
    inputCaptureInit.Channel = timHw->channel;
    inputCaptureInit.ICPolarity = directRising ? TIM_IC_POLARITY_RISING : TIM_IC_POLARITY_FALLING;
    inputCaptureInit.ICSelection = TIM_IC_SELECTION_DIRECTTI;
    inputCaptureInit.ICPrescaler = TIM_IC_PSC_DIV1;
    inputCaptureInit.ICFilter = getFilter(inputFilterTicks);
    TIM_ICInit((TIM_TypeDef *)timHw->tim, &inputCaptureInit);

    inputCaptureInit.Channel = timHw->channel ^ TIM_CH_2;
    inputCaptureInit.ICPolarity = directRising ? TIM_IC_POLARITY_FALLING : TIM_IC_POLARITY_RISING;
    inputCaptureInit.ICSelection = TIM_IC_SELECTION_INDIRECTTI;
    TIM_ICInit((TIM_TypeDef *)timHw->tim, &inputCaptureInit);
}

void timerChannelInputPolarity(const timerHardware_t *timHw, bool polarityRising)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    timCCER_t ccen = tim->CCEN;
    const timCCER_t polarityMask = (timCCER_t)(TIM_CCEN_CC1P << timHw->channel);
    ccen &= ~polarityMask;
    ccen |= polarityRising ? (TIM_IC_POLARITY_RISING << timHw->channel) : (TIM_IC_POLARITY_FALLING << timHw->channel);
    tim->CCEN = ccen;
}

void timerChannelConfigOutput(const timerHardware_t *timHw, bool outEnable, bool stateHigh)
{
    OCInitType ocInit;
    TIM_InitOcStruct(&ocInit);

    if (outEnable) {
        ocInit.OCMode = TIM_OCMODE_INACTIVE;
        if (timHw->output & TIMER_OUTPUT_N_CHANNEL) {
            ocInit.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
            ocInit.OCNIdleState = TIM_OCN_IDLE_STATE_RESET;
            ocInit.OCNPolarity = (timHw->output & TIMER_OUTPUT_INVERTED) ? (stateHigh ? TIM_OCN_POLARITY_HIGH : TIM_OCN_POLARITY_LOW) : (stateHigh ? TIM_OCN_POLARITY_LOW : TIM_OCN_POLARITY_HIGH);
        } else {
            ocInit.OutputState = TIM_OUTPUT_STATE_ENABLE;
            ocInit.OCIdleState = TIM_OC_IDLE_STATE_SET;
            if (timHw->output & TIMER_OUTPUT_INVERTED) {
                stateHigh = !stateHigh;
            }
            ocInit.OCPolarity = stateHigh ? TIM_OC_POLARITY_HIGH : TIM_OC_POLARITY_LOW;
        }
    } else {
        ocInit.OCMode = TIM_OCMODE_TIMING;
    }

    timerOCInit((TIM_TypeDef *)timHw->tim, timHw->channel, &ocInit);
    timerOCPreloadConfig((TIM_TypeDef *)timHw->tim, timHw->channel, TIM_OC_PRE_LOAD_DISABLE);
}

volatile timCCR_t *timerChCCR(const timerHardware_t *timHw)
{
    return timerGetCCRPointer((TIM_TypeDef *)timHw->tim, timHw->channel);
}

volatile timCCR_t *timerChCCRLo(const timerHardware_t *timHw)
{
    return timerGetCCRPointer((TIM_TypeDef *)timHw->tim, timHw->channel & ~TIM_CH_2);
}

volatile timCCR_t *timerChCCRHi(const timerHardware_t *timHw)
{
    return timerGetCCRPointer((TIM_TypeDef *)timHw->tim, timHw->channel | TIM_CH_2);
}

volatile timCCR_t *timerCCR(timerResource_t *tim, uint8_t channel)
{
    return timerGetCCRPointer((TIM_TypeDef *)tim, channel);
}

void timerChannelEnable(const timerHardware_t *timHw)
{
    if (timHw->output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_EnableCapCmpChN((TIM_TypeDef *)timHw->tim, timHw->channel, TIM_CAP_CMP_N_ENABLE);
    } else {
        TIM_EnableCapCmpCh((TIM_TypeDef *)timHw->tim, timHw->channel, TIM_CAP_CMP_ENABLE);
    }
}

void timerChannelDisable(const timerHardware_t *timHw)
{
    if (timHw->output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_EnableCapCmpChN((TIM_TypeDef *)timHw->tim, timHw->channel, TIM_CAP_CMP_N_DISABLE);
    } else {
        TIM_EnableCapCmpCh((TIM_TypeDef *)timHw->tim, timHw->channel, TIM_CAP_CMP_DISABLE);
    }
}

void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init)
{
    switch (channel) {
    case TIM_CH_1:
        TIM_InitOc1(tim, init);
        break;
    case TIM_CH_2:
        TIM_InitOc2(tim, init);
        break;
    case TIM_CH_3:
        TIM_InitOc3(tim, init);
        break;
    case TIM_CH_4:
        TIM_InitOc4(tim, init);
        break;
    }
}

void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t preload)
{
    switch (channel) {
    case TIM_CH_1:
        TIM_ConfigOc1Preload(tim, preload);
        break;
    case TIM_CH_2:
        TIM_ConfigOc2Preload(tim, preload);
        break;
    case TIM_CH_3:
        TIM_ConfigOc3Preload(tim, preload);
        break;
    case TIM_CH_4:
        TIM_ConfigOc4Preload(tim, preload);
        break;
    }
}

void timerForceOverflow(timerResource_t *tim)
{
    TIM_TypeDef *timer = (TIM_TypeDef *)tim;
    const uint8_t timerIndex = lookupTimerIndex((timerResource_t *)timer);
    if (timerIndex >= ARRAYLEN(timerInstances)) {
        return;
    }

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        timerConfig[timerIndex].forcedOverflowTimerValue = (timer->CNT & 0xFFFFU) + 1U;
        TIM_GenerateEvent(timer, TIM_EVT_SRC_UPDATE);
    }
}

uint16_t timerDmaSource(uint8_t channel)
{
    switch (channel) {
    case TIM_CH_1:
        return TIM_DMA_CC1;
    case TIM_CH_2:
        return TIM_DMA_CC2;
    case TIM_CH_3:
        return TIM_DMA_CC3;
    case TIM_CH_4:
        return TIM_DMA_CC4;
    default:
        return 0;
    }
}

uint16_t timerGetPrescalerByDesiredMhz(timerResource_t *tim, uint16_t mhz)
{
    return timerGetPrescalerByDesiredHertz(tim, MHZ_TO_HZ(mhz));
}

uint16_t timerGetPeriodByPrescaler(timerResource_t *tim, uint16_t prescaler, uint32_t hz)
{
    if (hz == 0) {
        return 0;
    }

    return (uint16_t)((timerClockFromInstance(tim) / (prescaler + 1U)) / hz);
}

uint16_t timerGetPrescalerByDesiredHertz(timerResource_t *tim, uint32_t hz)
{
    const uint32_t clock = timerClockFromInstance(tim);
    if ((hz == 0) || (hz > clock)) {
        return 0;
    }

    return (uint16_t)(((clock + (hz / 2U)) / hz) - 1U);
}

void timerReset(const timerHardware_t *timHw)
{
    TIM_Enable((TIM_TypeDef *)timHw->tim, DISABLE);
}

void timerSetPeriod(const timerHardware_t *timHw, uint32_t period)
{
    TIM_SetAutoReload((TIM_TypeDef *)timHw->tim, period);
}

uint32_t timerGetPeriod(const timerHardware_t *timHw)
{
    return TIM_GetAutoReload((TIM_TypeDef *)timHw->tim);
}

void timerSetCounter(const timerHardware_t *timHw, uint32_t counter)
{
    TIM_SetCnt((TIM_TypeDef *)timHw->tim, counter);
}

void timerDisable(const timerHardware_t *timHw)
{
    TIM_ConfigInt((TIM_TypeDef *)timHw->tim, TIM_INT_UPDATE, DISABLE);
    TIM_Enable((TIM_TypeDef *)timHw->tim, DISABLE);
}

void timerEnable(const timerHardware_t *timHw)
{
    if (isAdvancedTimer((TIM_TypeDef *)timHw->tim)) {
        TIM_EnableCtrlPwmOutputs((TIM_TypeDef *)timHw->tim, ENABLE);
    }
    TIM_Enable((TIM_TypeDef *)timHw->tim, ENABLE);
}

void timerEnableInterrupt(const timerHardware_t *timHw)
{
    TIM_ClrIntPendingBit((TIM_TypeDef *)timHw->tim, TIM_INT_UPDATE);
    TIM_ConfigInt((TIM_TypeDef *)timHw->tim, TIM_INT_UPDATE, ENABLE);
}

uint32_t timerGetPrescaler(const timerHardware_t *timHw)
{
    return TIM_GetPrescaler((TIM_TypeDef *)timHw->tim);
}

void *timerFindTimerHandle(timerResource_t *tim)
{
    return tim;
}

void timerInit(void)
{
    memset(timerConfig, 0, sizeof(timerConfig));
    memset(timerInfo, 0xFF, sizeof(timerInfo));
}

void pwmICConfig(timerResource_t *tim, uint8_t channel, uint16_t polarity, uint8_t filter)
{
    TIM_ICInitType inputCaptureInit;
    TIM_InitIcStruct(&inputCaptureInit);
    inputCaptureInit.Channel = channel;
    inputCaptureInit.ICPolarity = (polarity == TIMER_POLARITY_RISING) ? TIM_IC_POLARITY_RISING : TIM_IC_POLARITY_FALLING;
    inputCaptureInit.ICSelection = TIM_IC_SELECTION_DIRECTTI;
    inputCaptureInit.ICPrescaler = TIM_IC_PSC_DIV1;
    inputCaptureInit.ICFilter = filter;
    TIM_ICInit((TIM_TypeDef *)tim, &inputCaptureInit);
}

#endif
