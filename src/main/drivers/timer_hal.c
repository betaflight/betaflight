/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/nvic.h"

#include "drivers/io.h"
#include "rcc.h"
#include "drivers/time.h"

#include "timer.h"
#include "timer_impl.h"

#define TIM_N(n) (1 << (n))

/*
    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM1 2 channels
    TIM2 4 channels
    TIM3 4 channels
    TIM4 4 channels
*/


/// TODO: HAL in a lot af calls lookupTimerIndex is used. Instead of passing the timer instance the index should be passed.

#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)
#define CC_CHANNELS_PER_TIMER 4 // TIM_Channel_1..4

#define TIM_IT_CCx(ch) (TIM_IT_CC1 << ((ch) / 4))

typedef struct timerConfig_s {
    timerCCHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallbackActive; // null-terminated linkded list of active overflow callbacks
    uint32_t forcedOverflowTimerValue;
} timerConfig_t;
timerConfig_t timerConfig[USED_TIMER_COUNT+1];

typedef struct {
    channelType_t type;
} timerChannelInfo_t;
timerChannelInfo_t timerChannelInfo[USABLE_TIMER_CHANNEL_COUNT];

typedef struct {
    uint8_t priority;
} timerInfo_t;
timerInfo_t timerInfo[USED_TIMER_COUNT+1];

typedef struct
{
    TIM_HandleTypeDef Handle;
} timerHandle_t;
timerHandle_t timeHandle[USED_TIMER_COUNT+1];

// return index of timer in timer table. Lowest timer has index 0
#define TIMER_INDEX(i) BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

static uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
#define _CASE_SHF 10           // amount we can safely shift timer address to the right. gcc will throw error if some timers overlap
#define _CASE_(tim, index) case ((unsigned)tim >> _CASE_SHF): return index; break
#define _CASE(i) _CASE_(TIM##i##_BASE, TIMER_INDEX(i))

// let gcc do the work, switch should be quite optimized
    switch ((unsigned)tim >> _CASE_SHF) {
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
#if USED_TIMERS & TIM_N(9)
        _CASE(9);
#endif
#if USED_TIMERS & TIM_N(10)
        _CASE(10);
#endif
#if USED_TIMERS & TIM_N(11)
        _CASE(11);
#endif
#if USED_TIMERS & TIM_N(12)
        _CASE(12);
#endif
#if USED_TIMERS & TIM_N(13)
        _CASE(13);
#endif
#if USED_TIMERS & TIM_N(14)
        _CASE(14);
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
    default:  return ~1;  // make sure final index is out of range
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
#if USED_TIMERS & TIM_N(9)
    _DEF(9),
#endif
#if USED_TIMERS & TIM_N(10)
    _DEF(10),
#endif
#if USED_TIMERS & TIM_N(11)
    _DEF(11),
#endif
#if USED_TIMERS & TIM_N(12)
    _DEF(12),
#endif
#if USED_TIMERS & TIM_N(13)
    _DEF(13),
#endif
#if USED_TIMERS & TIM_N(14)
    _DEF(14),
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

static inline uint8_t lookupChannelIndex(const uint16_t channel)
{
    return channel >> 2;
}

rccPeriphTag_t timerRCC(TIM_TypeDef *tim)
{
    for (uint8_t i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].rcc;
        }
    }
    return 0;
}

uint8_t timerInputIrq(TIM_TypeDef *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].irq;
        }
    }
    return 0;
}

void timerNVICConfigure(uint8_t irq)
{
    HAL_NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
    HAL_NVIC_EnableIRQ(irq);
}

TIM_HandleTypeDef* timerFindTimerHandle(TIM_TypeDef *tim)
{
    uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex >= USED_TIMER_COUNT)
        return NULL;

    return &timeHandle[timerIndex].Handle;
}

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz)
{
    uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    if (timeHandle[timerIndex].Handle.Instance == tim)
    {
        // already configured
        return;
    }

    timeHandle[timerIndex].Handle.Instance = tim;

    timeHandle[timerIndex].Handle.Init.Period = (period - 1) & 0xffff; // AKA TIMx_ARR
    if (tim == TIM1 || tim == TIM8 || tim == TIM9 || tim == TIM10 || tim == TIM11)
        timeHandle[timerIndex].Handle.Init.Prescaler = (HAL_RCC_GetPCLK2Freq() * 2 / ((uint32_t)mhz * 1000000)) - 1;
    else
        timeHandle[timerIndex].Handle.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() * 2 * 2  / ((uint32_t)mhz * 1000000)) - 1;

    timeHandle[timerIndex].Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timeHandle[timerIndex].Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    timeHandle[timerIndex].Handle.Init.RepetitionCounter = 0x0000;

    HAL_TIM_Base_Init(&timeHandle[timerIndex].Handle);
    if (tim == TIM1 || tim == TIM2 || tim == TIM3 || tim == TIM4 || tim == TIM5 || tim == TIM8 || tim == TIM9)
    {
        TIM_ClockConfigTypeDef sClockSourceConfig;
        sClockSourceConfig.ClockFilter = 0;
        sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
        sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(&timeHandle[timerIndex].Handle, &sClockSourceConfig) != HAL_OK)
        {
            return;
        }
    }
    if (tim == TIM1 || tim == TIM2 || tim == TIM3 || tim == TIM4 || tim == TIM5 || tim == TIM8 )
    {
        TIM_MasterConfigTypeDef sMasterConfig;
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&timeHandle[timerIndex].Handle, &sMasterConfig) != HAL_OK)
        {
            return;
        }
    }

}

// old interface for PWM inputs. It should be replaced
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz)
{
    uint8_t timerIndex = lookupTimerIndex(timerHardwarePtr->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    configTimeBase(timerHardwarePtr->tim, period, mhz);
    HAL_TIM_Base_Start(&timeHandle[timerIndex].Handle);

    uint8_t irq = timerInputIrq(timerHardwarePtr->tim);
    timerNVICConfigure(irq);
    // HACK - enable second IRQ on timers that need it
    switch (irq) {

    case TIM1_CC_IRQn:
        timerNVICConfigure(TIM1_UP_TIM10_IRQn);
        break;
    case TIM8_CC_IRQn:
        timerNVICConfigure(TIM8_UP_TIM13_IRQn);
        break;

    }
}

// allocate and configure timer channel. Timer priority is set to highest priority of its channels
void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    unsigned channel = timHw - timerHardware;
    if (channel >= USABLE_TIMER_CHANNEL_COUNT)
        return;

    timerChannelInfo[channel].type = type;
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;
    if (irqPriority < timerInfo[timer].priority) {
        // it would be better to set priority in the end, but current startup sequence is not ready
        configTimeBase(usedTimers[timer], 0, 1);
        HAL_TIM_Base_Start(&timeHandle[timerIndex].Handle);
        HAL_NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));
        HAL_NVIC_EnableIRQ(irq);

        timerInfo[timer].priority = irqPriority;
    }
}

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn = fn;
}

void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn = fn;
    self->next = NULL;
}

// update overflow callback list
// some synchronization mechanism is neccesary to avoid disturbing other channels (BASEPRI used now)
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, TIM_TypeDef *tim) {
    uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    timerOvrHandlerRec_t **chain = &cfg->overflowCallbackActive;
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        for (int i = 0; i < CC_CHANNELS_PER_TIMER; i++)
            if (cfg->overflowCallback[i]) {
                *chain = cfg->overflowCallback[i];
                chain = &cfg->overflowCallback[i]->next;
            }
        *chain = NULL;
    }
    // enable or disable IRQ
    if (cfg->overflowCallbackActive)
        __HAL_TIM_ENABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_UPDATE);
    else
        __HAL_TIM_DISABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_UPDATE);
}

// config edge and overflow callback for channel. Try to avoid overflowCallback, it is a bit expensive
void timerChConfigCallbacks(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint8_t channelIndex = lookupChannelIndex(timHw->channel);
    if (edgeCallback == NULL)   // disable irq before changing callback to NULL
        __HAL_TIM_DISABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    // enable channel IRQ
    if (edgeCallback)
        __HAL_TIM_ENABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

// configure callbacks for pair of channels (1+2 or 3+4).
// Hi(2,4) and Lo(1,3) callbacks are specified, it is not important which timHw channel is used.
// This is intended for dual capture mode (each channel handles one transition)
void timerChConfigCallbacksDual(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint16_t chLo = timHw->channel & ~TIM_CHANNEL_2;   // lower channel
    uint16_t chHi = timHw->channel | TIM_CHANNEL_2;    // upper channel
    uint8_t channelIndex = lookupChannelIndex(chLo);   // get index of lower channel

    if (edgeCallbackLo == NULL)   // disable irq before changing setting callback to NULL
        __HAL_TIM_DISABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_CCx(chLo));
    if (edgeCallbackHi == NULL)   // disable irq before changing setting callback to NULL
        __HAL_TIM_DISABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_CCx(chHi));

    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallbackLo;
    timerConfig[timerIndex].edgeCallback[channelIndex + 1] = edgeCallbackHi;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex + 1] = NULL;

    // enable channel IRQs
    if (edgeCallbackLo) {
        __HAL_TIM_CLEAR_FLAG(&timeHandle[timerIndex].Handle, TIM_IT_CCx(chLo));
        __HAL_TIM_ENABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_CCx(chLo));
    }
    if (edgeCallbackHi) {
        __HAL_TIM_CLEAR_FLAG(&timeHandle[timerIndex].Handle, TIM_IT_CCx(chHi));
        __HAL_TIM_ENABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_CCx(chHi));
    }

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

// enable/disable IRQ for low channel in dual configuration
//void timerChITConfigDualLo(const timerHardware_t *timHw, FunctionalState newState) {
//    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel&~TIM_Channel_2), newState);
//}
// enable/disable IRQ for low channel in dual configuration
void timerChITConfigDualLo(const timerHardware_t *timHw, FunctionalState newState) {
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    if (newState)
        __HAL_TIM_ENABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel&~TIM_CHANNEL_2));
    else
        __HAL_TIM_DISABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel&~TIM_CHANNEL_2));
}

//// enable or disable IRQ
//void timerChITConfig(const timerHardware_t *timHw, FunctionalState newState)
//{
//    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), newState);
//}
// enable or disable IRQ
void timerChITConfig(const timerHardware_t *timHw, FunctionalState newState)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    if (newState)
        __HAL_TIM_ENABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
    else
        __HAL_TIM_DISABLE_IT(&timeHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
}

// clear Compare/Capture flag for channel
//void timerChClearCCFlag(const timerHardware_t *timHw)
//{
//    TIM_ClearFlag(timHw->tim, TIM_IT_CCx(timHw->channel));
//}
// clear Compare/Capture flag for channel
void timerChClearCCFlag(const timerHardware_t *timHw)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    __HAL_TIM_CLEAR_FLAG(&timeHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
}

// configure timer channel GPIO mode
void timerChConfigGPIO(const timerHardware_t* timHw, ioConfig_t mode)
{
    IOInit(IOGetByTag(timHw->tag), OWNER_TIMER, RESOURCE_TIMER, 0);
    IOConfigGPIO(IOGetByTag(timHw->tag), mode);
}

// calculate input filter constant
// TODO - we should probably setup DTS to higher value to allow reasonable input filtering
//   - notice that prescaler[0] does use DTS for sampling - the sequence won't be monotonous anymore
static unsigned getFilter(unsigned ticks)
{
    static const unsigned ftab[16] = {
        1*1,                 // fDTS !
        1*2, 1*4, 1*8,       // fCK_INT
        2*6, 2*8,            // fDTS/2
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

// Configure input captupre
void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;

    TIM_IC_InitTypeDef TIM_ICInitStructure;

    TIM_ICInitStructure.ICPolarity = polarityRising ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
    TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
    TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.ICFilter = getFilter(inputFilterTicks);
    HAL_TIM_IC_ConfigChannel(&timeHandle[timer].Handle,&TIM_ICInitStructure, timHw->channel);
}

// configure dual channel input channel for capture
// polarity is for Low channel (capture order is always Lo - Hi)
void timerChConfigICDual(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;

    TIM_IC_InitTypeDef TIM_ICInitStructure;
    bool directRising = (timHw->channel & TIM_CHANNEL_2) ? !polarityRising : polarityRising;

    // configure direct channel
    TIM_ICInitStructure.ICPolarity = directRising ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
    TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
    TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.ICFilter = getFilter(inputFilterTicks);
    HAL_TIM_IC_ConfigChannel(&timeHandle[timer].Handle,&TIM_ICInitStructure, timHw->channel);

    // configure indirect channel
    TIM_ICInitStructure.ICPolarity = directRising ? TIM_ICPOLARITY_FALLING : TIM_ICPOLARITY_RISING;
    TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&timeHandle[timer].Handle,&TIM_ICInitStructure, timHw->channel ^ TIM_CHANNEL_2);
}

void timerChICPolarity(const timerHardware_t *timHw, bool polarityRising)
{
    timCCER_t tmpccer = timHw->tim->CCER;
    tmpccer &= ~(TIM_CCER_CC1P << timHw->channel);
    tmpccer |= polarityRising ? (TIM_ICPOLARITY_RISING << timHw->channel) : (TIM_ICPOLARITY_FALLING << timHw->channel);
    timHw->tim->CCER = tmpccer;
}

volatile timCCR_t* timerChCCRHi(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + (timHw->channel | TIM_CHANNEL_2));
}

volatile timCCR_t* timerChCCRLo(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + (timHw->channel & ~TIM_CHANNEL_2));
}

uint16_t timerGetPeriod(const timerHardware_t *timHw)
{
    return timHw->tim->ARR;
}

volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + timHw->channel);
}

void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool stateHigh)
{
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;

    TIM_OC_InitTypeDef  TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TIM_OCMODE_INACTIVE;
    TIM_OCInitStructure.Pulse = 0x00000000;
    TIM_OCInitStructure.OCPolarity = stateHigh ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
    TIM_OCInitStructure.OCNPolarity = TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_OC_ConfigChannel(&timeHandle[timer].Handle, &TIM_OCInitStructure, timHw->channel);

    if (outEnable) {
        TIM_OCInitStructure.OCMode = TIM_OCMODE_INACTIVE;
        HAL_TIM_OC_ConfigChannel(&timeHandle[timer].Handle, &TIM_OCInitStructure, timHw->channel);
        HAL_TIM_OC_Start(&timeHandle[timer].Handle, timHw->channel);
    } else {
        TIM_OCInitStructure.OCMode = TIM_OCMODE_TIMING;
        HAL_TIM_OC_ConfigChannel(&timeHandle[timer].Handle, &TIM_OCInitStructure, timHw->channel);
        HAL_TIM_OC_Start_IT(&timeHandle[timer].Handle, timHw->channel);
    }
}

static void timCCxHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim->SR & tim->DIER;
#if 1
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->SR = mask;
        tim_status &= mask;
        switch (bit) {
            case __builtin_clz(TIM_IT_UPDATE): {

                if (timerConfig->forcedOverflowTimerValue != 0){
                    capture = timerConfig->forcedOverflowTimerValue - 1;
                    timerConfig->forcedOverflowTimerValue = 0;
                } else {
                    capture = tim->ARR;
                }

                timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
                while (cb) {
                    cb->fn(cb, capture);
                    cb = cb->next;
                }
                break;
            }
            case __builtin_clz(TIM_IT_CC1):
                timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);
                break;
            case __builtin_clz(TIM_IT_CC2):
                timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CCR2);
                break;
            case __builtin_clz(TIM_IT_CC3):
                timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
                break;
            case __builtin_clz(TIM_IT_CC4):
                timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
                break;
        }
    }
#else
    if (tim_status & (int)TIM_IT_Update) {
        tim->SR = ~TIM_IT_Update;
        capture = tim->ARR;
        timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
        while (cb) {
            cb->fn(cb, capture);
            cb = cb->next;
        }
    }
    if (tim_status & (int)TIM_IT_CC1) {
        tim->SR = ~TIM_IT_CC1;
        timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);
    }
    if (tim_status & (int)TIM_IT_CC2) {
        tim->SR = ~TIM_IT_CC2;
        timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CCR2);
    }
    if (tim_status & (int)TIM_IT_CC3) {
        tim->SR = ~TIM_IT_CC3;
        timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
    }
    if (tim_status & (int)TIM_IT_CC4) {
        tim->SR = ~TIM_IT_CC4;
        timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
    }
#endif
}


// handler for shared interrupts when both timers need to check status bits
#define _TIM_IRQ_HANDLER2(name, i, j)                                   \
    void name(void)                                                     \
    {                                                                   \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
        timCCxHandler(TIM ## j, &timerConfig[TIMER_INDEX(j)]);          \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
    } struct dummy

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
#  if USED_TIMERS & TIM_N(10)
_TIM_IRQ_HANDLER2(TIM1_UP_TIM10_IRQHandler, 1,10);  // both timers are in use
#  else
_TIM_IRQ_HANDLER(TIM1_UP_TIM10_IRQHandler, 1);     // timer10 is not used
#  endif
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

#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8);

#  if USED_TIMERS & TIM_N(13)
_TIM_IRQ_HANDLER2(TIM8_UP_TIM13_IRQHandler, 8,13);  // both timers are in use
#  else
_TIM_IRQ_HANDLER(TIM8_UP_TIM13_IRQHandler, 8);     // timer13 is not used
#  endif
#endif

#if USED_TIMERS & TIM_N(9)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM9_IRQHandler, 9);
#endif
#  if USED_TIMERS & TIM_N(11)
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM11_IRQHandler, 11);
#  endif
#if USED_TIMERS & TIM_N(12)
_TIM_IRQ_HANDLER(TIM8_BRK_TIM12_IRQHandler, 12);
#endif
#if USED_TIMERS & TIM_N(15)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM15_IRQHandler, 15);
#endif
#if defined(STM32F303xC) && ((USED_TIMERS & (TIM_N(1)|TIM_N(16))) == (TIM_N(16)))
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 16);    // only timer16 is used, not timer1
#endif
#if USED_TIMERS & TIM_N(17)
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM17_IRQHandler, 17);
#endif

void timerInit(void)
{
    memset(timerConfig, 0, sizeof (timerConfig));


#if USED_TIMERS & TIM_N(1)
    __HAL_RCC_TIM1_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(2)
    __HAL_RCC_TIM2_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(3)
    __HAL_RCC_TIM3_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(4)
    __HAL_RCC_TIM4_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(5)
    __HAL_RCC_TIM5_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(6)
    __HAL_RCC_TIM6_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(7)
    __HAL_RCC_TIM7_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(8)
    __HAL_RCC_TIM8_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(9)
    __HAL_RCC_TIM9_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(10)
    __HAL_RCC_TIM10_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(11)
    __HAL_RCC_TIM11_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(12)
    __HAL_RCC_TIM12_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(13)
    __HAL_RCC_TIM13_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(14)
    __HAL_RCC_TIM14_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(15)
    __HAL_RCC_TIM15_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(16)
    __HAL_RCC_TIM16_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(17)
    __HAL_RCC_TIM17_CLK_ENABLE();
#endif

#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7)
    for (uint8_t timerIndex = 0; timerIndex < USABLE_TIMER_CHANNEL_COUNT; timerIndex++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];
        IOConfigGPIOAF(IOGetByTag(timerHardwarePtr->tag), timerHardwarePtr->ioMode, timerHardwarePtr->alternateFunction);
    }
#endif

    // initialize timer channel structures
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        timerChannelInfo[i].type = TYPE_FREE;
    }
    for (int i = 0; i < USED_TIMER_COUNT; i++) {
        timerInfo[i].priority = ~0;
    }
}

// finish configuring timers after allocation phase
// start timers
// TODO - Work in progress - initialization routine must be modified/verified to start correctly without timers
void timerStart(void)
{
#if 0
    for (unsigned timer = 0; timer < USED_TIMER_COUNT; timer++) {
        int priority = -1;
        int irq = -1;
        for (unsigned hwc = 0; hwc < USABLE_TIMER_CHANNEL_COUNT; hwc++)
            if ((timerChannelInfo[hwc].type != TYPE_FREE) && (timerHardware[hwc].tim == usedTimers[timer])) {
                // TODO - move IRQ to timer info
                irq = timerHardware[hwc].irq;
            }
        // TODO - aggregate required timer paramaters
        configTimeBase(usedTimers[timer], 0, 1);
        TIM_Cmd(usedTimers[timer],  ENABLE);
        if (priority >= 0) {  // maybe none of the channels was configured
            NVIC_InitTypeDef NVIC_InitStructure;

            NVIC_InitStructure.NVIC_IRQChannel = irq;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SPLIT_PRIORITY_BASE(priority);
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_SPLIT_PRIORITY_SUB(priority);
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
        }
    }
#endif
}

/**
 * Force an overflow for a given timer.
 * Saves the current value of the counter in the relevant timerConfig's forcedOverflowTimerValue variable.
 * @param TIM_Typedef *tim The timer to overflow
 * @return void
 **/
void timerForceOverflow(TIM_TypeDef *tim)
{
    uint8_t timerIndex = lookupTimerIndex((const TIM_TypeDef *)tim);

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        // Save the current count so that PPM reading will work on the same timer that was forced to overflow
        timerConfig[timerIndex].forcedOverflowTimerValue = tim->CNT + 1;

        // Force an overflow by setting the UG bit
        tim->EGR |= TIM_EGR_UG;
    }
}

const timerHardware_t *timerGetByTag(ioTag_t tag, timerUsageFlag_e flag)
{
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if (timerHardware[i].tag == tag) {
            if (timerHardware[i].usageFlags & flag || flag == 0) {
                return &timerHardware[i];
            }
        }
    }
    return NULL;
}

uint16_t timerDmaSource(uint8_t channel)
{
    switch (channel) {
        case TIM_CHANNEL_1:
            return TIM_DMA_ID_CC1;
        case TIM_CHANNEL_2:
            return TIM_DMA_ID_CC2;
        case TIM_CHANNEL_3:
            return TIM_DMA_ID_CC3;
        case TIM_CHANNEL_4:
            return TIM_DMA_ID_CC4;
    }
    return 0;
}

uint16_t timerGetPrescalerByDesiredMhz(TIM_TypeDef *tim, uint16_t mhz)
{
    // protection here for desired MHZ > SystemCoreClock???
    if ((uint32_t)(mhz * 1000000) > (SystemCoreClock / timerClockDivisor(tim))) {
        return 0;
    }
    return (uint16_t)(round((SystemCoreClock / timerClockDivisor(tim) / (mhz * 1000000)) - 1));
}

uint16_t timerGetPeriodByPrescaler(TIM_TypeDef *tim, uint16_t prescaler, uint32_t hertz)
{
    return ((uint16_t)((SystemCoreClock / timerClockDivisor(tim) / (prescaler + 1)) / hertz));
}
