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
#include <math.h>

#include "platform.h"

#ifdef USE_TIMER

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/nvic.h"

#include "drivers/io.h"
#include "drivers/dma.h"

#include "rcc.h"

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

#define TIM_IT_CCx(ch) (TIM_IT_CC1 << ((ch) / 4))

typedef struct timerConfig_s {
    // per-timer
    timerOvrHandlerRec_t *updateCallback;

    // per-channel
    timerCCHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallback[CC_CHANNELS_PER_TIMER];

    // state
    timerOvrHandlerRec_t *overflowCallbackActive; // null-terminated linkded list of active overflow callbacks
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

typedef struct {
    TIM_HandleTypeDef Handle;
} timerHandle_t;
timerHandle_t timerHandle[USED_TIMER_COUNT + 1];

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
#if USED_TIMERS & TIM_N(20)
        _CASE(20);
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
#if !(defined(STM32H7) || defined(STM32G4))
#if USED_TIMERS & TIM_N(9)
    _DEF(9),
#endif
#if USED_TIMERS & TIM_N(10)
    _DEF(10),
#endif
#if USED_TIMERS & TIM_N(11)
    _DEF(11),
#endif
#endif
#if !defined(STM32G4)
#if USED_TIMERS & TIM_N(12)
    _DEF(12),
#endif
#if USED_TIMERS & TIM_N(13)
    _DEF(13),
#endif
#if USED_TIMERS & TIM_N(14)
    _DEF(14),
#endif
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
#if defined(STM32G4)
#if USED_TIMERS & TIM_N(20)
    _DEF(20),
#endif
#endif
#undef _DEF
};

// Map timer index to timer number (Straight copy of usedTimers array)
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
#if USED_TIMERS & TIM_N(20)
    _DEF(20),
#endif
#undef _DEF
};

int8_t timerGetNumberByIndex(uint8_t index)
{
    if (index < USED_TIMER_COUNT) {
        return timerNumbers[index];
    } else {
        return 0;
    }
}

int8_t timerGetTIMNumber(const TIM_TypeDef *tim)
{
    uint8_t index = lookupTimerIndex(tim);

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

rccPeriphTag_t timerRCC(TIM_TypeDef *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
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
            return timerDefinitions[i].inputIrq;
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

    return &timerHandle[timerIndex].Handle;
}

void timerReconfigureTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz)
{
    TIM_HandleTypeDef* handle = timerFindTimerHandle(tim);
    if (handle == NULL) return;

    handle->Init.Period = (period - 1) & 0xffff; // AKA TIMx_ARR
    handle->Init.Prescaler = (timerClock(tim) / hz) - 1;

    TIM_Base_SetConfig(handle->Instance, &handle->Init);
}

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz)
{
    TIM_HandleTypeDef* handle = timerFindTimerHandle(tim);
    if (handle == NULL) return;

    if (handle->Instance == tim) {
        // already configured
        return;
    }

    handle->Instance = tim;

    handle->Init.Period = (period - 1) & 0xffff; // AKA TIMx_ARR
    handle->Init.Prescaler = (timerClock(tim) / hz) - 1;

    handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    handle->Init.CounterMode = TIM_COUNTERMODE_UP;
    handle->Init.RepetitionCounter = 0x0000;

    HAL_TIM_Base_Init(handle);
    if (tim == TIM1 || tim == TIM2 || tim == TIM3 || tim == TIM4 || tim == TIM5 || tim == TIM8
#if !(defined(STM32H7) || defined(STM32G4))
        || tim == TIM9
#endif
      ) {
        TIM_ClockConfigTypeDef sClockSourceConfig;
        memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(handle, &sClockSourceConfig) != HAL_OK) {
            return;
        }
    }
    if (tim == TIM1 || tim == TIM2 || tim == TIM3 || tim == TIM4 || tim == TIM5 || tim == TIM8) {
        TIM_MasterConfigTypeDef sMasterConfig;
        memset(&sMasterConfig, 0, sizeof(sMasterConfig));
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(handle, &sMasterConfig) != HAL_OK) {
            return;
        }
    }
}

// old interface for PWM inputs. It should be replaced
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint32_t hz)
{
    uint8_t timerIndex = lookupTimerIndex(timerHardwarePtr->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    configTimeBase(timerHardwarePtr->tim, period, hz);
    HAL_TIM_Base_Start(&timerHandle[timerIndex].Handle);

    uint8_t irq = timerInputIrq(timerHardwarePtr->tim);
    timerNVICConfigure(irq);
    // HACK - enable second IRQ on timers that need it
    switch (irq) {

    case TIM1_CC_IRQn:
#if defined(STM32F7)
        timerNVICConfigure(TIM1_UP_TIM10_IRQn);
#elif defined(STM32H7)
        timerNVICConfigure(TIM1_UP_IRQn);
#elif defined(STM32G4)
        timerNVICConfigure(TIM1_UP_TIM16_IRQn);
#else
        // Empty
#endif
        break;
    case TIM8_CC_IRQn:
#if defined(STM32G4)
        timerNVICConfigure(TIM8_UP_IRQn);
#else
        timerNVICConfigure(TIM8_UP_TIM13_IRQn);
#endif
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
    unsigned channel = timHw - TIMER_HARDWARE;
    if (channel >= TIMER_CHANNEL_COUNT)
        return;

    timerChannelInfo[channel].type = type;
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;
    if (irqPriority < timerInfo[timer].priority) {
        // it would be better to set priority in the end, but current startup sequence is not ready
        configTimeBase(usedTimers[timer], 0, 1);
        HAL_TIM_Base_Start(&timerHandle[timerIndex].Handle);

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
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, const TIM_TypeDef *tim)
{
    uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

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
    // enable or disable IRQ
    if (cfg->overflowCallbackActive)
        __HAL_TIM_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_UPDATE);
    else
        __HAL_TIM_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_UPDATE);
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
        __HAL_TIM_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    // enable channel IRQ
    if (edgeCallback)
        __HAL_TIM_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

void timerConfigUpdateCallback(const TIM_TypeDef *tim, timerOvrHandlerRec_t *updateCallback)
{
    uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    timerConfig[timerIndex].updateCallback = updateCallback;
    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], tim);
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
        __HAL_TIM_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chLo));
    if (edgeCallbackHi == NULL)   // disable irq before changing setting callback to NULL
        __HAL_TIM_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chHi));

    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallbackLo;
    timerConfig[timerIndex].edgeCallback[channelIndex + 1] = edgeCallbackHi;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex + 1] = NULL;

    // enable channel IRQs
    if (edgeCallbackLo) {
        __HAL_TIM_CLEAR_FLAG(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chLo));
        __HAL_TIM_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chLo));
    }
    if (edgeCallbackHi) {
        __HAL_TIM_CLEAR_FLAG(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chHi));
        __HAL_TIM_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chHi));
    }

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

// enable/disable IRQ for low channel in dual configuration
//void timerChITConfigDualLo(const timerHardware_t *timHw, FunctionalState newState) {
//    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel&~TIM_Channel_2), newState);
//}
// enable/disable IRQ for low channel in dual configuration
void timerChITConfigDualLo(const timerHardware_t *timHw, FunctionalState newState)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    if (newState)
        __HAL_TIM_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel & ~TIM_CHANNEL_2));
    else
        __HAL_TIM_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel & ~TIM_CHANNEL_2));
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
        __HAL_TIM_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
    else
        __HAL_TIM_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
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

    __HAL_TIM_CLEAR_FLAG(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
}

// configure timer channel GPIO mode
void timerChConfigGPIO(const timerHardware_t* timHw, ioConfig_t mode)
{
    IOInit(IOGetByTag(timHw->tag), OWNER_TIMER, 0);
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
    HAL_TIM_IC_ConfigChannel(&timerHandle[timer].Handle, &TIM_ICInitStructure, timHw->channel);
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
    HAL_TIM_IC_ConfigChannel(&timerHandle[timer].Handle, &TIM_ICInitStructure, timHw->channel);

    // configure indirect channel
    TIM_ICInitStructure.ICPolarity = directRising ? TIM_ICPOLARITY_FALLING : TIM_ICPOLARITY_RISING;
    TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&timerHandle[timer].Handle, &TIM_ICInitStructure, timHw->channel ^ TIM_CHANNEL_2);
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

volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + timHw->channel);
}

void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool stateHigh)
{
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TIM_OCMODE_INACTIVE;
    TIM_OCInitStructure.Pulse = 0x00000000;
    TIM_OCInitStructure.OCPolarity = stateHigh ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
    TIM_OCInitStructure.OCNPolarity = TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_OC_ConfigChannel(&timerHandle[timer].Handle, &TIM_OCInitStructure, timHw->channel);

    if (outEnable) {
        TIM_OCInitStructure.OCMode = TIM_OCMODE_INACTIVE;
        HAL_TIM_OC_ConfigChannel(&timerHandle[timer].Handle, &TIM_OCInitStructure, timHw->channel);
        HAL_TIM_OC_Start(&timerHandle[timer].Handle, timHw->channel);
    } else {
        TIM_OCInitStructure.OCMode = TIM_OCMODE_TIMING;
        HAL_TIM_OC_ConfigChannel(&timerHandle[timer].Handle, &TIM_OCInitStructure, timHw->channel);
        HAL_TIM_OC_Start_IT(&timerHandle[timer].Handle, timHw->channel);
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

            if (timerConfig->forcedOverflowTimerValue != 0) {
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

static inline void timUpdateHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim->SR & tim->DIER;
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->SR = mask;
        tim_status &= mask;
        switch (bit) {
        case __builtin_clz(TIM_IT_UPDATE): {

            if (timerConfig->forcedOverflowTimerValue != 0) {
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
        }
    }
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

#define _TIM_IRQ_HANDLER_UPDATE_ONLY(name, i)                           \
    void name(void)                                                     \
    {                                                                   \
        timUpdateHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);       \
    } struct dummy

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
#  if defined(STM32H7)
_TIM_IRQ_HANDLER(TIM1_UP_IRQHandler, 1);
#  elif defined(STM32G4)
#    if USED_TIMERS & TIM_N(16)
_TIM_IRQ_HANDLER2(TIM1_UP_TIM16_IRQHandler, 1, 16);  // both timers are in use
#    else
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 1);       // timer16 is not used timers are in use
#    endif
#  else
#    if USED_TIMERS & TIM_N(10)
_TIM_IRQ_HANDLER2(TIM1_UP_TIM10_IRQHandler, 1, 10);  // both timers are in use
#    else
_TIM_IRQ_HANDLER(TIM1_UP_TIM10_IRQHandler, 1);     // timer10 is not used
#    endif
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

#if USED_TIMERS & TIM_N(6)
#  if !(defined(USE_PID_AUDIO) && (defined(STM32H7) || defined(STM32F7)))
_TIM_IRQ_HANDLER_UPDATE_ONLY(TIM6_DAC_IRQHandler, 6);
#  endif
#endif
#if USED_TIMERS & TIM_N(7)
// The USB VCP_HAL driver conflicts with TIM7, see TIMx_IRQHandler in usbd_cdc_interface.h
#  if !(defined(USE_VCP) && (defined(STM32F4) || defined(STM32G4) || defined(STM32H7) || defined(STM32F7)))
#    if defined(STM32G4)
_TIM_IRQ_HANDLER_UPDATE_ONLY(TIM7_DAC_IRQHandler, 7);
#    else
_TIM_IRQ_HANDLER_UPDATE_ONLY(TIM7_IRQHandler, 7);
#    endif
#  endif
#endif

#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8);
#  if defined(STM32G4)
_TIM_IRQ_HANDLER(TIM8_UP_IRQHandler, 8);
#  endif

#  if USED_TIMERS & TIM_N(13)
_TIM_IRQ_HANDLER2(TIM8_UP_TIM13_IRQHandler, 8, 13);  // both timers are in use
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
#if defined(STM32H7) && (USED_TIMERS & TIM_N(14))
_TIM_IRQ_HANDLER(TIM8_TRG_COM_TIM14_IRQHandler, 14);
#endif
#if USED_TIMERS & TIM_N(15)
#  if defined(STM32H7)
_TIM_IRQ_HANDLER(TIM15_IRQHandler, 15);
#  else
_TIM_IRQ_HANDLER(TIM1_BRK_TIM15_IRQHandler, 15);
#  endif
#endif
#if defined(STM32H7) && (USED_TIMERS & TIM_N(16))
_TIM_IRQ_HANDLER(TIM16_IRQHandler, 16);
#endif
#if USED_TIMERS & TIM_N(17)
#  if defined(STM32H7)
_TIM_IRQ_HANDLER(TIM17_IRQHandler, 17);
#  else
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM17_IRQHandler, 17);
#  endif
#endif
#if USED_TIMERS & TIM_N(20)
_TIM_IRQ_HANDLER(TIM20_CC_IRQHandler, 20);
#endif

void timerInit(void)
{
    memset(timerConfig, 0, sizeof(timerConfig));

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
#if !defined(STM32H7)
#if USED_TIMERS & TIM_N(9)
    __HAL_RCC_TIM9_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(10)
    __HAL_RCC_TIM10_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(11)
    __HAL_RCC_TIM11_CLK_ENABLE();
#endif
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
#if USED_TIMERS & TIM_N(20)
    __HAL_RCC_TIM20_CLK_ENABLE();
#endif

    /* enable the timer peripherals */
    for (int i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        RCC_ClockCmd(timerRCC(TIMER_HARDWARE[i].tim), ENABLE);
    }

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
    for (unsigned timerIndex = 0; timerIndex < TIMER_CHANNEL_COUNT; timerIndex++) {
        const timerHardware_t *timerHardwarePtr = &TIMER_HARDWARE[timerIndex];
        if (timerHardwarePtr->usageFlags == TIM_USE_NONE) {
            continue;
        }
        // XXX IOConfigGPIOAF in timerInit should eventually go away.
        IOConfigGPIOAF(IOGetByTag(timerHardwarePtr->tag), IOCFG_AF_PP, timerHardwarePtr->alternateFunction);
    }
#endif

    /* enable the timer peripherals */
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        RCC_ClockCmd(timerRCC(TIMER_HARDWARE[i].tim), ENABLE);
    }

    // initialize timer channel structures
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        timerChannelInfo[i].type = TYPE_FREE;
    }

    for (unsigned i = 0; i < USED_TIMER_COUNT; i++) {
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
        for (unsigned hwc = 0; hwc < TIMER_CHANNEL_COUNT; hwc++) {
            if ((timerChannelInfo[hwc].type != TYPE_FREE) && (TIMER_HARDWARE[hwc].tim == usedTimers[timer])) {
                // TODO - move IRQ to timer info
                irq = TIMER_HARDWARE[hwc].irq;
            }
        }
        // TODO - aggregate required timer paramaters
        configTimeBase(usedTimers[timer], 0, 1);
        TIM_Cmd(usedTimers[timer], ENABLE);
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

// DMA_Handle_index
uint16_t timerDmaIndex(uint8_t channel)
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

// TIM_DMA_sources
uint16_t timerDmaSource(uint8_t channel)
{
    switch (channel) {
    case TIM_CHANNEL_1:
        return TIM_DMA_CC1;
    case TIM_CHANNEL_2:
        return TIM_DMA_CC2;
    case TIM_CHANNEL_3:
        return TIM_DMA_CC3;
    case TIM_CHANNEL_4:
        return TIM_DMA_CC4;
    }
    return 0;
}

uint16_t timerGetPrescalerByDesiredMhz(TIM_TypeDef *tim, uint16_t mhz)
{
    return timerGetPrescalerByDesiredHertz(tim, MHZ_TO_HZ(mhz));
}

uint16_t timerGetPeriodByPrescaler(TIM_TypeDef *tim, uint16_t prescaler, uint32_t hz)
{
    return (uint16_t)((timerClock(tim) / (prescaler + 1)) / hz);
}

uint16_t timerGetPrescalerByDesiredHertz(TIM_TypeDef *tim, uint32_t hz)
{
    // protection here for desired hertz > SystemCoreClock???
    if (hz > timerClock(tim)) {
        return 0;
    }
    return (uint16_t)((timerClock(tim) + hz / 2) / hz) - 1;
}

HAL_StatusTypeDef TIM_DMACmd(TIM_HandleTypeDef *htim, uint32_t Channel, FunctionalState NewState)
{
    switch (Channel) {
    case TIM_CHANNEL_1: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 1 DMA request */
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);
        } else {
            /* Disable the TIM Capture/Compare 1 DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
        }
    }
        break;

    case TIM_CHANNEL_2: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 2 DMA request */
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC2);
        } else {
            /* Disable the TIM Capture/Compare 2 DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
        }
    }
        break;

    case TIM_CHANNEL_3: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 3 DMA request */
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC3);
        } else {
            /* Disable the TIM Capture/Compare 3 DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
        }
    }
        break;

    case TIM_CHANNEL_4: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 4 DMA request */
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);
        } else {
            /* Disable the TIM Capture/Compare 4 DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
        }
    }
        break;

    default:
        break;
    }
    /* Change the htim state */
    htim->State = HAL_TIM_STATE_READY;
    /* Return function status */
    return HAL_OK;
}

HAL_StatusTypeDef DMA_SetCurrDataCounter(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
    if ((htim->State == HAL_TIM_STATE_BUSY)) {
        return HAL_BUSY;
    } else if ((htim->State == HAL_TIM_STATE_READY)) {
        if (((uint32_t) pData == 0) && (Length > 0)) {
            return HAL_ERROR;
        } else {
            htim->State = HAL_TIM_STATE_BUSY;
        }
    }
    switch (Channel) {
    case TIM_CHANNEL_1: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = HAL_TIM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = HAL_TIM_DMAError;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t) pData, (uint32_t) & htim->Instance->CCR1, Length);
    }
        break;

    case TIM_CHANNEL_2: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = HAL_TIM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC2]->XferErrorCallback = HAL_TIM_DMAError;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC2], (uint32_t) pData, (uint32_t) & htim->Instance->CCR2, Length);
    }
        break;

    case TIM_CHANNEL_3: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = HAL_TIM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC3]->XferErrorCallback = HAL_TIM_DMAError;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC3], (uint32_t) pData, (uint32_t) & htim->Instance->CCR3, Length);
    }
        break;

    case TIM_CHANNEL_4: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = HAL_TIM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC4]->XferErrorCallback = HAL_TIM_DMAError;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC4], (uint32_t) pData, (uint32_t) & htim->Instance->CCR4, Length);
    }
        break;

    default:
        break;
    }
    /* Return function status */
    return HAL_OK;
}
#endif
