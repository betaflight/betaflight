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

#include "drivers/rcc.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"

#define TIM_N(n) (1 << (n))

/*
    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM1 2 channels
    TIM2 4 channels
    TIM3 4 channels
    TIM4 4 channels
*/

/// TODO: DAL in a lot af calls lookupTimerIndex is used. Instead of passing the timer instance the index should be passed.
#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)

#define TIM_IT_CCx(ch) (TMR_IT_CC1 << ((ch) / 4))

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
    TMR_HandleTypeDef Handle;
} timerHandle_t;
timerHandle_t timerHandle[USED_TIMER_COUNT + 1];

// return index of timer in timer table. Lowest timer has index 0
#define TIMER_INDEX(i) BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

static uint8_t lookupTimerIndex(const TMR_TypeDef *tim)
{
#define _CASE_SHF 10           // amount we can safely shift timer address to the right. gcc will throw error if some timers overlap
#define _CASE_(tim, index) case ((unsigned)tim >> _CASE_SHF): return index; break
#define _CASE(i) _CASE_(TMR##i##_BASE, TIMER_INDEX(i))

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

TMR_TypeDef * const usedTimers[USED_TIMER_COUNT] = {
#define _DEF(i) TMR##i

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

int8_t timerGetTIMNumber(const TMR_TypeDef *tim)
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

rccPeriphTag_t timerRCC(const TMR_TypeDef *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].rcc;
        }
    }
    return 0;
}

uint8_t timerInputIrq(const TMR_TypeDef *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].inputIrq;
        }
    }
    return 0;
}

static void timerNVICConfigure(uint8_t irq)
{
    DAL_NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
    DAL_NVIC_EnableIRQ(irq);
}

TMR_HandleTypeDef* timerFindTimerHandle(TMR_TypeDef *tim)
{
    uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex >= USED_TIMER_COUNT)
        return NULL;

    return &timerHandle[timerIndex].Handle;
}

void timerReconfigureTimeBase(TMR_TypeDef *tim, uint16_t period, uint32_t hz)
{
    TMR_HandleTypeDef* handle = timerFindTimerHandle(tim);
    if (handle == NULL) return;

    handle->Init.Period = (period - 1) & 0xffff; // AKA TIMx_ARR
    handle->Init.Prescaler = (timerClock(tim) / hz) - 1;

    TMR_Base_SetConfig(handle->Instance, &handle->Init);
}

void configTimeBase(TMR_TypeDef *tim, uint16_t period, uint32_t hz)
{
    TMR_HandleTypeDef* handle = timerFindTimerHandle(tim);
    if (handle == NULL) return;

    if (handle->Instance == tim) {
        // already configured
        return;
    }

    handle->Instance = tim;

    handle->Init.Period = (period - 1) & 0xffff; // AKA TIMx_ARR
    handle->Init.Prescaler = (timerClock(tim) / hz) - 1;

    handle->Init.ClockDivision = TMR_CLOCKDIVISION_DIV1;
    handle->Init.CounterMode = TMR_COUNTERMODE_UP;
    handle->Init.RepetitionCounter = 0x0000;

    DAL_TMR_Base_Init(handle);
    if (tim == TMR1 || tim == TMR2 || tim == TMR3 || tim == TMR4 || tim == TMR5 || tim == TMR8 || tim == TMR9)
    {
        TMR_ClockConfigTypeDef sClockSourceConfig;
        memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
        sClockSourceConfig.ClockSource = TMR_CLOCKSOURCE_INTERNAL;
        if (DAL_TMR_ConfigClockSource(handle, &sClockSourceConfig) != DAL_OK) {
            return;
        }
    }
    if (tim == TMR1 || tim == TMR2 || tim == TMR3 || tim == TMR4 || tim == TMR5 || tim == TMR8) {
        TMR_MasterConfigTypeDef sMasterConfig;
        memset(&sMasterConfig, 0, sizeof(sMasterConfig));
        sMasterConfig.MasterSlaveMode = TMR_MASTERSLAVEMODE_DISABLE;
        if (DAL_TMREx_MasterConfigSynchronization(handle, &sMasterConfig) != DAL_OK) {
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
    DAL_TMR_Base_Start(&timerHandle[timerIndex].Handle);

    uint8_t irq = timerInputIrq(timerHardwarePtr->tim);
    timerNVICConfigure(irq);
    // HACK - enable second IRQ on timers that need it
    switch (irq) {

    case TMR1_CC_IRQn:
        timerNVICConfigure(TMR1_UP_TMR10_IRQn);
        break;
    case TMR8_CC_IRQn:
        timerNVICConfigure(TMR8_UP_TMR13_IRQn);
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
        DAL_TMR_Base_Start(&timerHandle[timerIndex].Handle);

        DAL_NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));
        DAL_NVIC_EnableIRQ(irq);

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
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, const TMR_TypeDef *tim)
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
        __DAL_TMR_ENABLE_IT(&timerHandle[timerIndex].Handle, TMR_IT_UPDATE);
    else
        __DAL_TMR_DISABLE_IT(&timerHandle[timerIndex].Handle, TMR_IT_UPDATE);
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
        __DAL_TMR_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    // enable channel IRQ
    if (edgeCallback)
        __DAL_TMR_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

void timerConfigUpdateCallback(const TMR_TypeDef *tim, timerOvrHandlerRec_t *updateCallback)
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
    uint16_t chLo = timHw->channel & ~TMR_CHANNEL_2;   // lower channel
    uint16_t chHi = timHw->channel | TMR_CHANNEL_2;    // upper channel
    uint8_t channelIndex = lookupChannelIndex(chLo);   // get index of lower channel

    if (edgeCallbackLo == NULL)   // disable irq before changing setting callback to NULL
        __DAL_TMR_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chLo));
    if (edgeCallbackHi == NULL)   // disable irq before changing setting callback to NULL
        __DAL_TMR_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chHi));

    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallbackLo;
    timerConfig[timerIndex].edgeCallback[channelIndex + 1] = edgeCallbackHi;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex + 1] = NULL;

    // enable channel IRQs
    if (edgeCallbackLo) {
        __DAL_TMR_CLEAR_FLAG(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chLo));
        __DAL_TMR_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chLo));
    }
    if (edgeCallbackHi) {
        __DAL_TMR_CLEAR_FLAG(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chHi));
        __DAL_TMR_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(chHi));
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
        __DAL_TMR_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel & ~TMR_CHANNEL_2));
    else
        __DAL_TMR_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel & ~TMR_CHANNEL_2));
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
        __DAL_TMR_ENABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
    else
        __DAL_TMR_DISABLE_IT(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
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

    __DAL_TMR_CLEAR_FLAG(&timerHandle[timerIndex].Handle, TIM_IT_CCx(timHw->channel));
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

    TMR_IC_InitTypeDef TIM_ICInitStructure;

    TIM_ICInitStructure.ICPolarity = polarityRising ? TMR_ICPOLARITY_RISING : TMR_ICPOLARITY_FALLING;
    TIM_ICInitStructure.ICSelection = TMR_ICSELECTION_DIRECTTI;
    TIM_ICInitStructure.ICPrescaler = TMR_ICPSC_DIV1;
    TIM_ICInitStructure.ICFilter = getFilter(inputFilterTicks);
    DAL_TMR_IC_ConfigChannel(&timerHandle[timer].Handle, &TIM_ICInitStructure, timHw->channel);
}

// configure dual channel input channel for capture
// polarity is for Low channel (capture order is always Lo - Hi)
void timerChConfigICDual(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;

    TMR_IC_InitTypeDef TIM_ICInitStructure;
    bool directRising = (timHw->channel & TMR_CHANNEL_2) ? !polarityRising : polarityRising;

    // configure direct channel
    TIM_ICInitStructure.ICPolarity = directRising ? TMR_ICPOLARITY_RISING : TMR_ICPOLARITY_FALLING;
    TIM_ICInitStructure.ICSelection = TMR_ICSELECTION_DIRECTTI;
    TIM_ICInitStructure.ICPrescaler = TMR_ICPSC_DIV1;
    TIM_ICInitStructure.ICFilter = getFilter(inputFilterTicks);
    DAL_TMR_IC_ConfigChannel(&timerHandle[timer].Handle, &TIM_ICInitStructure, timHw->channel);

    // configure indirect channel
    TIM_ICInitStructure.ICPolarity = directRising ? TMR_ICPOLARITY_FALLING : TMR_ICPOLARITY_RISING;
    TIM_ICInitStructure.ICSelection = TMR_ICSELECTION_INDIRECTTI;
    DAL_TMR_IC_ConfigChannel(&timerHandle[timer].Handle, &TIM_ICInitStructure, timHw->channel ^ TMR_CHANNEL_2);
}

void timerChICPolarity(const timerHardware_t *timHw, bool polarityRising)
{
    timCCER_t tmpccer = timHw->tim->CCEN;
    tmpccer &= ~(TMR_CCEN_CC1POL << timHw->channel);
    tmpccer |= polarityRising ? (TMR_ICPOLARITY_RISING << timHw->channel) : (TMR_ICPOLARITY_FALLING << timHw->channel);
    timHw->tim->CCEN = tmpccer;
}

volatile timCCR_t* timerChCCRHi(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CC1 + (timHw->channel | TMR_CHANNEL_2));
}

volatile timCCR_t* timerChCCRLo(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CC1 + (timHw->channel & ~TMR_CHANNEL_2));
}

volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CC1 + timHw->channel);
}

void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool stateHigh)
{
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;

    TMR_OC_InitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TMR_OCMODE_INACTIVE;
    TIM_OCInitStructure.Pulse = 0x00000000;
    TIM_OCInitStructure.OCPolarity = stateHigh ? TMR_OCPOLARITY_HIGH : TMR_OCPOLARITY_LOW;
    TIM_OCInitStructure.OCNPolarity = TMR_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCIdleState = TMR_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCNIdleState = TMR_OCNIDLESTATE_RESET;

    DAL_TMR_OC_ConfigChannel(&timerHandle[timer].Handle, &TIM_OCInitStructure, timHw->channel);

    if (outEnable) {
        TIM_OCInitStructure.OCMode = TMR_OCMODE_INACTIVE;
        DAL_TMR_OC_ConfigChannel(&timerHandle[timer].Handle, &TIM_OCInitStructure, timHw->channel);
        DAL_TMR_OC_Start(&timerHandle[timer].Handle, timHw->channel);
    } else {
        TIM_OCInitStructure.OCMode = TMR_OCMODE_TIMING;
        DAL_TMR_OC_ConfigChannel(&timerHandle[timer].Handle, &TIM_OCInitStructure, timHw->channel);
        DAL_TMR_OC_Start_IT(&timerHandle[timer].Handle, timHw->channel);
    }
}

static void timCCxHandler(TMR_TypeDef *tim, timerConfig_t *timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim->STS & tim->DIEN;
#if 1
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->STS = mask;
        tim_status &= mask;
        switch (bit) {
        case __builtin_clz(TMR_IT_UPDATE): {

            if (timerConfig->forcedOverflowTimerValue != 0) {
                capture = timerConfig->forcedOverflowTimerValue - 1;
                timerConfig->forcedOverflowTimerValue = 0;
            } else {
                capture = tim->AUTORLD;
            }

            timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
            while (cb) {
                cb->fn(cb, capture);
                cb = cb->next;
            }
            break;
        }
        case __builtin_clz(TMR_IT_CC1):
            if (timerConfig->edgeCallback[0]) {
                timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CC1);
            }
            break;
        case __builtin_clz(TMR_IT_CC2):
            if (timerConfig->edgeCallback[1]) {
                timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CC2);
            }
            break;
        case __builtin_clz(TMR_IT_CC3):
            if (timerConfig->edgeCallback[2]) {
                timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CC3);
            }
            break;
        case __builtin_clz(TMR_IT_CC4):
            if (timerConfig->edgeCallback[3]) {
                timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CC4);
            }
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

static inline void timUpdateHandler(TMR_TypeDef *tim, timerConfig_t *timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim->STS & tim->DIEN;
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->STS = mask;
        tim_status &= mask;
        switch (bit) {
        case __builtin_clz(TMR_IT_UPDATE): {

            if (timerConfig->forcedOverflowTimerValue != 0) {
                capture = timerConfig->forcedOverflowTimerValue - 1;
                timerConfig->forcedOverflowTimerValue = 0;
            } else {
                capture = tim->AUTORLD;
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
        timCCxHandler(TMR ## i, &timerConfig[TIMER_INDEX(i)]);          \
        timCCxHandler(TMR ## j, &timerConfig[TIMER_INDEX(j)]);          \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        timCCxHandler(TMR ## i, &timerConfig[TIMER_INDEX(i)]);          \
    } struct dummy

#define _TIM_IRQ_HANDLER_UPDATE_ONLY(name, i)                           \
    void name(void)                                                     \
    {                                                                   \
        timUpdateHandler(TMR ## i, &timerConfig[TIMER_INDEX(i)]);       \
    } struct dummy

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TMR1_CC_IRQHandler, 1);
#    if USED_TIMERS & TIM_N(10)
_TIM_IRQ_HANDLER2(TMR1_UP_TMR10_IRQHandler, 1, 10);  // both timers are in use
#    else
_TIM_IRQ_HANDLER(TMR1_UP_TMR10_IRQHandler, 1);     // timer10 is not used
#    endif
#endif

#if USED_TIMERS & TIM_N(2)
_TIM_IRQ_HANDLER(TMR2_IRQHandler, 2);
#endif
#if USED_TIMERS & TIM_N(3)
_TIM_IRQ_HANDLER(TMR3_IRQHandler, 3);
#endif
#if USED_TIMERS & TIM_N(4)
_TIM_IRQ_HANDLER(TMR4_IRQHandler, 4);
#endif
#if USED_TIMERS & TIM_N(5)
_TIM_IRQ_HANDLER(TMR5_IRQHandler, 5);
#endif

#if USED_TIMERS & TIM_N(6)
#  if !(defined(USE_PID_AUDIO))
//not for APM32F4
//_TIM_IRQ_HANDLER_UPDATE_ONLY(TMR6_DAC_IRQHandler, 6);
#  endif
#endif
#if USED_TIMERS & TIM_N(7)
// The USB VCP_DAL driver conflicts with TIM7, see TIMx_IRQHandler in usbd_cdc_interface.h
#  if !(defined(USE_VCP) && (defined(APM32F4)))
_TIM_IRQ_HANDLER_UPDATE_ONLY(TIM7_IRQHandler, 7);
#  endif
#endif

#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TMR8_CC_IRQHandler, 8);

#  if USED_TIMERS & TIM_N(13)
_TIM_IRQ_HANDLER2(TMR8_UP_TMR13_IRQHandler, 8, 13);  // both timers are in use
#  else
_TIM_IRQ_HANDLER(TMR8_UP_TMR13_IRQHandler, 8);     // timer13 is not used
#  endif
#endif

#if USED_TIMERS & TIM_N(9)
_TIM_IRQ_HANDLER(TMR1_BRK_TMR9_IRQHandler, 9);
#endif
#  if USED_TIMERS & TIM_N(11)
_TIM_IRQ_HANDLER(TMR1_TRG_COM_TMR11_IRQHandler, 11);
#  endif
#if USED_TIMERS & TIM_N(12)
_TIM_IRQ_HANDLER(TMR8_BRK_TMR12_IRQHandler, 12);
#endif
#if USED_TIMERS & TIM_N(14)
_TIM_IRQ_HANDLER(TMR8_TRG_COM_TMR14_IRQHandler, 14);
#endif
#if USED_TIMERS & TIM_N(15)
_TIM_IRQ_HANDLER(TMR1_BRK_TMR15_IRQHandler, 15);
#endif
#if USED_TIMERS & TIM_N(17)
_TIM_IRQ_HANDLER(TMR1_TRG_COM_TMR17_IRQHandler, 17);
#endif

void timerInit(void)
{
    memset(timerConfig, 0, sizeof(timerConfig));

#if USED_TIMERS & TIM_N(1)
    __DAL_RCM_TMR1_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(2)
    __DAL_RCM_TMR2_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(3)
    __DAL_RCM_TMR3_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(4)
    __DAL_RCM_TMR4_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(5)
    __DAL_RCM_TMR5_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(6)
    __DAL_RCM_TMR6_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(7)
    __DAL_RCM_TMR7_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(8)
    __DAL_RCM_TMR8_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(9)
    __DAL_RCM_TMR9_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(10)
    __DAL_RCM_TMR10_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(11)
    __DAL_RCM_TMR11_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(12)
    __DAL_RCM_TMR12_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(13)
    __DAL_RCM_TMR13_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(14)
    __DAL_RCM_TMR14_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(15)
    __DAL_RCM_TMR15_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(16)
    __DAL_RCM_TMR16_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(17)
    __DAL_RCM_TMR17_CLK_ENABLE();
#endif
#if USED_TIMERS & TIM_N(20)
    __DAL_RCM_TMR20_CLK_ENABLE();
#endif

    /* enable the timer peripherals */
    for (int i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        RCC_ClockCmd(timerRCC(TIMER_HARDWARE[i].tim), ENABLE);
    }

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

void timerStart(TMR_TypeDef *tim)
{
    TMR_HandleTypeDef* handle = timerFindTimerHandle(tim);
    if (handle == NULL) return;

    __DAL_TMR_ENABLE(handle);
}

/**
 * Force an overflow for a given timer.
 * Saves the current value of the counter in the relevant timerConfig's forcedOverflowTimerValue variable.
 * @param TMR_Typedef *tim The timer to overflow
 * @return void
 **/
void timerForceOverflow(TMR_TypeDef *tim)
{
    uint8_t timerIndex = lookupTimerIndex((const TMR_TypeDef *)tim);

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        // Save the current count so that PPM reading will work on the same timer that was forced to overflow
        timerConfig[timerIndex].forcedOverflowTimerValue = tim->CNT + 1;

        // Force an overflow by setting the UG bit
        tim->CEG |= TMR_CEG_UEG;
    }
}

// DMA_Handle_index --TODO STD对应文件无该函数
uint16_t timerDmaIndex(uint8_t channel)
{
    switch (channel) {
    case TMR_CHANNEL_1:
        return TMR_DMA_ID_CC1;
    case TMR_CHANNEL_2:
        return TMR_DMA_ID_CC2;
    case TMR_CHANNEL_3:
        return TMR_DMA_ID_CC3;
    case TMR_CHANNEL_4:
        return TMR_DMA_ID_CC4;
    }
    return 0;
}

// TIM_DMA_sources
uint16_t timerDmaSource(uint8_t channel)
{
    switch (channel) {
    case TMR_CHANNEL_1:
        return TMR_DMA_CC1;
    case TMR_CHANNEL_2:
        return TMR_DMA_CC2;
    case TMR_CHANNEL_3:
        return TMR_DMA_CC3;
    case TMR_CHANNEL_4:
        return TMR_DMA_CC4;
    }
    return 0;
}

uint16_t timerGetPrescalerByDesiredMhz(TMR_TypeDef *tim, uint16_t mhz)
{
    return timerGetPrescalerByDesiredHertz(tim, MHZ_TO_HZ(mhz));
}

uint16_t timerGetPeriodByPrescaler(TMR_TypeDef *tim, uint16_t prescaler, uint32_t hz)
{
    return (uint16_t)((timerClock(tim) / (prescaler + 1)) / hz);
}

uint16_t timerGetPrescalerByDesiredHertz(TMR_TypeDef *tim, uint32_t hz)
{
    // protection here for desired hertz > SystemCoreClock???
    if (hz > timerClock(tim)) {
        return 0;
    }
    return (uint16_t)((timerClock(tim) + hz / 2) / hz) - 1;
}

DAL_StatusTypeDef TIM_DMACmd(TMR_HandleTypeDef *htim, uint32_t Channel, FunctionalState NewState)
{
    switch (Channel) {
    case TMR_CHANNEL_1: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 1 DMA request */
            __DAL_TMR_ENABLE_DMA(htim, TMR_DMA_CC1);
        } else {
            /* Disable the TIM Capture/Compare 1 DMA request */
            __DAL_TMR_DISABLE_DMA(htim, TMR_DMA_CC1);
        }
    }
        break;

    case TMR_CHANNEL_2: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 2 DMA request */
            __DAL_TMR_ENABLE_DMA(htim, TMR_DMA_CC2);
        } else {
            /* Disable the TIM Capture/Compare 2 DMA request */
            __DAL_TMR_DISABLE_DMA(htim, TMR_DMA_CC2);
        }
    }
        break;

    case TMR_CHANNEL_3: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 3 DMA request */
            __DAL_TMR_ENABLE_DMA(htim, TMR_DMA_CC3);
        } else {
            /* Disable the TIM Capture/Compare 3 DMA request */
            __DAL_TMR_DISABLE_DMA(htim, TMR_DMA_CC3);
        }
    }
        break;

    case TMR_CHANNEL_4: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 4 DMA request */
            __DAL_TMR_ENABLE_DMA(htim, TMR_DMA_CC4);
        } else {
            /* Disable the TIM Capture/Compare 4 DMA request */
            __DAL_TMR_DISABLE_DMA(htim, TMR_DMA_CC4);
        }
    }
        break;

    default:
        break;
    }
    /* Change the htim state */
    htim->State = DAL_TMR_STATE_READY;
    /* Return function status */
    return DAL_OK;
}

DAL_StatusTypeDef DMA_SetCurrDataCounter(TMR_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
    if ((htim->State == DAL_TMR_STATE_BUSY)) {
        return DAL_BUSY;
    } else if ((htim->State == DAL_TMR_STATE_READY)) {
        if (((uint32_t) pData == 0) && (Length > 0)) {
            return DAL_ERROR;
        } else {
            htim->State = DAL_TMR_STATE_BUSY;
        }
    }
    switch (Channel) {
    case TMR_CHANNEL_1: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAError;

        /* Enable the DMA Stream */
        DAL_DMA_Start_IT(htim->hdma[TMR_DMA_ID_CC1], (uint32_t) pData, (uint32_t) & htim->Instance->CC1, Length);
    }
        break;

    case TMR_CHANNEL_2: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TMR_DMA_ID_CC2]->XferCpltCallback = TMR_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TMR_DMA_ID_CC2]->XferErrorCallback = TMR_DMAError;

        /* Enable the DMA Stream */
        DAL_DMA_Start_IT(htim->hdma[TMR_DMA_ID_CC2], (uint32_t) pData, (uint32_t) & htim->Instance->CC2, Length);
    }
        break;

    case TMR_CHANNEL_3: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TMR_DMA_ID_CC3]->XferCpltCallback = TMR_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TMR_DMA_ID_CC3]->XferErrorCallback = TMR_DMAError;

        /* Enable the DMA Stream */
        DAL_DMA_Start_IT(htim->hdma[TMR_DMA_ID_CC3], (uint32_t) pData, (uint32_t) & htim->Instance->CC3, Length);
    }
        break;

    case TMR_CHANNEL_4: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TMR_DMA_ID_CC4]->XferCpltCallback = TMR_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TMR_DMA_ID_CC4]->XferErrorCallback = TMR_DMAError;

        /* Enable the DMA Stream */
        DAL_DMA_Start_IT(htim->hdma[TMR_DMA_ID_CC4], (uint32_t) pData, (uint32_t) & htim->Instance->CC4, Length);
    }
        break;

    default:
        break;
    }
    /* Return function status */
    return DAL_OK;
}
#endif
