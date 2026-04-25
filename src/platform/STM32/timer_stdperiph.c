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
#include "drivers/system.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"
#include "platform/timer.h"

#include "platform/rcc.h"

#define TIM_N(n) (1 << (n))

/*
    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM1 2 channels
    TIM2 4 channels
    TIM3 4 channels
    TIM4 4 channels
*/

#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)
#define CC_CHANNELS_PER_TIMER 4              // TIM_Channel_1..4

#define TIM_IT_CCx(ch) (TIM_IT_CC1 << ((ch) / 4))

typedef struct timerConfig_s {
    // per-timer
    timerOvrHandlerRec_t *updateCallback;

    // per-channel
    timerEdgeHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallback[CC_CHANNELS_PER_TIMER];

    // state
    timerOvrHandlerRec_t *overflowCallbackActive; // null-terminated linked list of active overflow callbacks
    uint32_t forcedOverflowTimerValue;
} timerConfig_t;

timerConfig_t timerConfig[USED_TIMER_COUNT];

typedef struct {
    channelType_t type;
} timerChannelInfo_t;

timerChannelInfo_t timerChannelInfo[TIMER_CHANNEL_COUNT];

typedef struct {
    uint8_t priority;
} timerInfo_t;
timerInfo_t timerInfo[USED_TIMER_COUNT];

// return index of timer in timer table. Lowest timer has index 0
#define TIMER_INDEX(i) BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

static uint8_t lookupTimerIndex(const timerResource_t *tim)
{
    const TIM_TypeDef *tim_ptr = (const TIM_TypeDef *)tim;
#define _CASE_SHF 10           // amount we can safely shift timer address to the right. gcc will throw error if some timers overlap
#define _CASE_(tim, index) case ((unsigned)tim >> _CASE_SHF): return index; break
#define _CASE(i) _CASE_(TIM##i##_BASE, TIMER_INDEX(i))

// let gcc do the work, switch should be quite optimized
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

int8_t timerGetIndexByNumber(uint8_t number)
{
    return TIM_N(number) & USED_TIMERS ? popcount((TIM_N(number) - 1) & USED_TIMERS) : -1;
}

int8_t timerGetTIMNumber(const timerHardware_t *timHw)
{
    const uint8_t index = lookupTimerIndex(timHw->tim);

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
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void timerReconfigureTimeBase(const timerHardware_t *timHw, uint16_t period, uint32_t hz)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)timHw->tim;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xFFFF; // AKA TIMx_ARR

    // "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
    // Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
    TIM_TimeBaseStructure.TIM_Prescaler = (timerClock(timHw) / hz) - 1;

    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim_ptr, &TIM_TimeBaseStructure);
}

// old interface for PWM inputs. It should be replaced
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint32_t hz)
{
    timerReconfigureTimeBase(timerHardwarePtr, period, hz);
    TIM_Cmd((TIM_TypeDef *)timerHardwarePtr->tim, ENABLE);

    uint8_t irq = timerInputInterrupt(timerHardwarePtr);
    timerNVICConfigure(irq);
    // HACK - enable second IRQ on timers that need it
    switch (irq) {
#if defined(STM32F40_41xxx) || defined(STM32F411xE)
    case TIM1_CC_IRQn:
        timerNVICConfigure(TIM1_UP_TIM10_IRQn);
        break;
#endif
#if defined(STM32F40_41xxx)
    case TIM8_CC_IRQn:
        timerNVICConfigure(TIM8_UP_TIM13_IRQn);
        break;
#endif
    }
}

// allocate and configure timer channel. Timer priority is set to highest priority of its channels
void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq)
{
    unsigned channel = timHw - TIMER_HARDWARE;
    if (channel >= TIMER_CHANNEL_COUNT) {
        return;
    }

    timerChannelInfo[channel].type = type;
    unsigned timer = lookupTimerIndex(timHw->tim);
    if (timer >= USED_TIMER_COUNT)
        return;
    if (irqPriority < timerInfo[timer].priority) {
        // it would be better to set priority in the end, but current startup sequence is not ready
        timerReconfigureTimeBase(timHw, 0, 1);
        TIM_Cmd(usedTimers[timer],  ENABLE);

        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_InitStructure.NVIC_IRQChannel = irq;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        timerInfo[timer].priority = irqPriority;
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

// update overflow callback list
// some synchronization mechanism is neccesary to avoid disturbing other channels (BASEPRI used now)
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, const timerResource_t *tim)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)tim;
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
    TIM_ITConfig(tim_ptr, TIM_IT_Update, cfg->overflowCallbackActive ? ENABLE : DISABLE);
}

// config edge and overflow callback for channel. Try to avoid per-channel overflowCallback, it is a bit expensive
void timerChannelConfigCallbacks(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint8_t channelIndex = lookupChannelIndex(timHw->channel);
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)timHw->tim;
    if (edgeCallback == NULL)   // disable irq before changing callback to NULL
        TIM_ITConfig(tim_ptr, TIM_IT_CCx(timHw->channel), DISABLE);
    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    // enable channel IRQ
    if (edgeCallback)
        TIM_ITConfig(tim_ptr, TIM_IT_CCx(timHw->channel), ENABLE);

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

// configure callbacks for pair of channels (1+2 or 3+4).
// Hi(2,4) and Lo(1,3) callbacks are specified, it is not important which timHw channel is used.
// This is intended for dual capture mode (each channel handles one transition)
void timerChConfigCallbacksDual(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edgeCallbackLo, timerEdgeHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)timHw->tim;
    uint16_t chLo = timHw->channel & ~TIM_Channel_2;   // lower channel
    uint16_t chHi = timHw->channel | TIM_Channel_2;    // upper channel
    uint8_t channelIndex = lookupChannelIndex(chLo);   // get index of lower channel

    if (edgeCallbackLo == NULL)   // disable irq before changing setting callback to NULL
        TIM_ITConfig(tim_ptr, TIM_IT_CCx(chLo), DISABLE);
    if (edgeCallbackHi == NULL)   // disable irq before changing setting callback to NULL
        TIM_ITConfig(tim_ptr, TIM_IT_CCx(chHi), DISABLE);

    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallbackLo;
    timerConfig[timerIndex].edgeCallback[channelIndex + 1] = edgeCallbackHi;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex + 1] = NULL;

    // enable channel IRQs
    if (edgeCallbackLo) {
        TIM_ClearFlag(tim_ptr, TIM_IT_CCx(chLo));
        TIM_ITConfig(tim_ptr, TIM_IT_CCx(chLo), ENABLE);
    }
    if (edgeCallbackHi) {
        TIM_ClearFlag(tim_ptr, TIM_IT_CCx(chHi));
        TIM_ITConfig(tim_ptr, TIM_IT_CCx(chHi), ENABLE);
    }

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

// enable/disable IRQ for low channel in dual configuration
void timerChannelConfigInterruptDualLo(const timerHardware_t *timHw, FunctionalState newState)
{
    TIM_ITConfig((TIM_TypeDef *)timHw->tim, TIM_IT_CCx(timHw->channel&~TIM_Channel_2), newState);
}

// enable or disable IRQ
void timerChannelConfigInterrupt(const timerHardware_t *timHw, FunctionalState newState)
{
    TIM_ITConfig((TIM_TypeDef *)timHw->tim, TIM_IT_CCx(timHw->channel), newState);
}

// clear Compare/Capture flag for channel
void timerChannelClearFlag(const timerHardware_t *timHw)
{
    TIM_ClearFlag((TIM_TypeDef *)timHw->tim, TIM_IT_CCx(timHw->channel));
}

// configure timer channel GPIO mode
void timerChannelConfigGPIO(const timerHardware_t* timHw, ioConfig_t mode)
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

// Configure input capture
void timerChannelConfigInput(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = timHw->channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarityRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);

    TIM_ICInit((TIM_TypeDef *)timHw->tim, &TIM_ICInitStructure);
}

// configure dual channel input channel for capture
// polarity is for Low channel (capture order is always Lo - Hi)
void timerChannelConfigInputDual(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;
    bool directRising = (timHw->channel & TIM_Channel_2) ? !polarityRising : polarityRising;
    // configure direct channel
    TIM_ICStructInit(&TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = timHw->channel;
    TIM_ICInitStructure.TIM_ICPolarity = directRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);
    TIM_ICInit((TIM_TypeDef *)timHw->tim, &TIM_ICInitStructure);
    // configure indirect channel
    TIM_ICInitStructure.TIM_Channel = timHw->channel ^ TIM_Channel_2;   // get opposite channel no
    TIM_ICInitStructure.TIM_ICPolarity = directRising ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
    TIM_ICInit((TIM_TypeDef *)timHw->tim, &TIM_ICInitStructure);
}

void timerChannelInputPolarity(const timerHardware_t *timHw, bool polarityRising)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    timCCER_t tmpccer = tim->CCER;
    tmpccer &= ~(TIM_CCER_CC1P << timHw->channel);
    tmpccer |= polarityRising ? (TIM_ICPolarity_Rising << timHw->channel) : (TIM_ICPolarity_Falling << timHw->channel);
    tim->CCER = tmpccer;
}

volatile timCCR_t* timerChCCRHi(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&((TIM_TypeDef *)timHw->tim)->CCR1 + (timHw->channel | TIM_Channel_2));
}

volatile timCCR_t* timerChCCRLo(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&((TIM_TypeDef *)timHw->tim)->CCR1 + (timHw->channel & ~TIM_Channel_2));
}

volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&((TIM_TypeDef *)timHw->tim)->CCR1 + timHw->channel);
}

void timerChannelConfigOutput(const timerHardware_t* timHw, bool outEnable, bool stateHigh)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)timHw->tim;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    if (outEnable) {
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        if (timHw->output & TIMER_OUTPUT_INVERTED) {
            stateHigh = !stateHigh;
        }
        TIM_OCInitStructure.TIM_OCPolarity = stateHigh ? TIM_OCPolarity_High : TIM_OCPolarity_Low;
    } else {
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    }

    switch (timHw->channel) {
    case TIM_Channel_1:
        TIM_OC1Init(tim_ptr, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(tim_ptr, TIM_OCPreload_Disable);
        break;
    case TIM_Channel_2:
        TIM_OC2Init(tim_ptr, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(tim_ptr, TIM_OCPreload_Disable);
        break;
    case TIM_Channel_3:
        TIM_OC3Init(tim_ptr, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(tim_ptr, TIM_OCPreload_Disable);
        break;
    case TIM_Channel_4:
        TIM_OC4Init(tim_ptr, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(tim_ptr, TIM_OCPreload_Disable);
        break;
    }
}

static void timCCxHandler(timerResource_t *tim, timerConfig_t *timerConfig)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)tim;
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim_ptr->SR & tim_ptr->DIER;
#if 1
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // current order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim_ptr->SR = mask;
        tim_status &= mask;
        switch (bit) {
            case __builtin_clz(TIM_IT_Update): {

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
                if (timerConfig->edgeCallback[0] && timerConfig->edgeCallback[0]->fn) {
                    timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim_ptr->CCR1);
                }
                break;
            case __builtin_clz(TIM_IT_CC2):
                if (timerConfig->edgeCallback[1] && timerConfig->edgeCallback[1]->fn) {
                    timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim_ptr->CCR2);
                }
                break;
            case __builtin_clz(TIM_IT_CC3):
                if (timerConfig->edgeCallback[2] && timerConfig->edgeCallback[2]->fn) {
                    timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim_ptr->CCR3);
                }
                break;
            case __builtin_clz(TIM_IT_CC4):
                if (timerConfig->edgeCallback[3] && timerConfig->edgeCallback[3]->fn) {
                    timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim_ptr->CCR4);
                }
                break;
        }
    }
#else
    if (tim_status & (int)TIM_IT_Update) {
        tim_ptr->SR = ~TIM_IT_Update;
        capture = tim_ptr->ARR;
        timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
        while (cb) {
            cb->fn(cb, capture);
            cb = cb->next;
        }
    }
    if (tim_status & (int)TIM_IT_CC1) {
        tim_ptr->SR = ~TIM_IT_CC1;
        if (timerConfig->edgeCallback[0] && timerConfig->edgeCallback[0]->fn) {
            timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim_ptr->CCR1);
        }
    }
    if (tim_status & (int)TIM_IT_CC2) {
        tim_ptr->SR = ~TIM_IT_CC2;
        if (timerConfig->edgeCallback[1] && timerConfig->edgeCallback[1]->fn) {
            timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim_ptr->CCR2);
        }
    }
    if (tim_status & (int)TIM_IT_CC3) {
        tim_ptr->SR = ~TIM_IT_CC3;
        if (timerConfig->edgeCallback[2] && timerConfig->edgeCallback[2]->fn) {
            timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim_ptr->CCR3);
        }
    }
    if (tim_status & (int)TIM_IT_CC4) {
        tim_ptr->SR = ~TIM_IT_CC4;
        if (timerConfig->edgeCallback[3] && timerConfig->edgeCallback[3]->fn) {
            timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim_ptr->CCR4);
        }
    }
#endif
}

static inline void timUpdateHandler(timerResource_t *tim, timerConfig_t *timerConfig)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)tim;
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim_ptr->SR & tim_ptr->DIER;
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim_ptr->SR = mask;
        tim_status &= mask;
        switch (bit) {
            case __builtin_clz(TIM_IT_Update): {

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

// handler for shared interrupts when both timers need to check status bits
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
# if defined(STM32F40_41xxx) || defined(STM32F411xE)
#  if USED_TIMERS & TIM_N(10)
_TIM_IRQ_HANDLER2(TIM1_UP_TIM10_IRQHandler, 1, 10);  // both timers are in use
#  else
_TIM_IRQ_HANDLER(TIM1_UP_TIM10_IRQHandler, 1);     // timer10 is not used
#  endif
# endif
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
_TIM_IRQ_HANDLER_UPDATE_ONLY(TIM6_IRQHandler, 6);
#  endif
#endif
#if USED_TIMERS & TIM_N(7)
// The USB VCP_HAL driver conflicts with TIM7, see TIMx_IRQHandler in usbd_cdc_interface.h
#  if !(defined(USE_VCP) && (defined(STM32F4) || defined(STM32G4) || defined(STM32H7)))
#    if defined(STM32G4)
_TIM_IRQ_HANDLER_UPDATE_ONLY(TIM7_DAC_IRQHandler, 7);
#    else
_TIM_IRQ_HANDLER_UPDATE_ONLY(TIM7_IRQHandler, 7);
#    endif
#  endif
#endif

#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8);
_TIM_IRQ_HANDLER(TIM8_UP_IRQHandler, 8);
# if defined(STM32F40_41xxx)
#  if USED_TIMERS & TIM_N(13)
_TIM_IRQ_HANDLER2(TIM8_UP_TIM13_IRQHandler, 8, 13);  // both timers are in use
#  else
_TIM_IRQ_HANDLER(TIM8_UP_TIM13_IRQHandler, 8);     // timer13 is not used
#  endif
# endif
# if defined(STM32F411xE)
# endif
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
#if USED_TIMERS & TIM_N(14)
_TIM_IRQ_HANDLER(TIM8_TRG_COM_TIM14_IRQHandler, 14);
#endif
#if USED_TIMERS & TIM_N(15)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM15_IRQHandler, 15);
#endif
#if USED_TIMERS & TIM_N(17)
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM17_IRQHandler, 17);
#endif

void timerInit(void)
{
    memset(timerConfig, 0, sizeof(timerConfig));

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

void timerStart(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)timHw->tim;
    TIM_Cmd(tim_ptr,  ENABLE);
}

/**
 * Force an overflow for a given timer.
 * Saves the current value of the counter in the relevant timerConfig's forcedOverflowTimerValue variable.
 * @param TIM_Typedef *tim The timer to overflow
 * @return void
 **/
void timerForceOverflow(timerResource_t *tim)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)tim;
    uint8_t timerIndex = lookupTimerIndex(tim);

    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        // Save the current count so that PPM reading will work on the same timer that was forced to overflow
        timerConfig[timerIndex].forcedOverflowTimerValue = tim_ptr->CNT + 1;

        // Force an overflow by setting the UG bit
        tim_ptr->EGR |= TIM_EGR_UG;
    }
}

#if !defined(USE_HAL_DRIVER)
void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init)
{
    switch (channel) {
    case TIM_Channel_1:
        TIM_OC1Init(tim, init);
        break;
    case TIM_Channel_2:
        TIM_OC2Init(tim, init);
        break;
    case TIM_Channel_3:
        TIM_OC3Init(tim, init);
        break;
    case TIM_Channel_4:
        TIM_OC4Init(tim, init);
        break;
    }
}

void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t preload)
{
    switch (channel) {
    case TIM_Channel_1:
        TIM_OC1PreloadConfig(tim, preload);
        break;
    case TIM_Channel_2:
        TIM_OC2PreloadConfig(tim, preload);
        break;
    case TIM_Channel_3:
        TIM_OC3PreloadConfig(tim, preload);
        break;
    case TIM_Channel_4:
        TIM_OC4PreloadConfig(tim, preload);
        break;
    }
}
#endif

volatile timCCR_t* timerCCR(timerResource_t *tim, uint8_t channel)
{
    TIM_TypeDef *tim_ptr = (TIM_TypeDef *)tim;
    return (volatile timCCR_t*)((volatile char*)&tim_ptr->CCR1 + channel);
}

#ifndef USE_HAL_DRIVER
uint16_t timerDmaSource(uint8_t channel)
{
    switch (channel) {
    case TIM_Channel_1:
        return TIM_DMA_CC1;
    case TIM_Channel_2:
        return TIM_DMA_CC2;
    case TIM_Channel_3:
        return TIM_DMA_CC3;
    case TIM_Channel_4:
        return TIM_DMA_CC4;
    }
    return 0;
}
#endif

uint16_t timerGetPrescalerByDesiredMhz(timerResource_t *tim, uint16_t mhz)
{
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
    // protection here for desired hertz > SystemCoreClock???
    if (hz == 0 || hz > timerClockFromInstance(tim)) {
        return 0;
    }
    return (uint16_t)((timerClockFromInstance(tim) + hz / 2 ) / hz) - 1;
}

void timerReset(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    TIM_DeInit(tim);
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

void timerDisable(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    TIM_ITConfig(tim, TIM_IT_Update, DISABLE);
    TIM_Cmd(tim, DISABLE);
}

void timerEnable(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    TIM_Cmd(tim, ENABLE);
    TIM_GenerateEvent(tim, TIM_EventSource_Update);
}

void timerEnableInterrupt(const timerHardware_t *timHw)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)timHw->tim;
    TIM_ClearFlag(tim, TIM_FLAG_Update);
    TIM_ITConfig(tim, TIM_IT_Update, ENABLE);
}

uint32_t timerGetPrescaler(const timerHardware_t *timHw)
{
    return ((const TIM_TypeDef *)timHw->tim)->PSC;
}

void *timerFindTimerHandle(timerResource_t *tim)
{
    UNUSED(tim);
    return NULL;
}

#endif
