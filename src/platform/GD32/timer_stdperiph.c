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
#include "platform/rcc.h"
#include "drivers/system.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"

#define TIM_N(n) (1 << (n))

/*
    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIMER0 2 channels
    TIMER1 4 channels
    TIMER2 4 channels
    TIMER3 4 channels
*/

#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)
#define CC_CHANNELS_PER_TIMER 4

#define TIMER_INT_CHx(ch) (TIMER_INT_CH0 << (ch))

typedef struct timerConfig_s {
    timerOvrHandlerRec_t *updateCallback;

    // per-channel
    timerCCHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];
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

static uint8_t lookupTimerIndex(const TIM_TypeDef *tim) // TIM_TypeDef只有
{
#define _CASE_SHF 10           // amount we can safely shift timer address to the right. gcc will throw error if some timers overlap
#define _CASE_(tim, index) case ((unsigned)tim >> _CASE_SHF): return index; break
#define _CASE(i) _CASE_(TIMER##i, TIMER_INDEX(i))

// let gcc do the work, switch should be quite optimized
    switch ((unsigned)tim >> _CASE_SHF) {
#if USED_TIMERS & TIM_N(0)
        _CASE(0);
#endif        
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
    default:  return ~1;  // make sure final index is out of range
    }
#undef _CASE
#undef _CASE_
}

uint32_t usedTimers[USED_TIMER_COUNT] = {
#define _DEF(i) (uint32_t)(TIMER##i)

#if USED_TIMERS & TIM_N(0)
    _DEF(0),
#endif
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
#undef _DEF
};

// Map timer index to timer number (Straight copy of usedTimers array)
const int8_t timerNumbers[USED_TIMER_COUNT] = {
#define _DEF(i) i

#if USED_TIMERS & TIM_N(0)
    _DEF(0),
#endif
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
    const uint8_t index = lookupTimerIndex(tim);

    return timerGetNumberByIndex(index);
}

static inline uint8_t lookupChannelIndex(const uint16_t channel)
{
    return channel;    
}

uint8_t timerLookupChannelIndex(const uint16_t channel)
{
    return lookupChannelIndex(channel);
}

rccPeriphTag_t timerRCC(const TIM_TypeDef *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].rcc;
        }
    }
    return 0;
}

uint8_t timerInputIrq(const TIM_TypeDef *tim)
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
    nvic_irq_enable(irq , NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
}

void timerReconfigureTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz)
{
    configTimeBase(tim, period, hz);
}

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz)
{
    timer_parameter_struct timer_initpara;

    timer_struct_para_init(&timer_initpara);

    timer_initpara.period            = (period - 1) & 0xFFFF;
    timer_initpara.prescaler         = (timerClock(tim) / hz) - 1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.repetitioncounter = 0;
    timer_init((uint32_t)tim, &timer_initpara);
}

// old interface for PWM inputs. It should be replaced
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint32_t hz)
{
    configTimeBase(timerHardwarePtr->tim, period, hz);
    timer_enable((uint32_t)timerHardwarePtr->tim);

    uint8_t irq = timerInputIrq(timerHardwarePtr->tim);
    timerNVICConfigure(irq);

    switch (irq) {
        case TIMER0_Channel_IRQn:
        timerNVICConfigure(TIMER0_UP_TIMER9_IRQn);
        break;

        case TIMER7_Channel_IRQn:
        timerNVICConfigure(TIMER7_UP_TIMER12_IRQn);
        break;
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
        configTimeBase((void *)usedTimers[timer], 0, 1);
        timer_enable(usedTimers[timer]);

        nvic_irq_enable(irq, NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));

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
    if (cfg->overflowCallbackActive) {
        timer_interrupt_enable((uint32_t)tim, TIMER_INT_UP);
    } else {
        timer_interrupt_disable((uint32_t)tim, TIMER_INT_UP);
    }
}

// config edge and overflow callback for channel. Try to avoid per-channel overflowCallback, it is a bit expensive
void timerChConfigCallbacks(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint8_t channelIndex = lookupChannelIndex(timHw->channel);
    if (edgeCallback == NULL)   // disable irq before changing callback to NULL
        timer_interrupt_disable((uint32_t)(timHw->tim), TIMER_INT_CHx(timHw->channel));

    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    // enable channel IRQ
    if (edgeCallback)
        timer_interrupt_enable((uint32_t)(timHw->tim), TIMER_INT_CHx(timHw->channel));

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

    uint16_t chLo = timHw->channel & ~TIMER_CH_1;      // lower channel
    uint16_t chHi = timHw->channel | TIMER_CH_1;       // upper channel

    uint8_t channelIndex = lookupChannelIndex(chLo);   // get index of lower channel

    if (edgeCallbackLo == NULL)   // disable irq before changing setting callback to NULL
        timer_interrupt_disable((uint32_t)(timHw->tim), TIMER_INT_CHx(chLo));

    if (edgeCallbackHi == NULL)   // disable irq before changing setting callback to NULL
        timer_interrupt_disable((uint32_t)(timHw->tim), TIMER_INT_CHx(chHi));

    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallbackLo;
    timerConfig[timerIndex].edgeCallback[channelIndex + 1] = edgeCallbackHi;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex + 1] = NULL;

    // enable channel IRQs
    if (edgeCallbackLo) {
        timer_flag_clear((uint32_t)(timHw->tim), TIMER_INT_CHx(chLo)); 
        timer_interrupt_enable((uint32_t)(timHw->tim), TIMER_INT_CHx(chLo)); 
    }

    if (edgeCallbackHi) {
        timer_flag_clear((uint32_t)(timHw->tim), TIMER_INT_CHx(chHi)); 
        timer_interrupt_enable((uint32_t)(timHw->tim), TIMER_INT_CHx(chHi));        
    }

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

// enable/disable IRQ for low channel in dual configuration
void timerChITConfigDualLo(const timerHardware_t *timHw, FunctionalState newState)
{
    if (newState) {
        timer_interrupt_enable((uint32_t)(timHw->tim), TIMER_INT_CHx(timHw->channel &~ TIMER_CH_1));
    } else {
        timer_interrupt_disable((uint32_t)(timHw->tim), TIMER_INT_CHx(timHw->channel &~ TIMER_CH_1));
    }
}

// enable or disable IRQ
void timerChITConfig(const timerHardware_t *timHw, FunctionalState newState)
{
    if (newState) {
        timer_interrupt_enable((uint32_t)(timHw->tim), TIMER_INT_CHx(timHw->channel));
    } else {
        timer_interrupt_disable((uint32_t)(timHw->tim), TIMER_INT_CHx(timHw->channel));
    }
}

// clear Compare/Capture flag for channel
void timerChClearCCFlag(const timerHardware_t *timHw)
{
    timer_flag_clear((uint32_t)(timHw->tim), TIMER_INT_CHx(timHw->channel));
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

// Configure input capture
void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    timer_ic_parameter_struct timer_icinitpara;
    timer_channel_input_struct_para_init(&timer_icinitpara);
    timer_icinitpara.icpolarity  = polarityRising ? TIMER_IC_POLARITY_RISING : TIMER_IC_POLARITY_FALLING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = getFilter(inputFilterTicks);
    timer_input_capture_config((uint32_t)(timHw->tim), timHw->channel, &timer_icinitpara); 
}

// configure dual channel input channel for capture
// polarity is for Low channel (capture order is always Lo - Hi)
void timerChConfigICDual(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    timer_ic_parameter_struct timer_icinitpara;
    bool directRising = (timHw->channel & TIMER_CH_1) ? !polarityRising : polarityRising;
    // configure direct channel
    timer_channel_input_struct_para_init(&timer_icinitpara);


    timer_icinitpara.icpolarity  = directRising ? TIMER_IC_POLARITY_RISING : TIMER_IC_POLARITY_FALLING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = getFilter(inputFilterTicks);
    timer_input_capture_config((uint32_t)(timHw->tim), timHw->channel, &timer_icinitpara); 
    // configure indirect channel
    timer_icinitpara.icpolarity  = directRising ? TIMER_IC_POLARITY_FALLING : TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_INDIRECTTI;
    timer_input_capture_config((uint32_t)(timHw->tim), (timHw->channel ^ TIMER_CH_1), &timer_icinitpara); // get opposite channel no
}

void timerChICPolarity(const timerHardware_t *timHw, bool polarityRising)
{
    timCCER_t tmpccer = TIMER_CHCTL2((uint32_t)(timHw->tim));
    tmpccer &= ~(TIMER_CHCTL2_CH0P << (timHw->channel << 2));
    tmpccer |= polarityRising ? (TIMER_IC_POLARITY_RISING << (timHw->channel << 2)) : (TIMER_IC_POLARITY_FALLING << (timHw->channel << 2));
    TIMER_CHCTL2((uint32_t)(timHw->tim)) = tmpccer;
}

volatile timCCR_t* timerChCCRHi(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile uint32_t*)((uintptr_t)&TIMER_CH0CV(&(timHw->tim))) + (timHw->channel | TIMER_CH_1));
}

volatile timCCR_t* timerChCCRLo(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile uint32_t*)((uintptr_t)&TIMER_CH0CV(&(timHw->tim))) + (timHw->channel & ~TIMER_CH_1));
}

volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile uint32_t*)((uintptr_t)&TIMER_CH0CV((uint32_t)(timHw->tim))) + (timHw->channel));
}

void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool stateHigh)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_channel_output_struct_para_init(&timer_ocintpara);

    uint16_t ocmode; 
    if (outEnable) {
        ocmode = TIMER_OC_MODE_INACTIVE;
        timer_ocintpara.outputstate = TIMER_CCX_ENABLE;

        if (timHw->output & TIMER_OUTPUT_INVERTED) {
            stateHigh = !stateHigh;
        }
        timer_ocintpara.ocpolarity  = stateHigh ? TIMER_OC_POLARITY_HIGH : TIMER_OC_POLARITY_LOW;
    } else {
        ocmode = TIMER_OC_MODE_TIMING;   
    }

    timer_channel_output_config((uint32_t)(timHw->tim), timHw->channel, &timer_ocintpara);
    timer_channel_output_mode_config((uint32_t)(timHw->tim), timHw->channel, ocmode); 
    timer_channel_output_shadow_config((uint32_t)(timHw->tim), timHw->channel, TIMER_OC_SHADOW_DISABLE);
}


static void timCCxHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
    tim_status = TIMER_INTF((uint32_t)tim) & TIMER_DMAINTEN((uint32_t)tim);

    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // current order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        TIMER_INTF((uint32_t)tim) = mask;
        tim_status &= mask;

        switch (bit) {
            case __builtin_clz(TIMER_INT_UP): {

                if (timerConfig->forcedOverflowTimerValue != 0) {
                    capture = timerConfig->forcedOverflowTimerValue - 1;
                    timerConfig->forcedOverflowTimerValue = 0;
                } else {
                    capture = TIMER_CAR((uint32_t)tim);
                }

                timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
                while (cb) {
                    cb->fn(cb, capture);
                    cb = cb->next;
                }
                break;
            }
            case __builtin_clz(TIMER_INT_CH0):
                if (timerConfig->edgeCallback[0]) {
                    timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], TIMER_CH0CV((uint32_t)tim));
                }
                break;
            case __builtin_clz(TIMER_INT_CH1):
                if (timerConfig->edgeCallback[1]) {
                        timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], TIMER_CH1CV((uint32_t)tim));
                }
                break;
            case __builtin_clz(TIMER_INT_CH2):
                if (timerConfig->edgeCallback[2]) {
                        timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], TIMER_CH2CV((uint32_t)tim));
                }
                break;
            case __builtin_clz(TIMER_INT_CH3):
                if (timerConfig->edgeCallback[3]) {
                        timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], TIMER_CH3CV((uint32_t)tim));
                }
                break;
        }
    }
}

static inline void timUpdateHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
    tim_status = TIMER_INTF((uint32_t)tim) & TIMER_DMAINTEN((uint32_t)tim);
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        TIMER_INTF((uint32_t)tim) = mask;
        tim_status &= mask;
        switch (bit) {
            case __builtin_clz(TIMER_INT_UP): {

                if (timerConfig->forcedOverflowTimerValue != 0) {
                    capture = timerConfig->forcedOverflowTimerValue - 1;
                    timerConfig->forcedOverflowTimerValue = 0;
                } else {
                    capture = TIMER_CAR((uint32_t)tim);
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
#define _TIM_IRQ_HANDLER2(name, i, j)                                               \
    void name(void)                                                                 \
    {                                                                               \
        timCCxHandler((void *)(TIMER ## i), &timerConfig[TIMER_INDEX(i)]);          \
        timCCxHandler((void *)(TIMER ## j), &timerConfig[TIMER_INDEX(j)]);          \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                                   \
    void name(void)                                                                 \
    {                                                                               \
        timCCxHandler((void *)(TIMER ## i), &timerConfig[TIMER_INDEX(i)]);          \
    } struct dummy

#define _TIM_IRQ_HANDLER_UPDATE_ONLY(name, i)                                       \
    void name(void)                                                                 \
    {                                                                               \
        timUpdateHandler((void *)(TIMER ## i), &timerConfig[TIMER_INDEX(i)]);       \
    } struct dummy

#if USED_TIMERS & TIM_N(0)
_TIM_IRQ_HANDLER(TIMER0_Channel_IRQHandler, 0);
#  if USED_TIMERS & TIM_N(9)
_TIM_IRQ_HANDLER2(TIMER0_UP_TIMER9_IRQHandler, 0, 9);  // both timers are in use
#  else
_TIM_IRQ_HANDLER(TIMER0_UP_TIMER9_IRQHandler, 0);      // timer9 is not used
#  endif
#endif

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TIMER1_IRQHandler, 1);
#endif
#if USED_TIMERS & TIM_N(2)
_TIM_IRQ_HANDLER(TIMER2_IRQHandler, 2);
#endif
#if USED_TIMERS & TIM_N(3)
_TIM_IRQ_HANDLER(TIMER3_IRQHandler, 3);
#endif
#if USED_TIMERS & TIM_N(4)
_TIM_IRQ_HANDLER(TIMER4_IRQHandler, 4);
#endif

#if USED_TIMERS & TIM_N(5)
#  if !(defined(USE_PID_AUDIO))
_TIM_IRQ_HANDLER(TIMER5_DAC_IRQHandler, 5);
#  endif
#endif

#if USED_TIMERS & TIM_N(6)
#  if !(defined(USE_VCP) && (defined(GD32F4)))
_TIM_IRQ_HANDLER_UPDATE_ONLY(TIMER6_IRQHandler, 6);
#  endif
#endif

#if USED_TIMERS & TIM_N(7)
_TIM_IRQ_HANDLER(TIMER7_Channel_IRQHandler, 7);

#  if USED_TIMERS & TIM_N(12)
_TIM_IRQ_HANDLER2(TIMER7_UP_TIMER12_IRQHandler, 7, 12);  // both timers are in use
#  else
_TIM_IRQ_HANDLER(TIMER7_UP_TIMER12_IRQHandler, 7);       // timer12 is not used
#  endif
#endif

#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TIMER0_BRK_TIMER8_IRQHandler, 8);
#endif
#if USED_TIMERS & TIM_N(10)
_TIM_IRQ_HANDLER(TIMER0_TRG_CMT_TIMER10_IRQHandler, 10);
#endif
#if USED_TIMERS & TIM_N(11)
_TIM_IRQ_HANDLER(TIMER7_BRK_TIMER11_IRQHandler, 11);
#endif
#if USED_TIMERS & TIM_N(13)
_TIM_IRQ_HANDLER(TIMER7_TRG_CMT_TIMER13_IRQHandler, 13);
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

void timerStart(TIM_TypeDef *tim)
{
    timer_enable((uint32_t)tim);
}

/*!
    \brief      force an overflow for a given timer
    \param[in]  tim: The timer to overflow
    \param[out] none
    \retval     none
*/
void timerForceOverflow(TIM_TypeDef *tim)
{
    uint8_t timerIndex = lookupTimerIndex((const TIM_TypeDef *)tim);

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        // Save the current count so that PPM reading will work on the same timer that was forced to overflow
        timerConfig[timerIndex].forcedOverflowTimerValue = TIMER_CNT((uint32_t)tim) + 1;

        // Force an overflow by setting the UG bit
        TIMER_SWEVG((uint32_t)tim) |= TIMER_SWEVG_UPG;
    }
}

void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init)
{
    timer_channel_output_config((uint32_t)tim, (uint16_t)channel, init);
}

void timerOCModeConfig(void *tim, uint8_t channel, uint16_t ocmode)
{
    timer_channel_output_mode_config((uint32_t)tim, (uint16_t)channel, ocmode);
}

void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t ocshadow)
{
    timer_channel_output_shadow_config((uint32_t)tim, channel, ocshadow);
}

volatile timCCR_t* timerCCR(TIM_TypeDef *tim, uint8_t channel)
{
    return (volatile timCCR_t*)((volatile uint32_t*)(&TIMER_CH0CV((uint32_t)tim)) + (channel));
}

uint16_t timerDmaSource(uint8_t channel)
{
    switch (channel) {
    case TIMER_CH_0:
        return TIMER_DMA_CH0D;
    case TIMER_CH_1:
        return TIMER_DMA_CH1D;
    case TIMER_CH_2:
        return TIMER_DMA_CH2D;
    case TIMER_CH_3:
        return TIMER_DMA_CH3D;
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
    return (uint16_t)((timerClock(tim) + hz / 2 ) / hz) - 1;
}

void timerReset(TIM_TypeDef *timer)
{
    timer_deinit((uint32_t)timer);
}

void timerSetPeriod(TIM_TypeDef *timer, uint32_t period)
{
    TIMER_CAR((uint32_t)timer) = period;
}

uint32_t timerGetPeriod(TIM_TypeDef *timer)
{
    return TIMER_CAR((uint32_t)timer);
}

void timerSetCounter(TIM_TypeDef *timer, uint32_t counter)
{
    TIMER_CNT((uint32_t)timer) = counter;
}

void timerDisable(TIM_TypeDef *timer)
{
    timer_interrupt_disable((uint32_t)timer, TIMER_INT_UP);
    timer_disable((uint32_t)timer);
}

void timerEnable(TIM_TypeDef *timer)
{
    timer_enable((uint32_t)timer);
    timer_event_software_generate((uint32_t)timer, TIMER_EVENT_SRC_UPG);
}

void timerEnableInterrupt(TIM_TypeDef *timer)
{
    timer_flag_clear((uint32_t)timer, TIMER_FLAG_UP);
    timer_interrupt_enable((uint32_t)timer, TIMER_INT_UP);
}

#endif
