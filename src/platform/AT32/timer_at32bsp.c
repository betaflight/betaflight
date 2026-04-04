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
#include "platform/timer.h"

#define TIM_N(n) (1 << (n))

/*
    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM1 2 channels
    TIM2 4 channels
    TIM3 4 channels
    TIM4 4 channels
*/

#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)
#define CC_CHANNELS_PER_TIMER 4

#define TIM_IT_CCx(ch) (TMR_C1_INT << ((ch)-1))

typedef struct timerConfig_s {
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

#ifdef TIMER_CHANNEL_COUNT
timerChannelInfo_t timerChannelInfo[TIMER_CHANNEL_COUNT];
#endif

typedef struct {
    uint8_t priority;
} timerInfo_t;
timerInfo_t timerInfo[USED_TIMER_COUNT];

// return index of timer in timer table. Lowest timer has index 0
#define TIMER_INDEX(i) BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

static uint8_t lookupTimerIndex(const timerResource_t *tim)
{
    const tmr_type *tim_ptr = (const tmr_type *)tim;
#define _CASE_SHF 10 // amount we can safely shift timer address to the right. gcc will throw error if some timers overlap
#define _CASE_(tim, index) case ((unsigned)tim >> _CASE_SHF): return index; break
#define _CASE(i) _CASE_(TMR##i##_BASE, TIMER_INDEX(i))

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
#if USED_TIMERS & TIM_N(20)
        _CASE(20);
#endif
    default:  return ~1;  // make sure final index is out of range
    }
#undef _CASE
#undef _CASE_
}

tmr_type * const usedTimers[USED_TIMER_COUNT] = {
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
#if USED_TIMERS & TIM_N(20)
    _DEF(20),
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
//    return channel >> 2;
    return channel -1 ;//at32 use 1\2\3\4 as channel num
}

uint8_t timerLookupChannelIndex(const uint16_t channel)
{
    return lookupChannelIndex(channel);
}

rccPeriphTag_t timerRCC(const timerResource_t *tim)
{
    const tmr_type *tim_ptr = (const tmr_type *)tim;
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim_ptr) {
            return timerDefinitions[i].rcc;
        }
    }
    return 0;
}

uint8_t timerInputInterrupt(const timerHardware_t *timHw)
{
    const tmr_type *tim_ptr = (const tmr_type *)timHw->tim;
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim_ptr) {
            return timerDefinitions[i].inputIrq;
        }
    }
    return 0;
}

static void timerNVICConfigure(uint8_t irq)
{
    nvic_irq_enable(irq,NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER),NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
}

// Helper for callers that only have a timerResource_t* timer instance
uint32_t timerClockFromInstance(const timerResource_t *tim)
{
    const tmr_type *tim_ptr = (const tmr_type *)tim;

    crm_clocks_freq_type clk_freq;
    crm_clocks_freq_get(&clk_freq);

    if (tim_ptr == TMR1 || tim_ptr == TMR8 || tim_ptr == TMR9 || tim_ptr == TMR10 || tim_ptr == TMR11 || tim_ptr == TMR20) {
        // APB2 timers
        uint32_t pclk = clk_freq.apb2_freq;
        if (CRM->cfg_bit.apb2div >= 4) {
            pclk *= 2;
        }
        return pclk;
    } else {
        // APB1 timers: TMR2-7, TMR12-14
        uint32_t pclk = clk_freq.apb1_freq;
        if (CRM->cfg_bit.apb1div >= 4) {
            pclk *= 2;
        }
        return pclk;
    }
}

void timerReconfigureTimeBase(const timerHardware_t *timHw, uint16_t period, uint32_t hz)
{
    if (hz == 0) {
        return;
    }
    tmr_type *tim_ptr = (tmr_type *)timHw->tim;
    //timer, period, perscaler
    tmr_base_init(tim_ptr,(period - 1) & 0xFFFF,(timerClock(timHw) / hz) - 1);
    //TMR_CLOCK_DIV1 = 0X00 NO DIV
    tmr_clock_source_div_set(tim_ptr,TMR_CLOCK_DIV1);
    //COUNT UP
    tmr_cnt_dir_set(tim_ptr,TMR_COUNT_UP);
}

// old interface for PWM inputs. It should be replaced
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint32_t hz)
{
    timerReconfigureTimeBase(timerHardwarePtr, period, hz);
    tmr_counter_enable((tmr_type *)timerHardwarePtr->tim, TRUE);

    uint8_t irq = timerInputInterrupt(timerHardwarePtr);
    timerNVICConfigure(irq);
    switch (irq) {
#if defined(AT32F4)
    case TMR1_CH_IRQn:
        timerNVICConfigure(TMR1_OVF_TMR10_IRQn);
        break;
#endif
    }
}

// allocate and configure timer channel. Timer priority is set to highest priority of its channels
void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq)
{
    #ifndef USE_TIMER_MGMT

    UNUSED(timHw);
    UNUSED(type);
    UNUSED(irqPriority);
    UNUSED(irq);
    return;

    #else

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
        tmr_counter_enable(usedTimers[timer], TRUE);

        nvic_irq_enable(irq, NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));

        timerInfo[timer].priority = irqPriority;
    }
    #endif
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
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, const timerResource_t *tim) {
    const tmr_type *tim_ptr = (const tmr_type *)tim;
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
    tmr_interrupt_enable((tmr_type *)tim_ptr, TMR_OVF_INT, cfg->overflowCallbackActive ? TRUE : FALSE);
}

// config edge and overflow callback for channel. Try to avoid per-channel overflowCallback, it is a bit expensive
void timerChannelConfigCallbacks(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    uint8_t channelIndex = lookupChannelIndex(timHw->channel);
    if (edgeCallback == NULL) {
        // disable irq before changing callback to NULL
        tmr_interrupt_enable((tmr_type *)timHw->tim, TIM_IT_CCx(timHw->channel), FALSE);
    }

    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    // enable channel IRQ
    if (edgeCallback) {
        tmr_interrupt_enable((tmr_type *)timHw->tim, TIM_IT_CCx(timHw->channel), TRUE);
    }

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

// enable or disable IRQ
void timerChannelConfigInterrupt(const timerHardware_t *timHw, FunctionalState newState)
{
    tmr_interrupt_enable((tmr_type *)timHw->tim, TIM_IT_CCx(timHw->channel), newState ? TRUE : FALSE);
}

// clear Compare/Capture flag for channel
void timerChannelClearFlag(const timerHardware_t *timHw)
{
    tmr_flag_clear((tmr_type *)timHw->tim, TIM_IT_CCx(timHw->channel));
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
    tmr_input_config_type tmr_icInitStructure;
    tmr_icInitStructure.input_channel_select = TIM_CH_TO_SELCHANNEL(timHw->channel) ;// MAPS 1234 TO 0 2 4 6
    tmr_icInitStructure.input_polarity_select = polarityRising ?TMR_INPUT_RISING_EDGE:TMR_INPUT_FALLING_EDGE;
    tmr_icInitStructure.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    tmr_icInitStructure.input_filter_value = getFilter(inputFilterTicks);

    tmr_input_channel_init((tmr_type *)timHw->tim,&tmr_icInitStructure,TMR_CHANNEL_INPUT_DIV_1);
}

// Internal helper
static inline volatile timCCR_t* timerGetCCRPointer(tmr_type *tim, uint8_t channel)
{
    switch (channel) {
    case 1: return (volatile timCCR_t*)(&tim->c1dt);
    case 2: return (volatile timCCR_t*)(&tim->c2dt);
    case 3: return (volatile timCCR_t*)(&tim->c3dt);
    case 4: return (volatile timCCR_t*)(&tim->c4dt);
    default: return NULL;
    }
}

volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
    return timerGetCCRPointer((tmr_type *)timHw->tim, timHw->channel);
}

static void timCCxHandler(timerResource_t *tim, timerConfig_t *timerConfig)
{
    tmr_type *tim_ptr = (tmr_type *)tim;
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim_ptr->ists & tim_ptr->iden;
#if 1
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // current order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim_ptr->ists = mask;
        tim_status &= mask;
        switch (bit) {
            case __builtin_clz(TMR_OVF_FLAG): {

                if (timerConfig->forcedOverflowTimerValue != 0) {
                    capture = timerConfig->forcedOverflowTimerValue - 1;
                    timerConfig->forcedOverflowTimerValue = 0;
                } else {
                    capture = tim_ptr->pr;
                }

                timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
                while (cb) {
                    cb->fn(cb, capture);
                    cb = cb->next;
                }
                break;
            }
            case __builtin_clz(TMR_C1_FLAG):
                if (timerConfig->edgeCallback[0] && timerConfig->edgeCallback[0]->fn) {
                    timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim_ptr->c1dt);
                }
                break;
            case __builtin_clz(TMR_C2_FLAG):
                if (timerConfig->edgeCallback[1] && timerConfig->edgeCallback[1]->fn) {
                    timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim_ptr->c2dt);
                }
                break;
            case __builtin_clz(TMR_C3_FLAG):
                if (timerConfig->edgeCallback[2] && timerConfig->edgeCallback[2]->fn) {
                    timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim_ptr->c3dt);
                }
                break;
            case __builtin_clz(TMR_C4_FLAG):
                if (timerConfig->edgeCallback[3] && timerConfig->edgeCallback[3]->fn) {
                    timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim_ptr->c4dt);
                }
                break;
        }
    }
#endif
}

static inline void timUpdateHandler(timerResource_t *tim, timerConfig_t *timerConfig)
{
    tmr_type *tim_ptr = (tmr_type *)tim;
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim_ptr->ists & tim_ptr->iden;
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim_ptr->ists = mask;
        tim_status &= mask;
        switch (bit) {
            case __builtin_clz(TMR_OVF_FLAG): { // tim_it_update= 0x0001 => TMR_OVF_FLAG

                if (timerConfig->forcedOverflowTimerValue != 0) {
                    capture = timerConfig->forcedOverflowTimerValue - 1;
                    timerConfig->forcedOverflowTimerValue = 0;
                } else {
                    capture = tim_ptr->pr;
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
        timCCxHandler((timerResource_t *)TMR ## i, &timerConfig[TIMER_INDEX(i)]); \
        timCCxHandler((timerResource_t *)TMR ## j, &timerConfig[TIMER_INDEX(j)]); \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        timCCxHandler((timerResource_t *)TMR ## i, &timerConfig[TIMER_INDEX(i)]); \
    } struct dummy

#define _TIM_IRQ_HANDLER_UPDATE_ONLY(name, i)                           \
    void name(void)                                                     \
    {                                                                   \
        timUpdateHandler((timerResource_t *)TMR ## i, &timerConfig[TIMER_INDEX(i)]); \
    } struct dummy

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TMR1_CH_IRQHandler, 1);
#endif
#if USED_TIMERS & TIM_N(2)
_TIM_IRQ_HANDLER(TMR2_GLOBAL_IRQHandler, 2);
#endif
#if USED_TIMERS & TIM_N(3)
_TIM_IRQ_HANDLER(TMR3_GLOBAL_IRQHandler, 3);
#endif
#if USED_TIMERS & TIM_N(4)
_TIM_IRQ_HANDLER(TMR4_GLOBAL_IRQHandler, 4);
#endif
#if USED_TIMERS & TIM_N(5)
_TIM_IRQ_HANDLER(TMR5_GLOBAL_IRQHandler, 5);
#endif
#if USED_TIMERS & TIM_N(6)
_TIM_IRQ_HANDLER(TMR6_DAC_GLOBAL_IRQHandler, 6);
#endif

#if USED_TIMERS & TIM_N(7)
_TIM_IRQ_HANDLER(TMR7_GLOBAL_IRQHandler, 7);
#endif

#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TMR8_CH_IRQHandler, 8);
#endif
#if USED_TIMERS & TIM_N(9)
_TIM_IRQ_HANDLER(TMR1_BRK_TMR9_IRQHandler, 9);
#endif
//TODO: there may be a bug
#if USED_TIMERS & TIM_N(10)
_TIM_IRQ_HANDLER2(TMR1_OVF_TMR10_IRQHandler, 1,10);
#endif
#  if USED_TIMERS & TIM_N(11)
_TIM_IRQ_HANDLER(TMR1_TRG_HALL_TMR11_IRQHandler, 11);
#  endif
#if USED_TIMERS & TIM_N(12)
_TIM_IRQ_HANDLER(TMR8_BRK_TMR12_IRQHandler, 12);
#endif
#if USED_TIMERS & TIM_N(13)
_TIM_IRQ_HANDLER(TMR8_OVF_TMR13_IRQHandler, 13);
#endif
#if USED_TIMERS & TIM_N(14)
_TIM_IRQ_HANDLER(TMR8_TRG_HALL_TMR14_IRQHandler, 14);
#endif
#if USED_TIMERS & TIM_N(20)
_TIM_IRQ_HANDLER(TMR20_CH_IRQHandler, 20);
#endif

void timerInit(void)
{
    memset(timerConfig, 0, sizeof(timerConfig));

    #ifdef USE_TIMER_MGMT
    /* enable the timer peripherals */
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        RCC_ClockCmd(timerRCC(TIMER_HARDWARE[i].tim), ENABLE);
    }

    // initialize timer channel structures
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        timerChannelInfo[i].type = TYPE_FREE;
    }
    #endif

    for (unsigned i = 0; i < USED_TIMER_COUNT; i++) {
        timerInfo[i].priority = ~0;
    }
}

void timerStart(const timerHardware_t *timHw)
{
    tmr_type *tim_ptr = (tmr_type *)timHw->tim;
    tmr_counter_enable(tim_ptr, TRUE);
}

/**
 * Force an overflow for a given timer.
 * Saves the current value of the counter in the relevant timerConfig's forcedOverflowTimerValue variable.
 * @param tmr_type *tim The timer to overflow
 * @return void
 **/
void timerForceOverflow(timerResource_t *tim)
{
    tmr_type *tim_ptr = (tmr_type *)tim;
    uint8_t timerIndex = lookupTimerIndex(tim);

    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        // Save the current count so that PPM reading will work on the same timer that was forced to overflow
        timerConfig[timerIndex].forcedOverflowTimerValue = tim_ptr->cval + 1;

        // Force an overflow by setting the UG bit
        tim_ptr->swevt_bit.ovfswtr = 1;
    }
}

void timerOCInit(tmr_type *tim, uint8_t channel, tmr_output_config_type *init)
{
    tmr_output_channel_config(tim, TIM_CH_TO_SELCHANNEL(channel), init);
}

void timerOCPreloadConfig(tmr_type *tim, uint8_t channel, uint16_t preload)
{
    tmr_output_channel_buffer_enable(tim, TIM_CH_TO_SELCHANNEL(channel), preload);
}

volatile timCCR_t* timerCCR(timerResource_t *tim, uint8_t channel)
{
    return timerGetCCRPointer((tmr_type *)tim, channel);
}

uint16_t timerDmaSource(uint8_t channel)
{
    switch (channel) {
    case 1:
        return TMR_C1_DMA_REQUEST;
    case 2:
        return TMR_C2_DMA_REQUEST;
    case 3:
        return TMR_C3_DMA_REQUEST;
    case 4:
        return TMR_C4_DMA_REQUEST;
    }
    return 0;
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
    tmr_type *timer_ptr = (tmr_type *)timHw->tim;
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        tmr_counter_enable(timer_ptr, FALSE);
    }
}

void timerSetPeriod(const timerHardware_t *timHw, uint32_t period)
{
    tmr_type *timer_ptr = (tmr_type *)timHw->tim;
    tmr_period_value_set(timer_ptr, period);
}

uint32_t timerGetPeriod(const timerHardware_t *timHw)
{
    tmr_type *timer_ptr = (tmr_type *)timHw->tim;
    return tmr_period_value_get(timer_ptr);
}

void timerSetCounter(const timerHardware_t *timHw, uint32_t counter)
{
    tmr_type *timer_ptr = (tmr_type *)timHw->tim;
    tmr_counter_value_set(timer_ptr, counter);
}

void timerDisable(const timerHardware_t *timHw)
{
    tmr_type *timer_ptr = (tmr_type *)timHw->tim;
    tmr_interrupt_enable(timer_ptr, TMR_OVF_INT, FALSE);
    tmr_counter_enable(timer_ptr, FALSE);
}

void timerEnable(const timerHardware_t *timHw)
{
    tmr_type *timer_ptr = (tmr_type *)timHw->tim;
    tmr_counter_enable(timer_ptr, TRUE);
    tmr_overflow_event_disable(timer_ptr, TRUE);
}

void timerEnableInterrupt(const timerHardware_t *timHw)
{
    tmr_type *timer_ptr = (tmr_type *)timHw->tim;
    tmr_flag_clear(timer_ptr, TMR_OVF_FLAG);
    tmr_interrupt_enable(timer_ptr, TMR_OVF_INT, TRUE);
}

uint32_t timerGetPrescaler(const timerHardware_t *timHw)
{
    return ((const tmr_type *)timHw->tim)->div;
}

void *timerFindTimerHandle(timerResource_t *tim)
{
    UNUSED(tim);
    return NULL;
}

#endif
