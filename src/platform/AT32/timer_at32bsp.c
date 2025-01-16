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
#include "drivers/rcc.h"
#include "drivers/system.h"
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

#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)
#define CC_CHANNELS_PER_TIMER 4

#define TIM_IT_CCx(ch) (TMR_C1_INT << ((ch)-1))

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

#ifdef TIMER_CHANNEL_COUNT
timerChannelInfo_t timerChannelInfo[TIMER_CHANNEL_COUNT];
#endif

typedef struct {
    uint8_t priority;
} timerInfo_t;
timerInfo_t timerInfo[USED_TIMER_COUNT];

// return index of timer in timer table. Lowest timer has index 0
#define TIMER_INDEX(i) BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

static uint8_t lookupTimerIndex(const tmr_type *tim)
{
#define _CASE_SHF 10 // amount we can safely shift timer address to the right. gcc will throw error if some timers overlap
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

int8_t timerGetTIMNumber(const tmr_type *tim)
{
    const uint8_t index = lookupTimerIndex(tim);

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

rccPeriphTag_t timerRCC(const tmr_type *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].rcc;
        }
    }
    return 0;
}

uint8_t timerInputIrq(const tmr_type *tim)
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
    nvic_irq_enable(irq,NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER),NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
}

void configTimeBase(tmr_type *tim, uint16_t period, uint32_t hz)
{
    //timer, period, perscaler
    tmr_base_init(tim,(period - 1) & 0xFFFF,(timerClock(tim) / hz) - 1);
    //TMR_CLOCK_DIV1 = 0X00 NO DIV
    tmr_clock_source_div_set(tim,TMR_CLOCK_DIV1);
    //COUNT UP
    tmr_cnt_dir_set(tim,TMR_COUNT_UP);

}

// old interface for PWM inputs. It should be replaced
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint32_t hz)
{
    configTimeBase(timerHardwarePtr->tim, period, hz);
    tmr_counter_enable(timerHardwarePtr->tim, TRUE);

    uint8_t irq = timerInputIrq(timerHardwarePtr->tim);
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
        configTimeBase(usedTimers[timer], 0, 1);
        tmr_counter_enable(usedTimers[timer], TRUE);

        nvic_irq_enable(irq, NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));

        timerInfo[timer].priority = irqPriority;
    }
    #endif
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
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, const tmr_type *tim) {
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
    tmr_interrupt_enable((tmr_type *)tim, TMR_OVF_INT, cfg->overflowCallbackActive ? TRUE : FALSE);
}

// config edge and overflow callback for channel. Try to avoid per-channel overflowCallback, it is a bit expensive
void timerChConfigCallbacks(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }

    uint8_t channelIndex = lookupChannelIndex(timHw->channel);
    if (edgeCallback == NULL) {
        // disable irq before changing callback to NULL
        tmr_interrupt_enable(timHw->tim, TIM_IT_CCx(timHw->channel), FALSE);
    }

    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    // enable channel IRQ
    if (edgeCallback) {
        tmr_interrupt_enable(timHw->tim, TIM_IT_CCx(timHw->channel), TRUE);
    }

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

void timerConfigUpdateCallback(const tmr_type *tim, timerOvrHandlerRec_t *updateCallback)
{
    uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    timerConfig[timerIndex].updateCallback = updateCallback;
    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], tim);
}

// enable or disable IRQ
void timerChITConfig(const timerHardware_t *timHw, FunctionalState newState)
{
    tmr_interrupt_enable(timHw->tim, TIM_IT_CCx(timHw->channel), newState ? TRUE : FALSE);
}

// clear Compare/Capture flag for channel
void timerChClearCCFlag(const timerHardware_t *timHw)
{
    tmr_flag_clear(timHw->tim, TIM_IT_CCx(timHw->channel));
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
    tmr_input_config_type tmr_icInitStructure;
    tmr_icInitStructure.input_channel_select = TIM_CH_TO_SELCHANNEL(timHw->channel) ;// MAPS 1234 TO 0 2 4 6
    tmr_icInitStructure.input_polarity_select = polarityRising ?TMR_INPUT_RISING_EDGE:TMR_INPUT_FALLING_EDGE;
    tmr_icInitStructure.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    tmr_icInitStructure.input_filter_value = getFilter(inputFilterTicks);

    tmr_input_channel_init(timHw->tim,&tmr_icInitStructure,TMR_CHANNEL_INPUT_DIV_1);
}

volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{

    if(timHw->channel == 1)
        return (volatile timCCR_t*)(&timHw->tim->c1dt);
    else if(timHw->channel == 2)
        return (volatile timCCR_t*)(&timHw->tim->c2dt);
    else if(timHw->channel == 3)
        return (volatile timCCR_t*)(&timHw->tim->c3dt);
    else if(timHw->channel == 4)
        return (volatile timCCR_t*)(&timHw->tim->c4dt);
    else
        return (volatile timCCR_t*)((volatile char*)&timHw->tim->c1dt + (timHw->channel-1)*0x04); //for 32bit need to debug

}

static void timCCxHandler(tmr_type *tim, timerConfig_t *timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim->ists & tim->iden;
#if 1
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // current order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->ists = mask;
        tim_status &= mask;
        switch (bit) {
            case __builtin_clz(TMR_OVF_FLAG): {

                if (timerConfig->forcedOverflowTimerValue != 0) {
                    capture = timerConfig->forcedOverflowTimerValue - 1;
                    timerConfig->forcedOverflowTimerValue = 0;
                } else {
                    capture = tim->pr;
                }

                timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
                while (cb) {
                    cb->fn(cb, capture);
                    cb = cb->next;
                }
                break;
            }
            case __builtin_clz(TMR_C1_FLAG):
                timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->c1dt);
                break;
            case __builtin_clz(TMR_C2_FLAG):
                timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->c2dt);
                break;
            case __builtin_clz(TMR_C3_FLAG):
                timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->c3dt);
                break;
            case __builtin_clz(TMR_C4_FLAG):
                timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->c4dt);
                break;
        }
    }
#endif
}

static inline void timUpdateHandler(tmr_type *tim, timerConfig_t *timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim->ists & tim->iden;
    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->ists = mask;
        tim_status &= mask;
        switch (bit) {
            case __builtin_clz(TMR_OVF_FLAG): { // tim_it_update= 0x0001 => TMR_OVF_FLAG

                if (timerConfig->forcedOverflowTimerValue != 0) {
                    capture = timerConfig->forcedOverflowTimerValue - 1;
                    timerConfig->forcedOverflowTimerValue = 0;
                } else {
                    capture = tim->pr;
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

#if defined(PARTIAL_REMAP_TIM3)
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
#endif

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

void timerStart(tmr_type *tim)
{
    tmr_counter_enable(tim, TRUE);
}

/**
 * Force an overflow for a given timer.
 * Saves the current value of the counter in the relevant timerConfig's forcedOverflowTimerValue variable.
 * @param tmr_type *tim The timer to overflow
 * @return void
 **/
void timerForceOverflow(tmr_type *tim)
{
    uint8_t timerIndex = lookupTimerIndex((const tmr_type *)tim);

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        // Save the current count so that PPM reading will work on the same timer that was forced to overflow
        timerConfig[timerIndex].forcedOverflowTimerValue = tim->cval + 1;

        // Force an overflow by setting the UG bit
        tim->swevt_bit.ovfswtr = 1;
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

//tmr_channel_value_get
volatile timCCR_t* timerCCR(tmr_type *tim, uint8_t channel)
{

    if(channel ==1)
        return (volatile timCCR_t*)(&tim->c1dt);
    else if(channel ==2)
        return (volatile timCCR_t*)(&tim->c2dt);
    else if(channel ==3)
        return (volatile timCCR_t*)(&tim->c3dt);
    else if(channel ==4)
        return (volatile timCCR_t*)(&tim->c4dt);
    else
        return (volatile timCCR_t*)((volatile char*)&tim->c1dt + (channel-1)*0x04); //for 32bit need to debug

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

uint16_t timerGetPrescalerByDesiredMhz(tmr_type *tim, uint16_t mhz)
{
    return timerGetPrescalerByDesiredHertz(tim, MHZ_TO_HZ(mhz));
}

uint16_t timerGetPeriodByPrescaler(tmr_type *tim, uint16_t prescaler, uint32_t hz)
{
    return (uint16_t)((timerClock(tim) / (prescaler + 1)) / hz);
}

uint16_t timerGetPrescalerByDesiredHertz(tmr_type *tim, uint32_t hz)
{
    // protection here for desired hertz > SystemCoreClock???
    if (hz > timerClock(tim)) {
        return 0;
    }
    return (uint16_t)((timerClock(tim) + hz / 2 ) / hz) - 1;
}
#endif
