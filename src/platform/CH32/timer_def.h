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

#pragma once

#include "platform.h"
#include "common/utils.h"
#include "platform/dma.h"

#ifdef USE_INTERNAL_OSD
//timer2/10/11/12 reserved for internal OSD 
#define USED_TIMERS  ( BIT(1) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(8) | BIT(9) )
#define TIMUP_TIMERS ( BIT(1) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(8) | BIT(9) )
#define FULL_TIMER_CHANNEL_COUNT        130
#define HARDWARE_TIMER_DEFINITION_COUNT BITCOUNT(USED_TIMERS)
#else
#define USED_TIMERS  ( BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(8) | BIT(9) | BIT(10) | BIT(11) | BIT(12))
#define TIMUP_TIMERS ( BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(8) | BIT(9) | BIT(10) | BIT(11) | BIT(12))
#define FULL_TIMER_CHANNEL_COUNT        130
#define HARDWARE_TIMER_DEFINITION_COUNT BITCOUNT(USED_TIMERS)
#endif

// allow conditional definition of DMA related members
#if defined(USE_TIMER_DMA)
# define DEF_TIM_DMA_COND(...) __VA_ARGS__
#else
# define DEF_TIM_DMA_COND(...)
#endif

#if defined(USE_TIMER_MGMT)
#define TIMER_GET_IO_TAG(pin) DEFIO_TAG_E(pin)
#else
#define TIMER_GET_IO_TAG(pin) DEFIO_TAG(pin)
#endif

// map to base channel (strip N from channel); works only when channel N exists
#define DEF_TIM_TCH2BTCH(timch) CONCAT(B, timch)
#define BTCH_TIM1_CH1N BTCH_TIM1_CH1
#define BTCH_TIM1_CH2N BTCH_TIM1_CH2
#define BTCH_TIM1_CH3N BTCH_TIM1_CH3

#define BTCH_TIM8_CH1N BTCH_TIM8_CH1
#define BTCH_TIM8_CH2N BTCH_TIM8_CH2
#define BTCH_TIM8_CH3N BTCH_TIM8_CH3


// channel table D(chan_n, n_type)
#define DEF_TIM_CH_GET(ch) CONCAT2(DEF_TIM_CH__, ch)
#define DEF_TIM_CH__CH_CH1  D(1, 0)
#define DEF_TIM_CH__CH_CH2  D(2, 0)
#define DEF_TIM_CH__CH_CH3  D(3, 0)
#define DEF_TIM_CH__CH_CH4  D(4, 0)
#define DEF_TIM_CH__CH_CH1N D(1, 1)
#define DEF_TIM_CH__CH_CH2N D(2, 1)
#define DEF_TIM_CH__CH_CH3N D(3, 1)

// timer table D(tim_n)
#define DEF_TIM_TIM_GET(tim) CONCAT2(DEF_TIM_TIM__, tim)
#define DEF_TIM_TIM__TIM_TIM1  D(1)
#define DEF_TIM_TIM__TIM_TIM2  D(2)
#define DEF_TIM_TIM__TIM_TIM3  D(3)
#define DEF_TIM_TIM__TIM_TIM4  D(4)
#define DEF_TIM_TIM__TIM_TIM5  D(5)
#define DEF_TIM_TIM__TIM_TIM6  D(6)
#define DEF_TIM_TIM__TIM_TIM7  D(7)
#define DEF_TIM_TIM__TIM_TIM8  D(8)
#define DEF_TIM_TIM__TIM_TIM9  D(9)
#define DEF_TIM_TIM__TIM_TIM10 D(10)
#define DEF_TIM_TIM__TIM_TIM11 D(11)
#define DEF_TIM_TIM__TIM_TIM12 D(12)
#define DEF_TIM_TIM__TIM_TIM13 D(13)
#define DEF_TIM_TIM__TIM_TIM14 D(14)
#define DEF_TIM_TIM__TIM_TIM15 D(15)
#define DEF_TIM_TIM__TIM_TIM16 D(16)
#define DEF_TIM_TIM__TIM_TIM17 D(17)
#define DEF_TIM_TIM__TIM_TIM18 D(18)
#define DEF_TIM_TIM__TIM_TIM19 D(19)
#define DEF_TIM_TIM__TIM_TIM20 D(20)
#define DEF_TIM_TIM__TIM_TIM21 D(21)
#define DEF_TIM_TIM__TIM_TIM22 D(22)

// get record from DMA table
// DMA table is identical for all targets for consistency, only variant 0 is defined on F1,F3
// DMA table entry for TIMx Channel y, with two variants:
//  #define DEF_TIM_DMA__BTCH_TIMx_CHy  D(var0),D(var1)
//  Parameters in D(...) are target-specific
// DMA table for channel without DMA
//  #define DEF_TIM_DMA__BTCH_TIMx_CHy NONE
// N channels are converted to corresponding base channel first

// Create accessor macro and call it with entry from table
// DMA_VARIANT_MISSING are used to satisfy variable arguments (-Wpedantic) and to get better error message (undefined symbol instead of preprocessor error)
#define DEF_TIM_DMA_GET(variant, timch) PP_CALL(CONCAT(DEF_TIM_DMA_GET_VARIANT__, variant), CONCAT(DEF_TIM_DMA__, DEF_TIM_TCH2BTCH(timch)), DMA_VARIANT_MISSING, DMA_VARIANT_MISSING, ERROR)

#define DEF_TIM_DMA_GET_VARIANT__0(_0, ...) _0
#define DEF_TIM_DMA_GET_VARIANT__1(_0, _1, ...) _1
#define DEF_TIM_DMA_GET_VARIANT__2(_0, _1, _2, ...) _2
#define DEF_TIM_DMA_GET_VARIANT__3(_0, _1, _2, _3, ...) _3
#define DEF_TIM_DMA_GET_VARIANT__4(_0, _1, _2, _3, _4, ...) _4
#define DEF_TIM_DMA_GET_VARIANT__5(_0, _1, _2, _3, _4, _5, ...) _5
#define DEF_TIM_DMA_GET_VARIANT__6(_0, _1, _2, _3, _4, _5, _6, ...) _6
#define DEF_TIM_DMA_GET_VARIANT__7(_0, _1, _2, _3, _4, _5, _6, _7, ...) _7
#define DEF_TIM_DMA_GET_VARIANT__8(_0, _1, _2, _3, _4, _5, _6, _7, _8, ...) _8
#define DEF_TIM_DMA_GET_VARIANT__9(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, ...) _9
#define DEF_TIM_DMA_GET_VARIANT__10(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, ...) _10
#define DEF_TIM_DMA_GET_VARIANT__11(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, ...) _11
#define DEF_TIM_DMA_GET_VARIANT__12(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, ...) _12
#define DEF_TIM_DMA_GET_VARIANT__13(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, ...) _13
#define DEF_TIM_DMA_GET_VARIANT__14(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, ...) _14
#define DEF_TIM_DMA_GET_VARIANT__15(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, ...) _15

// symbolic names for DMA variants
#define DMA_VAR0     0
#define DMA_VAR1     1
#define DMA_VAR2     2

// get record from AF table
//  Parameters in D(...) are target-specific
#define DEF_TIM_AF_GET(timch, pin) CONCAT4(DEF_TIM_AF__, pin, __, timch)

//AF
#define DEF_TIM_AF(timch, pin)                CONCAT(DEF_TIM_AF__, DEF_TIM_AF_GET(timch, pin))
#define DEF_TIM_AF__D(af_n, tim_n)            GPIO_AF ## af_n  /* GPIO_AF_Define */

// define output type (N-channel)
#define DEF_TIM_OUTPUT(ch)         CONCAT(DEF_TIM_OUTPUT__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_OUTPUT__D(chan_n, n_channel) PP_IIF(n_channel, TIMER_OUTPUT_N_CHANNEL, TIMER_OUTPUT_NONE)

/*
 DEF_TIM(tim, chan, pin, flags, out, dmaopt, upopt)
        @tim,
        @chan    tmr & channel
        @pin     output pin
        @out     0 for normal 1 for N_Channel
        @dmaopt  dma channel index used for timer channel data transmit
        @upopt   USE_DSHOT_DMAR  timeup dma channel index
*/
#define DEF_TIM(tim, chan, pin, out, dmaopt, upopt) {                   \
    tim,                                                                \
    TIMER_GET_IO_TAG(pin),                                              \
    DEF_TIM_CHANNEL(CH_ ## chan),                                       \
    (DEF_TIM_OUTPUT(CH_ ## chan) | out),                                \
    DEF_TIM_AF(TCH_## tim ## _ ## chan, pin)                            \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_CHANNEL(dmaopt, TCH_## tim ## _ ## chan),           \
        DEF_TIM_DMA_REQUEST(TCH_## tim ## _ ## chan)                    \
    )                                                                   \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_CHANNEL(upopt, TCH_## tim ## _UP),                  \
        DEF_TIM_DMA_REQUEST(TCH_## tim ## _UP),                         \
        DEF_TIM_DMA_HANDLER(upopt, TCH_## tim ## _UP)                   \
    )                                                                   \
}
/**/

//Channel
#define DEF_TIM_CHANNEL(ch)                   CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) TIM_Channel_ ## chan_n

#define DEF_TIM_DMA_CHANNEL(variant, timch)                              \
    CONCAT(DEF_TIM_DMA_CHANNEL__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_CHANNEL__D(dma_n, channel_n)  (dmaResource_t *)DMA ## dma_n ## _Channel ## channel_n
#define DEF_TIM_DMA_CHANNEL__NONE                        NULL

#define DEF_TIM_DMA_REQUEST(timch) \
    CONCAT(DEF_TIM_DMA_REQ__, DEF_TIM_TCH2BTCH(timch))

#define DEF_TIM_DMA_HANDLER(variant, timch) \
    CONCAT(DEF_TIM_DMA_HANDLER__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_HANDLER__D(dma_n, channel_n) DMA ## dma_n ## _CH ## channel_n ## _HANDLER
#define DEF_TIM_DMA_HANDLER__NONE                       0

/* DMA Channel Mappings */
// D(DMAx, Stream)
// ch32h417 has DMAMUX that allow arbitrary assignment of peripherals to streams.
#define DEF_TIM_DMA_FULL \
    D(1, 1), D(1, 2), D(1, 3), D(1, 4), D(1, 5), D(1, 6), D(1, 7),D(1, 8), \
    D(2, 1), D(2, 2), D(2, 3), D(2, 4), D(2, 5), D(2, 6), D(2, 7),D(2, 8)

#define DEF_TIM_DMA__BTCH_TIM1_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM1_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM1_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM1_CH4    DEF_TIM_DMA_FULL

#ifdef USE_INTERNAL_OSD
#define DEF_TIM_DMA__BTCH_TIM2_CH1    NONE
#define DEF_TIM_DMA__BTCH_TIM2_CH2    NONE
#define DEF_TIM_DMA__BTCH_TIM2_CH3    NONE
#define DEF_TIM_DMA__BTCH_TIM2_CH4    NONE
#else
#define DEF_TIM_DMA__BTCH_TIM2_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_CH4    DEF_TIM_DMA_FULL
#endif

#define DEF_TIM_DMA__BTCH_TIM3_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM3_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM3_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM3_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM4_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM4_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM4_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM4_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM5_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM5_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM5_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM5_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM8_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM8_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM8_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM8_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM9_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM9_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM9_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM9_CH4    DEF_TIM_DMA_FULL


#ifdef USE_INTERNAL_OSD
#define DEF_TIM_DMA__BTCH_TIM10_CH1   NONE
#define DEF_TIM_DMA__BTCH_TIM10_CH2   NONE
#define DEF_TIM_DMA__BTCH_TIM10_CH3   NONE
#define DEF_TIM_DMA__BTCH_TIM10_CH4   NONE

#define DEF_TIM_DMA__BTCH_TIM11_CH1   NONE
#define DEF_TIM_DMA__BTCH_TIM11_CH2   NONE
#define DEF_TIM_DMA__BTCH_TIM11_CH3   NONE
#define DEF_TIM_DMA__BTCH_TIM11_CH4   NONE

#define DEF_TIM_DMA__BTCH_TIM12_CH1   NONE
#define DEF_TIM_DMA__BTCH_TIM12_CH2   NONE
#define DEF_TIM_DMA__BTCH_TIM12_CH3   NONE
#define DEF_TIM_DMA__BTCH_TIM12_CH4   NONE
#else
#define DEF_TIM_DMA__BTCH_TIM10_CH1   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM10_CH2   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM10_CH3   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM10_CH4   DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM11_CH1   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM11_CH2   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM11_CH3   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM11_CH4   DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM12_CH1   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM12_CH2   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM12_CH3   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM12_CH4   DEF_TIM_DMA_FULL
#endif


// TIM_UP table
#define DEF_TIM_DMA__BTCH_TIM1_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM3_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM4_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM5_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM6_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM7_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM8_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM9_UP     DEF_TIM_DMA_FULL

#ifdef USE_INTERNAL_OSD
#define DEF_TIM_DMA__BTCH_TIM2_UP     NONE
#define DEF_TIM_DMA__BTCH_TIM10_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM11_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM12_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM13_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM14_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM15_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM16_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM17_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM20_UP    NONE
#else
#define DEF_TIM_DMA__BTCH_TIM2_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM10_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM11_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM12_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM13_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM14_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM15_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM16_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM17_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM20_UP    DEF_TIM_DMA_FULL
#endif
// TIMx_CHy request table

#define DMA_REQUEST_NONE 255

#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH1    DMAMUX_DMAREQ_ID_TIM1_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH2    DMAMUX_DMAREQ_ID_TIM1_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH3    DMAMUX_DMAREQ_ID_TIM1_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH4    DMAMUX_DMAREQ_ID_TIM1_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH1    DMAMUX_DMAREQ_ID_TIM2_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH2    DMAMUX_DMAREQ_ID_TIM2_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH3    DMAMUX_DMAREQ_ID_TIM2_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH4    DMAMUX_DMAREQ_ID_TIM2_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH1    DMAMUX_DMAREQ_ID_TIM3_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH2    DMAMUX_DMAREQ_ID_TIM3_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH3    DMAMUX_DMAREQ_ID_TIM3_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH4    DMAMUX_DMAREQ_ID_TIM3_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH1    DMAMUX_DMAREQ_ID_TIM4_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH2    DMAMUX_DMAREQ_ID_TIM4_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH3    DMAMUX_DMAREQ_ID_TIM4_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH4    DMAMUX_DMAREQ_ID_TIM4_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH1    DMAMUX_DMAREQ_ID_TIM5_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH2    DMAMUX_DMAREQ_ID_TIM5_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH3    DMAMUX_DMAREQ_ID_TIM5_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH4    DMAMUX_DMAREQ_ID_TIM5_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH1    DMAMUX_DMAREQ_ID_TIM8_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH2    DMAMUX_DMAREQ_ID_TIM8_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH3    DMAMUX_DMAREQ_ID_TIM8_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH4    DMAMUX_DMAREQ_ID_TIM8_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM9_CH1    DMAMUX_DMAREQ_ID_TIM9_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM9_CH2    DMAMUX_DMAREQ_ID_TIM9_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM9_CH3    DMAMUX_DMAREQ_ID_TIM9_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM9_CH4    DMAMUX_DMAREQ_ID_TIM9_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM10_CH1   DMAMUX_DMAREQ_ID_TIM10_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM10_CH2   DMAMUX_DMAREQ_ID_TIM10_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM10_CH3   DMAMUX_DMAREQ_ID_TIM10_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM10_CH4   DMAMUX_DMAREQ_ID_TIM10_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM11_CH1   DMAMUX_DMAREQ_ID_TIM11_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM11_CH2   DMAMUX_DMAREQ_ID_TIM11_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM11_CH3   DMAMUX_DMAREQ_ID_TIM11_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM11_CH4   DMAMUX_DMAREQ_ID_TIM11_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM12_CH1   DMAMUX_DMAREQ_ID_TIM12_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM12_CH2   DMAMUX_DMAREQ_ID_TIM12_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM12_CH3   DMAMUX_DMAREQ_ID_TIM12_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM12_CH4   DMAMUX_DMAREQ_ID_TIM12_CH4


#define DEF_TIM_DMA_REQ__BTCH_TIM13_CH1   DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TIM14_CH1   DMA_REQUEST_NONE

// #define DEF_TIM_DMA_REQ__BTCH_TIM20_CH1   DMAMUX_DMAREQ_ID_TMR20_CH1
// #define DEF_TIM_DMA_REQ__BTCH_TIM20_CH2   DMAMUX_DMAREQ_ID_TMR20_CH2
// #define DEF_TIM_DMA_REQ__BTCH_TIM20_CH3   DMAMUX_DMAREQ_ID_TMR20_CH3
// #define DEF_TIM_DMA_REQ__BTCH_TIM20_CH4   DMAMUX_DMAREQ_ID_TMR20_CH4

// TIM_UP request table
#define DEF_TIM_DMA_REQ__BTCH_TIM1_UP     DMAMUX_DMAREQ_ID_TIM1_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM2_UP     DMAMUX_DMAREQ_ID_TIM2_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM3_UP     DMAMUX_DMAREQ_ID_TIM3_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM4_UP     DMAMUX_DMAREQ_ID_TIM4_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM5_UP     DMAMUX_DMAREQ_ID_TIM5_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM8_UP     DMAMUX_DMAREQ_ID_TIM8_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM9_UP     DMAMUX_DMAREQ_ID_TIM9_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM10_UP    DMAMUX_DMAREQ_ID_TIM10_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM11_UP    DMAMUX_DMAREQ_ID_TIM11_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM12_UP    DMAMUX_DMAREQ_ID_TIM12_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM13_UP    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TIM14_UP    DMA_REQUEST_NONE

// #define DEF_TIM_DMA_REQ__BTCH_TIM20_UP    DMAMUX_DMAREQ_ID_TIM20_UP

// AF table for timer ,default is GPIO_MUX_1  should be check after debug

//NONE d(mux_id, timerid)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH1     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH2     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH3     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH4     D(1, 1)
//8x USE TIM3
#define DEF_TIM_AF__NONE__TCH_TIM3_CH1     D(1, 3)
#define DEF_TIM_AF__NONE__TCH_TIM3_CH2     D(1, 3)
#define DEF_TIM_AF__NONE__TCH_TIM3_CH3     D(1, 3)
#define DEF_TIM_AF__NONE__TCH_TIM3_CH4     D(1, 3)
//4x USE TIM8
#define DEF_TIM_AF__NONE__TCH_TIM8_CH1     D(1, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH2     D(1, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH3     D(1, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH4     D(1, 8)



//TIM1 MUX  d(mux_id, timerid)
#define DEF_TIM_AF__PE9__TCH_TIM1_CH1      D(1, 1)
#define DEF_TIM_AF__PA8__TCH_TIM1_CH1      D(1, 1)

#define DEF_TIM_AF__PE11__TCH_TIM1_CH2     D(1, 1)
#define DEF_TIM_AF__PA9__TCH_TIM1_CH2      D(1, 1)

#define DEF_TIM_AF__PE13__TCH_TIM1_CH3     D(1, 1)
#define DEF_TIM_AF__PA10__TCH_TIM1_CH3     D(1, 1)

#define DEF_TIM_AF__PE14__TCH_TIM1_CH4     D(1, 1)
#define DEF_TIM_AF__PA11__TCH_TIM1_CH4     D(1, 1)

#define DEF_TIM_AF__PE8__TCH_TIM1_CH1N     D(1, 1)
#define DEF_TIM_AF__PA7__TCH_TIM1_CH1N     D(1, 1)
#define DEF_TIM_AF__PB13__TCH_TIM1_CH1N    D(1, 1)

#define DEF_TIM_AF__PE10__TCH_TIM1_CH2N    D(1, 1)
#define DEF_TIM_AF__PB0__TCH_TIM1_CH2N     D(1, 1)
#define DEF_TIM_AF__PB14__TCH_TIM1_CH2N    D(1, 1)

#define DEF_TIM_AF__PE12__TCH_TIM1_CH3N    D(1, 1)
#define DEF_TIM_AF__PB1__TCH_TIM1_CH3N     D(1, 1)
#define DEF_TIM_AF__PB15__TCH_TIM1_CH3N    D(1, 1)

//TIM8 MUX  d(mux_id, timerid)
#define DEF_TIM_AF__PE3__TCH_TIM8_CH1      D(0, 8)
#define DEF_TIM_AF__PE4__TCH_TIM8_CH2      D(0, 8)
#define DEF_TIM_AF__PE5__TCH_TIM8_CH3      D(0, 8)
#define DEF_TIM_AF__PE6__TCH_TIM8_CH4      D(0, 8)

#define DEF_TIM_AF__PC6__TCH_TIM8_CH1      D(3, 8)
#define DEF_TIM_AF__PC7__TCH_TIM8_CH2      D(3, 8)
#define DEF_TIM_AF__PC8__TCH_TIM8_CH3      D(3, 8)
#define DEF_TIM_AF__PC9__TCH_TIM8_CH4      D(3, 8)

#define DEF_TIM_AF__PC1__TCH_TIM8_CH1N     D(0, 8)
#define DEF_TIM_AF__PC2__TCH_TIM8_CH2N     D(0, 8)
#define DEF_TIM_AF__PC3__TCH_TIM8_CH3N     D(0, 8)

#define DEF_TIM_AF__PA5__TCH_TIM8_CH1N     D(3, 8)
#define DEF_TIM_AF__PB14__TCH_TIM8_CH2N    D(3, 8)
#define DEF_TIM_AF__PB15__TCH_TIM8_CH3N    D(3, 8)

#define DEF_TIM_AF__PA7__TCH_TIM8_CH1N     D(3, 8)
#define DEF_TIM_AF__PB0__TCH_TIM8_CH2N     D(3, 8)
#define DEF_TIM_AF__PB1__TCH_TIM8_CH3N     D(3, 8)


//TIM2 MUX  d(mux_id, timerid)
#define DEF_TIM_AF__PA0__TCH_TIM2_CH1      D(1, 2)
#define DEF_TIM_AF__PA5__TCH_TIM2_CH1      D(1, 2)
#define DEF_TIM_AF__PA15__TCH_TIM2_CH1     D(1, 2)

#define DEF_TIM_AF__PA1__TCH_TIM2_CH2      D(1, 2)
#define DEF_TIM_AF__PB3__TCH_TIM2_CH2      D(1, 2)

#define DEF_TIM_AF__PA2__TCH_TIM2_CH3      D(1, 2)
#define DEF_TIM_AF__PB10__TCH_TIM2_CH3     D(1, 2)

#define DEF_TIM_AF__PA3__TCH_TIM2_CH4      D(1, 2)
#define DEF_TIM_AF__PB11__TCH_TIM2_CH4     D(1, 2)

//TIM3 MUX  d(mux_id, timerid)
#define DEF_TIM_AF__PA6__TCH_TIM3_CH1      D(2, 3)
#define DEF_TIM_AF__PB4__TCH_TIM3_CH1      D(2, 3)
#define DEF_TIM_AF__PC6__TCH_TIM3_CH1      D(2, 3)
#define DEF_TIM_AF__PD3__TCH_TIM3_CH1      D(9, 3)

#define DEF_TIM_AF__PA7__TCH_TIM3_CH2      D(2, 3)
#define DEF_TIM_AF__PB5__TCH_TIM3_CH2      D(2, 3)
#define DEF_TIM_AF__PC7__TCH_TIM3_CH2      D(2, 3)
#define DEF_TIM_AF__PD4__TCH_TIM3_CH2      D(9, 3)

#define DEF_TIM_AF__PB0__TCH_TIM3_CH3      D(2, 3)
#define DEF_TIM_AF__PC8__TCH_TIM3_CH3      D(2, 3)
#define DEF_TIM_AF__PD5__TCH_TIM3_CH3      D(9, 3)

#define DEF_TIM_AF__PB1__TCH_TIM3_CH4      D(2, 3)
#define DEF_TIM_AF__PC9__TCH_TIM3_CH4      D(2, 3)
#define DEF_TIM_AF__PD6__TCH_TIM3_CH4      D(9, 3)

//TIM4 MUX  d(mux_id, timerid)
#define DEF_TIM_AF__PB6__TCH_TIM4_CH1      D(2, 4)
#define DEF_TIM_AF__PD12__TCH_TIM4_CH1     D(2, 4)
#define DEF_TIM_AF__PE3__TCH_TIM4_CH1      D(2, 4)

#define DEF_TIM_AF__PB7__TCH_TIM4_CH2      D(2, 4)
#define DEF_TIM_AF__PD13__TCH_TIM4_CH2     D(2, 4)
#define DEF_TIM_AF__PE4__TCH_TIM4_CH2      D(2, 4)

#define DEF_TIM_AF__PB8__TCH_TIM4_CH3      D(2, 4)
#define DEF_TIM_AF__PD14__TCH_TIM4_CH3     D(2, 4)
#define DEF_TIM_AF__PE5__TCH_TIM4_CH3      D(2, 4)

#define DEF_TIM_AF__PB9__TCH_TIM4_CH4      D(2, 4)
#define DEF_TIM_AF__PD15__TCH_TIM4_CH4     D(2, 4)
#define DEF_TIM_AF__PE6__TCH_TIM4_CH4      D(2, 4)

//TIM5 MUX  d(mux_id, timerid)
#define DEF_TIM_AF__PA0__TCH_TIM5_CH1      D(2, 5)
#define DEF_TIM_AF__PD12__TCH_TIM5_CH1     D(6, 5)
#define DEF_TIM_AF__PC1__TCH_TIM5_CH1      D(2, 5)

#define DEF_TIM_AF__PA1__TCH_TIM5_CH2      D(2, 5)
#define DEF_TIM_AF__PD13__TCH_TIM5_CH2     D(6, 5)
#define DEF_TIM_AF__PC2__TCH_TIM5_CH2      D(2, 5)

#define DEF_TIM_AF__PA2__TCH_TIM5_CH3      D(2, 5)
#define DEF_TIM_AF__PD14__TCH_TIM5_CH3     D(6, 5)
#define DEF_TIM_AF__PC3__TCH_TIM5_CH3      D(2, 5)

#define DEF_TIM_AF__PA3__TCH_TIM5_CH4      D(2, 5)
#define DEF_TIM_AF__PD15__TCH_TIM5_CH4     D(6, 5)
#define DEF_TIM_AF__PB0__TCH_TIM5_CH4      D(4, 5)

//TIM9 MUX  d(mux_id, timerid)
#define DEF_TIM_AF__PB14__TCH_TIM9_CH1     D(2, 9)
#define DEF_TIM_AF__PA0__TCH_TIM9_CH1      D(6, 9)
#define DEF_TIM_AF__PC9__TCH_TIM9_CH1      D(6, 9)

#define DEF_TIM_AF__PB15__TCH_TIM9_CH2     D(2, 9)
#define DEF_TIM_AF__PA1__TCH_TIM9_CH2      D(6, 9)
#define DEF_TIM_AF__PC10__TCH_TIM9_CH2     D(2, 9)
#define DEF_TIM_AF__PB10__TCH_TIM9_CH2     D(2, 9)

#define DEF_TIM_AF__PE5__TCH_TIM9_CH3      D(4, 9)
#define DEF_TIM_AF__PA2__TCH_TIM9_CH3      D(4, 9)
#define DEF_TIM_AF__PC12__TCH_TIM9_CH3     D(2, 9)
#define DEF_TIM_AF__PB12__TCH_TIM9_CH3     D(8, 9)

#define DEF_TIM_AF__PE6__TCH_TIM9_CH4      D(4, 9)
#define DEF_TIM_AF__PA3__TCH_TIM9_CH4      D(4, 9)
#define DEF_TIM_AF__PC11__TCH_TIM9_CH4     D(2, 9)
#define DEF_TIM_AF__PB11__TCH_TIM9_CH4     D(9, 9)

//TIM10 MUX  d(mux_id, timerid)
#define DEF_TIM_AF__PF8__TCH_TIM10_CH1     D(9, 10)
#define DEF_TIM_AF__PA6__TCH_TIM10_CH1     D(9, 10)
#define DEF_TIM_AF__PB6__TCH_TIM10_CH1     D(0, 10)

#define DEF_TIM_AF__PF9__TCH_TIM10_CH2     D(9, 10)
#define DEF_TIM_AF__PA7__TCH_TIM10_CH2     D(9, 10)
#define DEF_TIM_AF__PB7__TCH_TIM10_CH2     D(0, 10)

#define DEF_TIM_AF__PF6__TCH_TIM10_CH3     D(9, 10)
#define DEF_TIM_AF__PA3__TCH_TIM10_CH3     D(8, 10)
#define DEF_TIM_AF__PB8__TCH_TIM10_CH3     D(1, 10)

#define DEF_TIM_AF__PF7__TCH_TIM10_CH4     D(9, 10)
#define DEF_TIM_AF__PA4__TCH_TIM10_CH4     D(9, 10)
#define DEF_TIM_AF__PB9__TCH_TIM10_CH4     D(1, 10)

//TIM11 MUX  d(mux_id, timerid)
#define DEF_TIM_AF__PD3__TCH_TIM11_CH1     D( 2, 11)
#define DEF_TIM_AF__PF6__TCH_TIM11_CH1     D(13, 11)
#define DEF_TIM_AF__PE0__TCH_TIM11_CH1     D(13, 11)

#define DEF_TIM_AF__PD4__TCH_TIM11_CH2     D( 2, 11)
#define DEF_TIM_AF__PF7__TCH_TIM11_CH2     D(13, 11)
#define DEF_TIM_AF__PE1__TCH_TIM11_CH2     D(13, 11)

#define DEF_TIM_AF__PD5__TCH_TIM11_CH3     D( 2, 11)
#define DEF_TIM_AF__PF8__TCH_TIM11_CH3     D(13, 11)
#define DEF_TIM_AF__PD7__TCH_TIM11_CH3     D(13, 11)

#define DEF_TIM_AF__PD6__TCH_TIM11_CH4     D( 2, 11)
#define DEF_TIM_AF__PF9__TCH_TIM11_CH4     D(13, 11)


//TIM12 MUX  d(mux_id, timerid)
#define DEF_TIM_AF__PB1__TCH_TIM12_CH1     D(5, 12)
#define DEF_TIM_AF__PE3__TCH_TIM12_CH1     D(3, 12)

#define DEF_TIM_AF__PB2__TCH_TIM12_CH2     D(5, 12)
#define DEF_TIM_AF__PE4__TCH_TIM12_CH2     D(3, 12)
#define DEF_TIM_AF__PE13__TCH_TIM12_CH2    D(2, 12)

#define DEF_TIM_AF__PF12__TCH_TIM12_CH3    D(13, 12)
#define DEF_TIM_AF__PE5__TCH_TIM12_CH3     D(3, 12)
#define DEF_TIM_AF__PE14__TCH_TIM12_CH3    D(2, 12)

#define DEF_TIM_AF__PF13__TCH_TIM12_CH4    D(13, 12)
#define DEF_TIM_AF__PE6__TCH_TIM12_CH4     D(3, 12)
#define DEF_TIM_AF__PE15__TCH_TIM12_CH4    D(2, 12)
