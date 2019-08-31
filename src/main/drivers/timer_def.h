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

#pragma once

#include "platform.h"
#include "common/utils.h"

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
#ifdef STM32G4
#define BTCH_TIM1_CH4N BTCH_TIM1_CH4
#endif

#define BTCH_TIM8_CH1N BTCH_TIM8_CH1
#define BTCH_TIM8_CH2N BTCH_TIM8_CH2
#define BTCH_TIM8_CH3N BTCH_TIM8_CH3
#ifdef STM32G4
#define BTCH_TIM8_CH4N BTCH_TIM8_CH4
#endif

#define BTCH_TIM20_CH1N BTCH_TIM20_CH1
#define BTCH_TIM20_CH2N BTCH_TIM20_CH2
#define BTCH_TIM20_CH3N BTCH_TIM20_CH3

#define BTCH_TIM13_CH1N BTCH_TIM13_CH1
#define BTCH_TIM14_CH1N BTCH_TIM14_CH1
#define BTCH_TIM15_CH1N BTCH_TIM15_CH1
#define BTCH_TIM16_CH1N BTCH_TIM16_CH1
#define BTCH_TIM17_CH1N BTCH_TIM17_CH1

// channel table D(chan_n, n_type)
#define DEF_TIM_CH_GET(ch) CONCAT2(DEF_TIM_CH__, ch)
#define DEF_TIM_CH__CH_CH1  D(1, 0)
#define DEF_TIM_CH__CH_CH2  D(2, 0)
#define DEF_TIM_CH__CH_CH3  D(3, 0)
#define DEF_TIM_CH__CH_CH4  D(4, 0)
#define DEF_TIM_CH__CH_CH1N D(1, 1)
#define DEF_TIM_CH__CH_CH2N D(2, 1)
#define DEF_TIM_CH__CH_CH3N D(3, 1)
#ifdef STM32G4
#define DEF_TIM_CH__CH_CH4N D(4, 1)
#endif

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

// define output type (N-channel)
#define DEF_TIM_OUTPUT(ch)         CONCAT(DEF_TIM_OUTPUT__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_OUTPUT__D(chan_n, n_channel) PP_IIF(n_channel, TIMER_OUTPUT_N_CHANNEL, TIMER_OUTPUT_NONE)

#if defined(STM32F1)

#define DEF_TIM(tim, chan, pin, flags, out) {                           \
    tim,                                                                \
    TIMER_GET_IO_TAG(pin),                                              \
    DEF_TIM_CHANNEL(CH_ ## chan),                                       \
    flags,                                                              \
    (DEF_TIM_OUTPUT(CH_ ## chan) | out)                                 \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_CHANNEL(TCH_## tim ## _ ## chan)                    \
    )                                                                   \
    }                                                                   \
/**/

#define DEF_TIM_CHANNEL(ch)        CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) TIM_Channel_ ## chan_n

#define DEF_TIM_DMA_CHANNEL(timch) CONCAT(DEF_TIM_DMA_CHANNEL__, DEF_TIM_DMA_GET(0, timch))
#define DEF_TIM_DMA_CHANNEL__D(dma_n, chan_n) DMA ## dma_n ## _Channel ## chan_n
#define DEF_TIM_DMA_CHANNEL__NONE NULL

/* add F1 DMA mappings here */
// D(dma_n, channel_n)
#define DEF_TIM_DMA__BTCH_TIM1_CH1    D(1, 2)
#define DEF_TIM_DMA__BTCH_TIM1_CH2    NONE
#define DEF_TIM_DMA__BTCH_TIM1_CH3    D(1, 6)
#define DEF_TIM_DMA__BTCH_TIM1_CH4    D(1, 4)

#define DEF_TIM_DMA__BTCH_TIM2_CH1    D(1, 5)
#define DEF_TIM_DMA__BTCH_TIM2_CH2    D(1, 7)
#define DEF_TIM_DMA__BTCH_TIM2_CH3    D(1, 1)
#define DEF_TIM_DMA__BTCH_TIM2_CH4    D(1, 7)

#define DEF_TIM_DMA__BTCH_TIM3_CH1    D(1, 6)
#define DEF_TIM_DMA__BTCH_TIM3_CH2    NONE
#define DEF_TIM_DMA__BTCH_TIM3_CH3    D(1, 2)
#define DEF_TIM_DMA__BTCH_TIM3_CH4    D(1, 3)

#define DEF_TIM_DMA__BTCH_TIM4_CH1    D(1, 1)
#define DEF_TIM_DMA__BTCH_TIM4_CH2    D(1, 4)
#define DEF_TIM_DMA__BTCH_TIM4_CH3    D(1, 5)
#define DEF_TIM_DMA__BTCH_TIM4_CH4    NONE

#elif defined(STM32F3)

#define DEF_TIM(tim, chan, pin, flags, out) {                           \
    tim,                                                                \
    TIMER_GET_IO_TAG(pin),                                              \
    DEF_TIM_CHANNEL(CH_ ## chan),                                       \
    flags,                                                              \
    (DEF_TIM_OUTPUT(CH_ ## chan) | out),                                \
    DEF_TIM_AF(TCH_## tim ## _ ## chan, pin)                            \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_CHANNEL(TCH_## tim ## _ ## chan)                    \
    )                                                                   \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_CHANNEL(TCH_## tim ## _UP),                         \
        DEF_TIM_DMA_HANDLER(TCH_## tim ## _UP)                          \
    )                                                                   \
    }                                                                   \
/**/

#define DEF_TIM_CHANNEL(ch)        CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) TIM_Channel_ ## chan_n

#define DEF_TIM_AF(timch, pin)     CONCAT(DEF_TIM_AF__, DEF_TIM_AF_GET(timch, pin))
#define DEF_TIM_AF__D(af_n) GPIO_AF_ ## af_n

#define DEF_TIM_DMA_CHANNEL(timch) CONCAT(DEF_TIM_DMA_CHANNEL__, DEF_TIM_DMA_GET(0, timch))
#define DEF_TIM_DMA_CHANNEL__D(dma_n, chan_n) (dmaResource_t *)DMA ## dma_n ## _Channel ## chan_n
#define DEF_TIM_DMA_CHANNEL__NONE NULL

#define DEF_TIM_DMA_HANDLER(timch) CONCAT(DEF_TIM_DMA_HANDLER__, DEF_TIM_DMA_GET(0, timch))
#define DEF_TIM_DMA_HANDLER__D(dma_n, chan_n) DMA ## dma_n ## _CH ## chan_n ## _HANDLER
#define DEF_TIM_DMA_HANDLER__NONE 0


/* add the DMA mappings here */
// D(dma_n, channel_n)

#define DEF_TIM_DMA__BTCH_TIM1_CH1    D(1, 2)
#define DEF_TIM_DMA__BTCH_TIM1_CH2    D(1, 3)
#define DEF_TIM_DMA__BTCH_TIM1_CH4    D(1, 4)
#define DEF_TIM_DMA__BTCH_TIM1_TRIG   D(1, 4)
#define DEF_TIM_DMA__BTCH_TIM1_COM    D(1, 4)
#define DEF_TIM_DMA__BTCH_TIM1_CH3    D(1, 6)

#define DEF_TIM_DMA__BTCH_TIM2_CH3    D(1, 1)
#define DEF_TIM_DMA__BTCH_TIM2_CH1    D(1, 5)
#define DEF_TIM_DMA__BTCH_TIM2_CH2    D(1, 7)
#define DEF_TIM_DMA__BTCH_TIM2_CH4    D(1, 7)

#define DEF_TIM_DMA__BTCH_TIM3_CH2    NONE
#define DEF_TIM_DMA__BTCH_TIM3_CH3    D(1, 2)
#define DEF_TIM_DMA__BTCH_TIM3_CH4    D(1, 3)
#define DEF_TIM_DMA__BTCH_TIM3_CH1    D(1, 6)
#define DEF_TIM_DMA__BTCH_TIM3_TRIG   D(1, 6)

#define DEF_TIM_DMA__BTCH_TIM4_CH1    D(1, 1)
#define DEF_TIM_DMA__BTCH_TIM4_CH2    D(1, 4)
#define DEF_TIM_DMA__BTCH_TIM4_CH3    D(1, 5)
#define DEF_TIM_DMA__BTCH_TIM4_CH4    NONE

#define DEF_TIM_DMA__BTCH_TIM15_CH1   D(1, 5)
#define DEF_TIM_DMA__BTCH_TIM15_CH2   NONE
#define DEF_TIM_DMA__BTCH_TIM15_UP    D(1, 5)
#define DEF_TIM_DMA__BTCH_TIM15_TRIG  D(1, 5)
#define DEF_TIM_DMA__BTCH_TIM15_COM   D(1, 5)

#ifdef REMAP_TIM16_DMA
#define DEF_TIM_DMA__BTCH_TIM16_CH1   D(1, 6)
#else
#define DEF_TIM_DMA__BTCH_TIM16_CH1   D(1, 3)
#endif

#ifdef REMAP_TIM17_DMA
#define DEF_TIM_DMA__BTCH_TIM17_CH1   D(1, 7)
#else
#define DEF_TIM_DMA__BTCH_TIM17_CH1   D(1, 1)
#endif

#define DEF_TIM_DMA__BTCH_TIM8_CH3    D(2, 1)
#define DEF_TIM_DMA__BTCH_TIM8_CH4    D(2, 2)
#define DEF_TIM_DMA__BTCH_TIM8_TRIG   D(2, 2)
#define DEF_TIM_DMA__BTCH_TIM8_COM    D(2, 2)
#define DEF_TIM_DMA__BTCH_TIM8_CH1    D(2, 3)
#define DEF_TIM_DMA__BTCH_TIM8_CH2    D(2, 5)

// TIM_UP table
#define DEF_TIM_DMA__BTCH_TIM1_UP     D(1, 5)
#define DEF_TIM_DMA__BTCH_TIM2_UP     D(1, 2)
#define DEF_TIM_DMA__BTCH_TIM3_UP     D(1, 3)
#define DEF_TIM_DMA__BTCH_TIM4_UP     D(1, 7)
#define DEF_TIM_DMA__BTCH_TIM6_UP     D(2, 3)
#define DEF_TIM_DMA__BTCH_TIM7_UP     D(2, 4)
#define DEF_TIM_DMA__BTCH_TIM8_UP     D(2, 1)
#define DEF_TIM_DMA__BTCH_TIM15_UP    D(1, 5)
#ifdef REMAP_TIM16_DMA
#define DEF_TIM_DMA__BTCH_TIM16_UP    D(1, 6)
#else
#define DEF_TIM_DMA__BTCH_TIM16_UP    D(1, 3)
#endif
#ifdef REMAP_TIM17_DMA
#define DEF_TIM_DMA__BTCH_TIM17_UP    D(1, 7)
#else
#define DEF_TIM_DMA__BTCH_TIM17_UP    D(1, 1)
#endif

// AF table

#define DEF_TIM_AF__PA0__TCH_TIM2_CH1     D(1)
#define DEF_TIM_AF__PA1__TCH_TIM2_CH2     D(1)
#define DEF_TIM_AF__PA2__TCH_TIM2_CH3     D(1)
#define DEF_TIM_AF__PA3__TCH_TIM2_CH4     D(1)
#define DEF_TIM_AF__PA5__TCH_TIM2_CH1     D(1)
#define DEF_TIM_AF__PA6__TCH_TIM16_CH1    D(1)
#define DEF_TIM_AF__PA7__TCH_TIM17_CH1    D(1)
#define DEF_TIM_AF__PA12__TCH_TIM16_CH1   D(1)
#define DEF_TIM_AF__PA13__TCH_TIM16_CH1N  D(1)
#define DEF_TIM_AF__PA15__TCH_TIM2_CH1    D(1)

#define DEF_TIM_AF__PA4__TCH_TIM3_CH2     D(2)
#define DEF_TIM_AF__PA6__TCH_TIM3_CH1     D(2)
#define DEF_TIM_AF__PA7__TCH_TIM3_CH2     D(2)
#define DEF_TIM_AF__PA15__TCH_TIM8_CH1    D(2)

#define DEF_TIM_AF__PA7__TCH_TIM8_CH1N    D(4)

#define DEF_TIM_AF__PA14__TCH_TIM8_CH2    D(5)

#define DEF_TIM_AF__PA7__TCH_TIM1_CH1N    D(6)
#define DEF_TIM_AF__PA8__TCH_TIM1_CH1     D(6)
#define DEF_TIM_AF__PA9__TCH_TIM1_CH2     D(6)
#define DEF_TIM_AF__PA10__TCH_TIM1_CH3    D(6)
#define DEF_TIM_AF__PA11__TCH_TIM1_CH1N   D(6)
#define DEF_TIM_AF__PA12__TCH_TIM1_CH2N   D(6)

#define DEF_TIM_AF__PA1__TCH_TIM15_CH1N   D(9)
#define DEF_TIM_AF__PA2__TCH_TIM15_CH1    D(9)
#define DEF_TIM_AF__PA3__TCH_TIM15_CH2    D(9)

#define DEF_TIM_AF__PA9__TCH_TIM2_CH3     D(10)
#define DEF_TIM_AF__PA10__TCH_TIM2_CH4    D(10)
#define DEF_TIM_AF__PA11__TCH_TIM4_CH1    D(10)
#define DEF_TIM_AF__PA12__TCH_TIM4_CH2    D(10)
#define DEF_TIM_AF__PA13__TCH_TIM4_CH3    D(10)
#define DEF_TIM_AF__PA11__TCH_TIM1_CH4    D(11)

#define DEF_TIM_AF__PB3__TCH_TIM2_CH2     D(1)
#define DEF_TIM_AF__PB4__TCH_TIM16_CH1    D(1)
#define DEF_TIM_AF__PB6__TCH_TIM16_CH1N   D(1)
#define DEF_TIM_AF__PB7__TCH_TIM17_CH1N   D(1)
#define DEF_TIM_AF__PB8__TCH_TIM16_CH1    D(1)
#define DEF_TIM_AF__PB9__TCH_TIM17_CH1    D(1)
#define DEF_TIM_AF__PB10__TCH_TIM2_CH3    D(1)
#define DEF_TIM_AF__PB11__TCH_TIM2_CH4    D(1)
#define DEF_TIM_AF__PB14__TCH_TIM15_CH1   D(1)
#define DEF_TIM_AF__PB15__TCH_TIM15_CH2   D(1)

#define DEF_TIM_AF__PB0__TCH_TIM3_CH3     D(2)
#define DEF_TIM_AF__PB1__TCH_TIM3_CH4     D(2)
#define DEF_TIM_AF__PB4__TCH_TIM3_CH1     D(2)
#define DEF_TIM_AF__PB5__TCH_TIM3_CH2     D(2)
#define DEF_TIM_AF__PB6__TCH_TIM4_CH1     D(2)
#define DEF_TIM_AF__PB7__TCH_TIM4_CH2     D(2)
#define DEF_TIM_AF__PB8__TCH_TIM4_CH3     D(2)
#define DEF_TIM_AF__PB9__TCH_TIM4_CH4     D(2)
#define DEF_TIM_AF__PB15__TCH_TIM15_CH1N  D(2)

#define DEF_TIM_AF__PB5__TCH_TIM8_CH3N    D(3)

#define DEF_TIM_AF__PB0__TCH_TIM8_CH2N    D(4)
#define DEF_TIM_AF__PB1__TCH_TIM8_CH3N    D(4)
#define DEF_TIM_AF__PB3__TCH_TIM8_CH1N    D(4)
#define DEF_TIM_AF__PB4__TCH_TIM8_CH2N    D(4)
#define DEF_TIM_AF__PB15__TCH_TIM1_CH3N   D(4)

#define DEF_TIM_AF__PB6__TCH_TIM8_CH1     D(5)

#define DEF_TIM_AF__PB0__TCH_TIM1_CH2N    D(6)
#define DEF_TIM_AF__PB1__TCH_TIM1_CH3N    D(6)
#define DEF_TIM_AF__PB13__TCH_TIM1_CH1N   D(6)
#define DEF_TIM_AF__PB14__TCH_TIM1_CH2N   D(6)

#define DEF_TIM_AF__PB5__TCH_TIM17_CH1    D(10)
#define DEF_TIM_AF__PB7__TCH_TIM3_CH4     D(10)
#define DEF_TIM_AF__PB8__TCH_TIM8_CH2     D(10)
#define DEF_TIM_AF__PB9__TCH_TIM8_CH3     D(10)

#define DEF_TIM_AF__PC6__TCH_TIM3_CH1     D(2)
#define DEF_TIM_AF__PC7__TCH_TIM3_CH2     D(2)
#define DEF_TIM_AF__PC8__TCH_TIM3_CH3     D(2)
#define DEF_TIM_AF__PC9__TCH_TIM3_CH4     D(2)

#define DEF_TIM_AF__PC6__TCH_TIM8_CH1     D(4)
#define DEF_TIM_AF__PC7__TCH_TIM8_CH2     D(4)
#define DEF_TIM_AF__PC8__TCH_TIM8_CH3     D(4)
#define DEF_TIM_AF__PC9__TCH_TIM8_CH4     D(4)

#define DEF_TIM_AF__PC10__TCH_TIM8_CH1N   D(4)
#define DEF_TIM_AF__PC11__TCH_TIM8_CH2N   D(4)
#define DEF_TIM_AF__PC12__TCH_TIM8_CH3N   D(4)
#define DEF_TIM_AF__PC13__TCH_TIM8_CH1N   D(4)

#define DEF_TIM_AF__PD3__TCH_TIM2_CH1     D(2)
#define DEF_TIM_AF__PD4__TCH_TIM2_CH2     D(2)
#define DEF_TIM_AF__PD6__TCH_TIM2_CH4     D(2)
#define DEF_TIM_AF__PD7__TCH_TIM2_CH3     D(2)

#define DEF_TIM_AF__PD12__TCH_TIM4_CH1    D(2)
#define DEF_TIM_AF__PD13__TCH_TIM4_CH2    D(2)
#define DEF_TIM_AF__PD14__TCH_TIM4_CH3    D(2)
#define DEF_TIM_AF__PD15__TCH_TIM4_CH4    D(2)

#define DEF_TIM_AF__PD1__TCH_TIM8_CH4     D(4)

#define DEF_TIM_AF__PF9__TCH_TIM15_CH1    D(3)
#define DEF_TIM_AF__PF10__TCH_TIM15_CH2   D(3)

#elif defined(STM32F4)

#define DEF_TIM(tim, chan, pin, flags, out, dmaopt) {           \
    tim,                                                        \
    TIMER_GET_IO_TAG(pin),                                      \
    DEF_TIM_CHANNEL(CH_ ## chan),                               \
    flags,                                                      \
    (DEF_TIM_OUTPUT(CH_ ## chan) | out),                        \
    DEF_TIM_AF(TIM_ ## tim)                                     \
    DEF_TIM_DMA_COND(/* add comma */ ,                          \
        DEF_TIM_DMA_STREAM(dmaopt, TCH_## tim ## _ ## chan),    \
        DEF_TIM_DMA_CHANNEL(dmaopt, TCH_## tim ## _ ## chan)    \
    )                                                           \
    DEF_TIM_DMA_COND(/* add comma */ ,                          \
        DEF_TIM_DMA_STREAM(0, TCH_## tim ## _UP),               \
        DEF_TIM_DMA_CHANNEL(0, TCH_## tim ## _UP),              \
        DEF_TIM_DMA_HANDLER(0, TCH_## tim ## _UP)               \
    )                                                           \
}                                                               \
/**/

#define DEF_TIM_CHANNEL(ch)                   CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) TIM_Channel_ ## chan_n

#define DEF_TIM_AF(tim)                       CONCAT(DEF_TIM_AF__, DEF_TIM_TIM_GET(tim))
#define DEF_TIM_AF__D(tim_n)                  GPIO_AF_TIM ## tim_n

#define DEF_TIM_DMA_CHANNEL(variant, timch) \
    CONCAT(DEF_TIM_DMA_CHANNEL__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_CHANNEL__D(dma_n, stream_n, chan_n)    DMA_Channel_ ## chan_n
#define DEF_TIM_DMA_CHANNEL__NONE                          DMA_Channel_0

#define DEF_TIM_DMA_STREAM(variant, timch)                              \
    CONCAT(DEF_TIM_DMA_STREAM__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_STREAM__D(dma_n, stream_n, chan_n)  (dmaResource_t *)DMA ## dma_n ## _Stream ## stream_n
#define DEF_TIM_DMA_STREAM__NONE                        NULL

#define DEF_TIM_DMA_HANDLER(variant, timch) \
    CONCAT(DEF_TIM_DMA_HANDLER__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_HANDLER__D(dma_n, stream_n, chan_n) DMA ## dma_n ## _ST ## stream_n ## _HANDLER
#define DEF_TIM_DMA_HANDLER__NONE                       0


/* F4 Stream Mappings */
// D(DMAx, Stream, Channel)
#define DEF_TIM_DMA__BTCH_TIM1_CH1    D(2, 6, 0),D(2, 1, 6),D(2, 3, 6)
#define DEF_TIM_DMA__BTCH_TIM1_CH2    D(2, 6, 0),D(2, 2, 6)
#define DEF_TIM_DMA__BTCH_TIM1_CH3    D(2, 6, 0),D(2, 6, 6)
#define DEF_TIM_DMA__BTCH_TIM1_CH4    D(2, 4, 6)

#define DEF_TIM_DMA__BTCH_TIM2_CH1    D(1, 5, 3)
#define DEF_TIM_DMA__BTCH_TIM2_CH2    D(1, 6, 3)
#define DEF_TIM_DMA__BTCH_TIM2_CH3    D(1, 1, 3)
#define DEF_TIM_DMA__BTCH_TIM2_CH4    D(1, 7, 3),D(1, 6, 3)

#define DEF_TIM_DMA__BTCH_TIM3_CH1    D(1, 4, 5)
#define DEF_TIM_DMA__BTCH_TIM3_CH2    D(1, 5, 5)
#define DEF_TIM_DMA__BTCH_TIM3_CH3    D(1, 7, 5)
#define DEF_TIM_DMA__BTCH_TIM3_CH4    D(1, 2, 5)

#define DEF_TIM_DMA__BTCH_TIM4_CH1    D(1, 0, 2)
#define DEF_TIM_DMA__BTCH_TIM4_CH2    D(1, 3, 2)
#define DEF_TIM_DMA__BTCH_TIM4_CH3    D(1, 7, 2)

#define DEF_TIM_DMA__BTCH_TIM5_CH1    D(1, 2, 6)
#define DEF_TIM_DMA__BTCH_TIM5_CH2    D(1, 4, 6)
#define DEF_TIM_DMA__BTCH_TIM5_CH3    D(1, 0, 6)
#define DEF_TIM_DMA__BTCH_TIM5_CH4    D(1, 1, 6),D(1, 3, 6)

#define DEF_TIM_DMA__BTCH_TIM8_CH1    D(2, 2, 0),D(2, 2, 7)
#define DEF_TIM_DMA__BTCH_TIM8_CH2    D(2, 2, 0),D(2, 3, 7)
#define DEF_TIM_DMA__BTCH_TIM8_CH3    D(2, 2, 0),D(2, 4, 7)
#define DEF_TIM_DMA__BTCH_TIM8_CH4    D(2, 7, 7)

#define DEF_TIM_DMA__BTCH_TIM4_CH4    NONE

#define DEF_TIM_DMA__BTCH_TIM9_CH1    NONE
#define DEF_TIM_DMA__BTCH_TIM9_CH2    NONE

#define DEF_TIM_DMA__BTCH_TIM10_CH1   NONE

#define DEF_TIM_DMA__BTCH_TIM11_CH1   NONE

#define DEF_TIM_DMA__BTCH_TIM12_CH1   NONE
#define DEF_TIM_DMA__BTCH_TIM12_CH2   NONE

#define DEF_TIM_DMA__BTCH_TIM13_CH1   NONE

#define DEF_TIM_DMA__BTCH_TIM14_CH1   NONE

// TIM_UP table
#define DEF_TIM_DMA__BTCH_TIM1_UP     D(2, 5, 6)
#define DEF_TIM_DMA__BTCH_TIM2_UP     D(1, 7, 3)
#define DEF_TIM_DMA__BTCH_TIM3_UP     D(1, 2, 5)
#define DEF_TIM_DMA__BTCH_TIM4_UP     D(1, 6, 2)
#define DEF_TIM_DMA__BTCH_TIM5_UP     D(1, 0, 6)
#define DEF_TIM_DMA__BTCH_TIM6_UP     D(1, 1, 7)
#define DEF_TIM_DMA__BTCH_TIM7_UP     D(1, 4, 1)
#define DEF_TIM_DMA__BTCH_TIM8_UP     D(2, 1, 7)
#define DEF_TIM_DMA__BTCH_TIM9_UP     NONE
#define DEF_TIM_DMA__BTCH_TIM10_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM11_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM12_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM13_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM14_UP    NONE

#elif defined(STM32F7)
#define DEF_TIM(tim, chan, pin, flags, out, dmaopt) {                   \
    tim,                                                                \
    TIMER_GET_IO_TAG(pin),                                              \
    DEF_TIM_CHANNEL(CH_ ## chan),                                       \
    flags,                                                              \
    (DEF_TIM_OUTPUT(CH_ ## chan) | out),                                \
    DEF_TIM_AF(TCH_## tim ## _ ## chan, pin)                            \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_STREAM(dmaopt, TCH_## tim ## _ ## chan),            \
        DEF_TIM_DMA_CHANNEL(dmaopt, TCH_## tim ## _ ## chan)            \
    )                                                                   \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_STREAM(0, TCH_## tim ## _UP),                       \
        DEF_TIM_DMA_CHANNEL(0, TCH_## tim ## _UP),                      \
        DEF_TIM_DMA_HANDLER(0, TCH_## tim ## _UP)                       \
    )                                                                   \
}                                                                       \
/**/

#define DEF_TIM_CHANNEL(ch)                   CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) TIM_CHANNEL_ ## chan_n

#define DEF_TIM_AF(timch, pin)                CONCAT(DEF_TIM_AF__, DEF_TIM_AF_GET(timch, pin))
#define DEF_TIM_AF__D(af_n, tim_n)            GPIO_AF ## af_n ## _TIM ## tim_n

#define DEF_TIM_DMA_CHANNEL(variant, timch) \
    CONCAT(DEF_TIM_DMA_CHANNEL__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_CHANNEL__D(dma_n, stream_n, chan_n)    DMA_CHANNEL_ ## chan_n
#define DEF_TIM_DMA_CHANNEL__NONE                          DMA_CHANNEL_0

#define DEF_TIM_DMA_STREAM(variant, timch)                              \
    CONCAT(DEF_TIM_DMA_STREAM__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_STREAM__D(dma_n, stream_n, chan_n)  (dmaResource_t *)DMA ## dma_n ## _Stream ## stream_n
#define DEF_TIM_DMA_STREAM__NONE                        NULL

#define DEF_TIM_DMA_HANDLER(variant, timch) \
    CONCAT(DEF_TIM_DMA_HANDLER__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_HANDLER__D(dma_n, stream_n, chan_n) DMA ## dma_n ## _ST ## stream_n ## _HANDLER
#define DEF_TIM_DMA_HANDLER__NONE                       0

/* F7 Stream Mappings */
// D(DMAx, Stream, Channel)
#define DEF_TIM_DMA__BTCH_TIM1_CH1    D(2, 6, 0),D(2, 1, 6),D(2, 3, 6)
#define DEF_TIM_DMA__BTCH_TIM1_CH2    D(2, 6, 0),D(2, 2, 6)
#define DEF_TIM_DMA__BTCH_TIM1_CH3    D(2, 6, 0),D(2, 6, 6)
#define DEF_TIM_DMA__BTCH_TIM1_CH4    D(2, 4, 6)

#define DEF_TIM_DMA__BTCH_TIM2_CH1    D(1, 5, 3)
#define DEF_TIM_DMA__BTCH_TIM2_CH2    D(1, 6, 3)
#define DEF_TIM_DMA__BTCH_TIM2_CH3    D(1, 1, 3)
#define DEF_TIM_DMA__BTCH_TIM2_CH4    D(1, 7, 3),D(1, 6, 3)

#define DEF_TIM_DMA__BTCH_TIM3_CH1    D(1, 4, 5)
#define DEF_TIM_DMA__BTCH_TIM3_CH2    D(1, 5, 5)
#define DEF_TIM_DMA__BTCH_TIM3_CH3    D(1, 7, 5)
#define DEF_TIM_DMA__BTCH_TIM3_CH4    D(1, 2, 5)

#define DEF_TIM_DMA__BTCH_TIM4_CH1    D(1, 0, 2)
#define DEF_TIM_DMA__BTCH_TIM4_CH2    D(1, 3, 2)
#define DEF_TIM_DMA__BTCH_TIM4_CH3    D(1, 7, 2)

#define DEF_TIM_DMA__BTCH_TIM5_CH1    D(1, 2, 6)
#define DEF_TIM_DMA__BTCH_TIM5_CH2    D(1, 4, 6)
#define DEF_TIM_DMA__BTCH_TIM5_CH3    D(1, 0, 6)
#define DEF_TIM_DMA__BTCH_TIM5_CH4    D(1, 1, 6),D(1, 3, 6)

#define DEF_TIM_DMA__BTCH_TIM8_CH1    D(2, 2, 0),D(2, 2, 7)
#define DEF_TIM_DMA__BTCH_TIM8_CH2    D(2, 2, 0),D(2, 3, 7)
#define DEF_TIM_DMA__BTCH_TIM8_CH3    D(2, 2, 0),D(2, 4, 7)
#define DEF_TIM_DMA__BTCH_TIM8_CH4    D(2, 7, 7)

#define DEF_TIM_DMA__BTCH_TIM4_CH4    NONE

#define DEF_TIM_DMA__BTCH_TIM9_CH1    NONE
#define DEF_TIM_DMA__BTCH_TIM9_CH2    NONE

#define DEF_TIM_DMA__BTCH_TIM10_CH1   NONE

#define DEF_TIM_DMA__BTCH_TIM11_CH1   NONE

#define DEF_TIM_DMA__BTCH_TIM12_CH1   NONE
#define DEF_TIM_DMA__BTCH_TIM12_CH2   NONE

#define DEF_TIM_DMA__BTCH_TIM13_CH1   NONE

#define DEF_TIM_DMA__BTCH_TIM14_CH1   NONE

// TIM_UP table
#define DEF_TIM_DMA__BTCH_TIM1_UP     D(2, 5, 6)
#define DEF_TIM_DMA__BTCH_TIM2_UP     D(1, 7, 3)
#define DEF_TIM_DMA__BTCH_TIM3_UP     D(1, 2, 5)
#define DEF_TIM_DMA__BTCH_TIM4_UP     D(1, 6, 2)
#define DEF_TIM_DMA__BTCH_TIM5_UP     D(1, 0, 6)
#define DEF_TIM_DMA__BTCH_TIM6_UP     D(1, 1, 7)
#define DEF_TIM_DMA__BTCH_TIM7_UP     D(1, 4, 1)
#define DEF_TIM_DMA__BTCH_TIM8_UP     D(2, 1, 7)
#define DEF_TIM_DMA__BTCH_TIM9_UP     NONE
#define DEF_TIM_DMA__BTCH_TIM10_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM11_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM12_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM13_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM14_UP    NONE

// AF table

// NONE
#define DEF_TIM_AF__NONE__TCH_TIM1_CH1     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH2     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH3     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH4     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH1     D(3, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH2     D(3, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH3     D(3, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH4     D(3, 8)

//PORTA
#define DEF_TIM_AF__PA0__TCH_TIM2_CH1     D(1, 2)
#define DEF_TIM_AF__PA1__TCH_TIM2_CH2     D(1, 2)
#define DEF_TIM_AF__PA2__TCH_TIM2_CH3     D(1, 2)
#define DEF_TIM_AF__PA3__TCH_TIM2_CH4     D(1, 2)
#define DEF_TIM_AF__PA5__TCH_TIM2_CH1     D(1, 2)
#define DEF_TIM_AF__PA7__TCH_TIM1_CH1N    D(1, 1)
#define DEF_TIM_AF__PA8__TCH_TIM1_CH1     D(1, 1)
#define DEF_TIM_AF__PA9__TCH_TIM1_CH2     D(1, 1)
#define DEF_TIM_AF__PA10__TCH_TIM1_CH3    D(1, 1)
#define DEF_TIM_AF__PA11__TCH_TIM1_CH1N   D(1, 1)
#define DEF_TIM_AF__PA15__TCH_TIM2_CH1    D(1, 2)

#define DEF_TIM_AF__PA0__TCH_TIM5_CH1     D(2, 5)
#define DEF_TIM_AF__PA1__TCH_TIM5_CH2     D(2, 5)
#define DEF_TIM_AF__PA2__TCH_TIM5_CH3     D(2, 5)
#define DEF_TIM_AF__PA3__TCH_TIM5_CH4     D(2, 5)
#define DEF_TIM_AF__PA6__TCH_TIM3_CH1     D(2, 3)
#define DEF_TIM_AF__PA7__TCH_TIM3_CH2     D(2, 3)

#define DEF_TIM_AF__PA2__TCH_TIM9_CH1     D(3, 9)
#define DEF_TIM_AF__PA3__TCH_TIM9_CH2     D(3, 9)
#define DEF_TIM_AF__PA5__TCH_TIM8_CH1N    D(3, 8)
#define DEF_TIM_AF__PA7__TCH_TIM8_CH1N    D(3, 8)

#define DEF_TIM_AF__PA6__TCH_TIM13_CH1    D(9, 13)
#define DEF_TIM_AF__PA7__TCH_TIM14_CH1    D(9, 14)

//PORTB
#define DEF_TIM_AF__PB0__TCH_TIM1_CH2N    D(1, 1)
#define DEF_TIM_AF__PB1__TCH_TIM1_CH3N    D(1, 1)
#define DEF_TIM_AF__PB3__TCH_TIM2_CH2     D(1, 2)
#define DEF_TIM_AF__PB10__TCH_TIM2_CH3    D(1, 2)
#define DEF_TIM_AF__PB11__TCH_TIM2_CH4    D(1, 2)
#define DEF_TIM_AF__PB13__TCH_TIM1_CH1N   D(1, 1)
#define DEF_TIM_AF__PB14__TCH_TIM1_CH2N   D(1, 1)
#define DEF_TIM_AF__PB15__TCH_TIM1_CH3N   D(1, 1)

#define DEF_TIM_AF__PB0__TCH_TIM3_CH3     D(2, 3)
#define DEF_TIM_AF__PB1__TCH_TIM3_CH4     D(2, 3)
#define DEF_TIM_AF__PB4__TCH_TIM3_CH1     D(2, 3)
#define DEF_TIM_AF__PB5__TCH_TIM3_CH2     D(2, 3)
#define DEF_TIM_AF__PB6__TCH_TIM4_CH1     D(2, 4)
#define DEF_TIM_AF__PB7__TCH_TIM4_CH2     D(2, 4)
#define DEF_TIM_AF__PB8__TCH_TIM4_CH3     D(2, 4)
#define DEF_TIM_AF__PB9__TCH_TIM4_CH4     D(2, 4)

#define DEF_TIM_AF__PB0__TCH_TIM8_CH2N    D(3, 8)
#define DEF_TIM_AF__PB1__TCH_TIM8_CH3N    D(3, 8)
#define DEF_TIM_AF__PB8__TCH_TIM10_CH1    D(3, 10)
#define DEF_TIM_AF__PB9__TCH_TIM11_CH1    D(3, 11)
#define DEF_TIM_AF__PB14__TCH_TIM8_CH2N   D(3, 8)
#define DEF_TIM_AF__PB15__TCH_TIM8_CH3N   D(3, 8)

#define DEF_TIM_AF__PB14__TCH_TIM12_CH1   D(9, 12)
#define DEF_TIM_AF__PB15__TCH_TIM12_CH2   D(9, 12)

//PORTC
#define DEF_TIM_AF__PC6__TCH_TIM3_CH1     D(2, 3)
#define DEF_TIM_AF__PC7__TCH_TIM3_CH2     D(2, 3)
#define DEF_TIM_AF__PC8__TCH_TIM3_CH3     D(2, 3)
#define DEF_TIM_AF__PC9__TCH_TIM3_CH4     D(2, 3)

#define DEF_TIM_AF__PC6__TCH_TIM8_CH1     D(3, 8)
#define DEF_TIM_AF__PC7__TCH_TIM8_CH2     D(3, 8)
#define DEF_TIM_AF__PC8__TCH_TIM8_CH3     D(3, 8)
#define DEF_TIM_AF__PC9__TCH_TIM8_CH4     D(3, 8)

//PORTD
#define DEF_TIM_AF__PD12__TCH_TIM4_CH1    D(2, 4)
#define DEF_TIM_AF__PD13__TCH_TIM4_CH2    D(2, 4)
#define DEF_TIM_AF__PD14__TCH_TIM4_CH3    D(2, 4)
#define DEF_TIM_AF__PD15__TCH_TIM4_CH4    D(2, 4)

//PORTE
#define DEF_TIM_AF__PE8__TCH_TIM1_CH1N    D(1, 1)
#define DEF_TIM_AF__PE9__TCH_TIM1_CH1     D(1, 1)
#define DEF_TIM_AF__PE10__TCH_TIM1_CH2N   D(1, 1)
#define DEF_TIM_AF__PE11__TCH_TIM1_CH2    D(1, 1)
#define DEF_TIM_AF__PE12__TCH_TIM1_CH3N   D(1, 1)
#define DEF_TIM_AF__PE13__TCH_TIM1_CH3    D(1, 1)
#define DEF_TIM_AF__PE14__TCH_TIM1_CH4    D(1, 1)

#define DEF_TIM_AF__PE5__TCH_TIM9_CH1     D(3, 9)
#define DEF_TIM_AF__PE6__TCH_TIM9_CH2     D(3, 9)

//PORTF
#define DEF_TIM_AF__PF6__TCH_TIM10_CH1    D(3, 10)
#define DEF_TIM_AF__PF7__TCH_TIM11_CH1    D(3, 11)

//PORTH
#define DEF_TIM_AF__PH10__TCH_TIM5_CH1    D(2, 5)
#define DEF_TIM_AF__PH11__TCH_TIM5_CH2    D(2, 5)
#define DEF_TIM_AF__PH12__TCH_TIM5_CH3    D(2, 5)

#define DEF_TIM_AF__PH13__TCH_TIM8_CH1N   D(3, 8)
#define DEF_TIM_AF__PH14__TCH_TIM8_CH2N   D(3, 8)
#define DEF_TIM_AF__PH15__TCH_TIM8_CH3N   D(3, 8)

#define DEF_TIM_AF__PH6__TCH_TIM12_CH1    D(9, 12)
#define DEF_TIM_AF__PH9__TCH_TIM12_CH2    D(9, 12)

//PORTI
#define DEF_TIM_AF__PI0__TCH_TIM5_CH4     D(2, 5)

#define DEF_TIM_AF__PI2__TCH_TIM8_CH4     D(3, 8)
#define DEF_TIM_AF__PI5__TCH_TIM8_CH1     D(3, 8)
#define DEF_TIM_AF__PI6__TCH_TIM8_CH2     D(3, 8)
#define DEF_TIM_AF__PI7__TCH_TIM8_CH3     D(3, 8)

#elif defined(STM32H7)
#define DEF_TIM(tim, chan, pin, flags, out, dmaopt, upopt) {            \
    tim,                                                                \
    TIMER_GET_IO_TAG(pin),                                              \
    DEF_TIM_CHANNEL(CH_ ## chan),                                       \
    flags,                                                              \
    (DEF_TIM_OUTPUT(CH_ ## chan) | out),                                \
    DEF_TIM_AF(TCH_## tim ## _ ## chan, pin)                            \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_STREAM(dmaopt, TCH_## tim ## _ ## chan),            \
        DEF_TIM_DMA_REQUEST(TCH_## tim ## _ ## chan)                    \
    )                                                                   \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_STREAM(upopt, TCH_## tim ## _UP),                   \
        DEF_TIM_DMA_REQUEST(TCH_## tim ## _UP),                         \
        DEF_TIM_DMA_HANDLER(upopt, TCH_## tim ## _UP)                   \
    )                                                                   \
}                                                                       \
/**/

#define DEF_TIM_CHANNEL(ch)                   CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) TIM_CHANNEL_ ## chan_n

#define DEF_TIM_AF(timch, pin)                CONCAT(DEF_TIM_AF__, DEF_TIM_AF_GET(timch, pin))
#define DEF_TIM_AF__D(af_n, tim_n)            GPIO_AF ## af_n ## _TIM ## tim_n

#define DEF_TIM_DMA_STREAM(variant, timch)                              \
    CONCAT(DEF_TIM_DMA_STREAM__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_STREAM__D(dma_n, stream_n)  (dmaResource_t *)DMA ## dma_n ## _Stream ## stream_n
#define DEF_TIM_DMA_STREAM__NONE                        NULL

// XXX This is awful. There must be some smart way of doing this ...
#define DEF_TIM_DMA_REQUEST(timch) \
    CONCAT(DEF_TIM_DMA_REQ__, DEF_TIM_TCH2BTCH(timch))

#define DEF_TIM_DMA_HANDLER(variant, timch) \
    CONCAT(DEF_TIM_DMA_HANDLER__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_HANDLER__D(dma_n, stream_n) DMA ## dma_n ## _ST ## stream_n ## _HANDLER
#define DEF_TIM_DMA_HANDLER__NONE                       0

/* H7 Stream Mappings */
// D(DMAx, Stream)

// H7 has DMAMUX that allow arbitrary assignment of peripherals to streams.

#define DEF_TIM_DMA_FULL \
    D(1, 0), D(1, 1), D(1, 2), D(1, 3), D(1, 4), D(1, 5), D(1, 6), D(1, 7), \
    D(2, 0), D(2, 1), D(2, 2), D(2, 3), D(2, 4), D(2, 5), D(2, 6), D(2, 7)

#define DEF_TIM_DMA__BTCH_TIM1_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM1_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM1_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM1_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM2_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM3_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM3_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM3_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM3_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM4_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM4_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM4_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM4_CH4    NONE

#define DEF_TIM_DMA__BTCH_TIM5_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM5_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM5_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM5_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM8_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM8_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM8_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM8_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM12_CH1   NONE
#define DEF_TIM_DMA__BTCH_TIM12_CH2   NONE

#define DEF_TIM_DMA__BTCH_TIM13_CH1   NONE

#define DEF_TIM_DMA__BTCH_TIM14_CH1   NONE

#define DEF_TIM_DMA__BTCH_TIM15_CH1   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM15_CH2   NONE

#define DEF_TIM_DMA__BTCH_TIM16_CH1   DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM17_CH1   DEF_TIM_DMA_FULL

// TIM_UP table
#define DEF_TIM_DMA__BTCH_TIM1_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM3_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM4_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM5_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM6_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM7_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM8_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM12_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM13_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM14_UP    NONE
#define DEF_TIM_DMA__BTCH_TIM15_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM16_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM17_UP    DEF_TIM_DMA_FULL

// TIMx_CHy request table

// This is not defined in stm32h7xx_hal_timer.h
#define DMA_REQUEST_NONE 255

#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH1    DMA_REQUEST_TIM1_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH2    DMA_REQUEST_TIM1_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH3    DMA_REQUEST_TIM1_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH4    DMA_REQUEST_TIM1_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH1    DMA_REQUEST_TIM2_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH2    DMA_REQUEST_TIM2_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH3    DMA_REQUEST_TIM2_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH4    DMA_REQUEST_TIM2_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH1    DMA_REQUEST_TIM3_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH2    DMA_REQUEST_TIM3_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH3    DMA_REQUEST_TIM3_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH4    DMA_REQUEST_TIM3_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH1    DMA_REQUEST_TIM4_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH2    DMA_REQUEST_TIM4_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH3    DMA_REQUEST_TIM4_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH4    DMA_REQUEST_NONE

#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH1    DMA_REQUEST_TIM5_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH2    DMA_REQUEST_TIM5_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH3    DMA_REQUEST_TIM5_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH4    DMA_REQUEST_TIM5_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH1    DMA_REQUEST_TIM8_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH2    DMA_REQUEST_TIM8_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH3    DMA_REQUEST_TIM8_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH4    DMA_REQUEST_TIM8_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM12_CH1   DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TIM12_CH2   DMA_REQUEST_NONE

#define DEF_TIM_DMA_REQ__BTCH_TIM13_CH1   DMA_REQUEST_NONE

#define DEF_TIM_DMA_REQ__BTCH_TIM14_CH1   DMA_REQUEST_NONE

#define DEF_TIM_DMA_REQ__BTCH_TIM15_CH1   DMA_REQUEST_TIM15_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM15_CH2   DMA_REQUEST_NONE

#define DEF_TIM_DMA_REQ__BTCH_TIM16_CH1   DMA_REQUEST_TIM16_CH1

#define DEF_TIM_DMA_REQ__BTCH_TIM17_CH1   DMA_REQUEST_TIM17_CH1

// TIM_UP request table
#define DEF_TIM_DMA_REQ__BTCH_TIM1_UP     DMA_REQUEST_TIM1_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM2_UP     DMA_REQUEST_TIM2_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM3_UP     DMA_REQUEST_TIM3_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM4_UP     DMA_REQUEST_TIM4_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM5_UP     DMA_REQUEST_TIM5_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM6_UP     DMA_REQUEST_TIM6_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM7_UP     DMA_REQUEST_TIM7_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM8_UP     DMA_REQUEST_TIM8_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM12_UP    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TIM13_UP    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TIM14_UP    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TIM15_UP    DMA_REQUEST_TIM15_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM16_UP    DMA_REQUEST_TIM16_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM17_UP    DMA_REQUEST_TIM17_UP

// AF table

// NONE
#define DEF_TIM_AF__NONE__TCH_TIM1_CH1     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH2     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH3     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH4     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH1     D(3, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH2     D(3, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH3     D(3, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH4     D(3, 8)

//PORTA
#define DEF_TIM_AF__PA0__TCH_TIM2_CH1     D(1, 2)
#define DEF_TIM_AF__PA1__TCH_TIM2_CH2     D(1, 2)
#define DEF_TIM_AF__PA2__TCH_TIM2_CH3     D(1, 2)
#define DEF_TIM_AF__PA3__TCH_TIM2_CH4     D(1, 2)
#define DEF_TIM_AF__PA5__TCH_TIM2_CH1     D(1, 2)
#define DEF_TIM_AF__PA7__TCH_TIM1_CH1N    D(1, 1)
#define DEF_TIM_AF__PA8__TCH_TIM1_CH1     D(1, 1)
#define DEF_TIM_AF__PA9__TCH_TIM1_CH2     D(1, 1)
#define DEF_TIM_AF__PA10__TCH_TIM1_CH3    D(1, 1)
#define DEF_TIM_AF__PA11__TCH_TIM1_CH4    D(1, 1)
#define DEF_TIM_AF__PA15__TCH_TIM2_CH1    D(1, 2)

#define DEF_TIM_AF__PA0__TCH_TIM5_CH1     D(2, 5)
#define DEF_TIM_AF__PA1__TCH_TIM5_CH2     D(2, 5)
#define DEF_TIM_AF__PA2__TCH_TIM5_CH3     D(2, 5)
#define DEF_TIM_AF__PA3__TCH_TIM5_CH4     D(2, 5)
#define DEF_TIM_AF__PA6__TCH_TIM3_CH1     D(2, 3)
#define DEF_TIM_AF__PA7__TCH_TIM3_CH2     D(2, 3)

#define DEF_TIM_AF__PA5__TCH_TIM8_CH1N    D(3, 8)
#define DEF_TIM_AF__PA7__TCH_TIM8_CH1N    D(3, 8)

#define DEF_TIM_AF__PA6__TCH_TIM13_CH1    D(9, 13)
#define DEF_TIM_AF__PA7__TCH_TIM14_CH1    D(9, 14)

#define DEF_TIM_AF__PA1__TCH_TIM15_CH1N   D(4, 15)
#define DEF_TIM_AF__PA2__TCH_TIM15_CH1    D(4, 15)
#define DEF_TIM_AF__PA2__TCH_TIM15_CH2    D(4, 15)

//PORTB
#define DEF_TIM_AF__PB0__TCH_TIM1_CH2N    D(1, 1)
#define DEF_TIM_AF__PB1__TCH_TIM1_CH3N    D(1, 1)
#define DEF_TIM_AF__PB3__TCH_TIM2_CH2     D(1, 2)
#define DEF_TIM_AF__PB6__TCH_TIM16_CH1N   D(1, 16)
#define DEF_TIM_AF__PB7__TCH_TIM17_CH1N   D(1, 17)
#define DEF_TIM_AF__PB8__TCH_TIM16_CH1    D(1, 16)
#define DEF_TIM_AF__PB9__TCH_TIM17_CH1    D(1, 17)
#define DEF_TIM_AF__PB10__TCH_TIM2_CH3    D(1, 2)
#define DEF_TIM_AF__PB11__TCH_TIM2_CH4    D(1, 2)
#define DEF_TIM_AF__PB13__TCH_TIM1_CH1N   D(1, 1)
#define DEF_TIM_AF__PB14__TCH_TIM1_CH2N   D(1, 1)
#define DEF_TIM_AF__PB15__TCH_TIM1_CH3N   D(1, 1)

#define DEF_TIM_AF__PB0__TCH_TIM3_CH3     D(2, 3)
#define DEF_TIM_AF__PB1__TCH_TIM3_CH4     D(2, 3)
#define DEF_TIM_AF__PB4__TCH_TIM3_CH1     D(2, 3)
#define DEF_TIM_AF__PB5__TCH_TIM3_CH2     D(2, 3)
#define DEF_TIM_AF__PB6__TCH_TIM4_CH1     D(2, 4)
#define DEF_TIM_AF__PB7__TCH_TIM4_CH2     D(2, 4)
#define DEF_TIM_AF__PB8__TCH_TIM4_CH3     D(2, 4)
#define DEF_TIM_AF__PB9__TCH_TIM4_CH4     D(2, 4)

#define DEF_TIM_AF__PB14__TCH_TIM12_CH1   D(2, 12)
#define DEF_TIM_AF__PB15__TCH_TIM12_CH2   D(2, 12)

//PORTC
#define DEF_TIM_AF__PC6__TCH_TIM3_CH1     D(2, 3)
#define DEF_TIM_AF__PC7__TCH_TIM3_CH2     D(2, 3)
#define DEF_TIM_AF__PC8__TCH_TIM3_CH3     D(2, 3)
#define DEF_TIM_AF__PC9__TCH_TIM3_CH4     D(2, 3)

#define DEF_TIM_AF__PC6__TCH_TIM8_CH1     D(3, 8)
#define DEF_TIM_AF__PC7__TCH_TIM8_CH2     D(3, 8)
#define DEF_TIM_AF__PC8__TCH_TIM8_CH3     D(3, 8)
#define DEF_TIM_AF__PC9__TCH_TIM8_CH4     D(3, 8)

//PORTD
#define DEF_TIM_AF__PD12__TCH_TIM4_CH1    D(2, 4)
#define DEF_TIM_AF__PD13__TCH_TIM4_CH2    D(2, 4)
#define DEF_TIM_AF__PD14__TCH_TIM4_CH3    D(2, 4)
#define DEF_TIM_AF__PD15__TCH_TIM4_CH4    D(2, 4)

//PORTE
#define DEF_TIM_AF__PE8__TCH_TIM1_CH1N    D(1, 1)
#define DEF_TIM_AF__PE9__TCH_TIM1_CH1     D(1, 1)
#define DEF_TIM_AF__PE10__TCH_TIM1_CH2N   D(1, 1)
#define DEF_TIM_AF__PE11__TCH_TIM1_CH2    D(1, 1)
#define DEF_TIM_AF__PE12__TCH_TIM1_CH3N   D(1, 1)
#define DEF_TIM_AF__PE13__TCH_TIM1_CH3    D(1, 1)
#define DEF_TIM_AF__PE14__TCH_TIM1_CH4    D(1, 1)

#define DEF_TIM_AF__PE4__TCH_TIM15_CH1N   D(4, 15)
#define DEF_TIM_AF__PE5__TCH_TIM15_CH1    D(4, 15)
#define DEF_TIM_AF__PE6__TCH_TIM15_CH2    D(4, 15)

//PORTF
#define DEF_TIM_AF__PF6__TCH_TIM16_CH1    D(1, 16)
#define DEF_TIM_AF__PF7__TCH_TIM17_CH1    D(1, 17)
#define DEF_TIM_AF__PF8__TCH_TIM16_CH1N   D(1, 16)
#define DEF_TIM_AF__PF9__TCH_TIM17_CH1N   D(1, 17)

#define DEF_TIM_AF__PF8__TCH_TIM13_CH1N   D(9, 13)
#define DEF_TIM_AF__PF9__TCH_TIM14_CH1N   D(9, 14)

//PORTH
#define DEF_TIM_AF__PH6__TCH_TIM12_CH1    D(2, 12)
#define DEF_TIM_AF__PH9__TCH_TIM12_CH2    D(2, 12)
#define DEF_TIM_AF__PH10__TCH_TIM5_CH1    D(2, 5)
#define DEF_TIM_AF__PH11__TCH_TIM5_CH2    D(2, 5)
#define DEF_TIM_AF__PH12__TCH_TIM5_CH3    D(2, 5)
#define DEF_TIM_AF__PH13__TCH_TIM8_CH1N   D(3, 8)
#define DEF_TIM_AF__PH14__TCH_TIM8_CH2N   D(3, 8)
#define DEF_TIM_AF__PH15__TCH_TIM8_CH3N   D(3, 8)

//PORTI
#define DEF_TIM_AF__PI0__TCH_TIM5_CH4     D(2, 5)

#define DEF_TIM_AF__PI2__TCH_TIM8_CH4     D(3, 8)
#define DEF_TIM_AF__PI5__TCH_TIM8_CH1     D(3, 8)
#define DEF_TIM_AF__PI6__TCH_TIM8_CH2     D(3, 8)
#define DEF_TIM_AF__PI7__TCH_TIM8_CH3     D(3, 8)

#elif defined(STM32G4)

// Missing from FW1.0.0 library?
#define GPIO_AF12_TIM1         ((uint8_t)0x0B)  /* TIM1 Alternate Function mapping    */

#define DEF_TIM(tim, chan, pin, flags, out, dmaopt, upopt) {            \
    tim,                                                                \
    TIMER_GET_IO_TAG(pin),                                              \
    DEF_TIM_CHANNEL(CH_ ## chan),                                       \
    flags,                                                              \
    (DEF_TIM_OUTPUT(CH_ ## chan) | out),                                \
    DEF_TIM_AF(TCH_## tim ## _ ## chan, pin)                            \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_CHANNEL(dmaopt, TCH_## tim ## _ ## chan),            \
        DEF_TIM_DMA_REQUEST(TCH_## tim ## _ ## chan)                    \
    )                                                                   \
    DEF_TIM_DMA_COND(/* add comma */ ,                                  \
        DEF_TIM_DMA_CHANNEL(upopt, TCH_## tim ## _UP),                   \
        DEF_TIM_DMA_REQUEST(TCH_## tim ## _UP),                         \
        DEF_TIM_DMA_HANDLER(upopt, TCH_## tim ## _UP)                   \
    )                                                                   \
}                                                                       \
/**/

#define DEF_TIM_CHANNEL(ch)                   CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) TIM_CHANNEL_ ## chan_n

#define DEF_TIM_AF(timch, pin)                CONCAT(DEF_TIM_AF__, DEF_TIM_AF_GET(timch, pin))
#define DEF_TIM_AF__D(af_n, tim_n)            GPIO_AF ## af_n ## _TIM ## tim_n

#define DEF_TIM_DMA_CHANNEL(variant, timch)                              \
    CONCAT(DEF_TIM_DMA_CHANNEL__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_CHANNEL__D(dma_n, channel_n)  (dmaResource_t *)DMA ## dma_n ## _Channel ## channel_n
#define DEF_TIM_DMA_CHANNEL__NONE                        NULL

// XXX This is awful. There must be some smart way of doing this ...
#define DEF_TIM_DMA_REQUEST(timch) \
    CONCAT(DEF_TIM_DMA_REQ__, DEF_TIM_TCH2BTCH(timch))

#define DEF_TIM_DMA_HANDLER(variant, timch) \
    CONCAT(DEF_TIM_DMA_HANDLER__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_HANDLER__D(dma_n, channel_n) DMA ## dma_n ## _CH ## channel_n ## _HANDLER
#define DEF_TIM_DMA_HANDLER__NONE                       0

/* G4 Channel Mappings */
// D(DMAx, Stream)

// G4 has DMAMUX that allow arbitrary assignment of peripherals to streams.

#define DEF_TIM_DMA_FULL \
    D(1, 1), D(1, 2), D(1, 3), D(1, 4), D(1, 5), D(1, 6), D(1, 7), D(1, 8), \
    D(2, 1), D(2, 2), D(2, 3), D(2, 4), D(2, 5), D(2, 6), D(2, 7), D(2, 8)

#define DEF_TIM_DMA__BTCH_TIM1_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM1_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM1_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM1_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM2_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_CH4    DEF_TIM_DMA_FULL

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

#define DEF_TIM_DMA__BTCH_TIM15_CH1   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM15_CH2   NONE

#define DEF_TIM_DMA__BTCH_TIM16_CH1   DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM17_CH1   DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TIM20_CH1   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM20_CH2   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM20_CH3   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM20_CH4   DEF_TIM_DMA_FULL

// TIM_UP table
#define DEF_TIM_DMA__BTCH_TIM1_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM2_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM3_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM4_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM5_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM6_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM7_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM8_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM15_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM16_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM17_UP    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TIM20_UP    DEF_TIM_DMA_FULL

// TIMx_CHy request table

// This is not defined in stm32g7xx_hal_timer.h
#define DMA_REQUEST_NONE 255

#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH1    DMA_REQUEST_TIM1_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH2    DMA_REQUEST_TIM1_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH3    DMA_REQUEST_TIM1_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM1_CH4    DMA_REQUEST_TIM1_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH1    DMA_REQUEST_TIM2_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH2    DMA_REQUEST_TIM2_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH3    DMA_REQUEST_TIM2_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM2_CH4    DMA_REQUEST_TIM2_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH1    DMA_REQUEST_TIM3_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH2    DMA_REQUEST_TIM3_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH3    DMA_REQUEST_TIM3_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM3_CH4    DMA_REQUEST_TIM3_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH1    DMA_REQUEST_TIM4_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH2    DMA_REQUEST_TIM4_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH3    DMA_REQUEST_TIM4_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM4_CH4    DMA_REQUEST_TIM4_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH1    DMA_REQUEST_TIM5_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH2    DMA_REQUEST_TIM5_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH3    DMA_REQUEST_TIM5_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM5_CH4    DMA_REQUEST_TIM5_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH1    DMA_REQUEST_TIM8_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH2    DMA_REQUEST_TIM8_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH3    DMA_REQUEST_TIM8_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM8_CH4    DMA_REQUEST_TIM8_CH4

#define DEF_TIM_DMA_REQ__BTCH_TIM15_CH1   DMA_REQUEST_TIM15_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM15_CH2   DMA_REQUEST_NONE

#define DEF_TIM_DMA_REQ__BTCH_TIM16_CH1   DMA_REQUEST_TIM16_CH1

#define DEF_TIM_DMA_REQ__BTCH_TIM17_CH1   DMA_REQUEST_TIM17_CH1

#define DEF_TIM_DMA_REQ__BTCH_TIM20_CH1   DMA_REQUEST_TIM20_CH1
#define DEF_TIM_DMA_REQ__BTCH_TIM20_CH2   DMA_REQUEST_TIM20_CH2
#define DEF_TIM_DMA_REQ__BTCH_TIM20_CH3   DMA_REQUEST_TIM20_CH3
#define DEF_TIM_DMA_REQ__BTCH_TIM20_CH4   DMA_REQUEST_TIM20_CH4

// TIM_UP request table
#define DEF_TIM_DMA_REQ__BTCH_TIM1_UP     DMA_REQUEST_TIM1_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM2_UP     DMA_REQUEST_TIM2_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM3_UP     DMA_REQUEST_TIM3_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM4_UP     DMA_REQUEST_TIM4_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM5_UP     DMA_REQUEST_TIM5_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM6_UP     DMA_REQUEST_TIM6_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM7_UP     DMA_REQUEST_TIM7_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM8_UP     DMA_REQUEST_TIM8_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM15_UP    DMA_REQUEST_TIM15_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM16_UP    DMA_REQUEST_TIM16_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM17_UP    DMA_REQUEST_TIM17_UP
#define DEF_TIM_DMA_REQ__BTCH_TIM20_UP    DMA_REQUEST_TIM20_UP

// AF table

//NONE
#define DEF_TIM_AF__NONE__TCH_TIM1_CH1     D(6, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH2     D(6, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH3     D(6, 1)
#define DEF_TIM_AF__NONE__TCH_TIM1_CH4     D(6, 1)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH1     D(3, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH2     D(3, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH3     D(3, 8)
#define DEF_TIM_AF__NONE__TCH_TIM8_CH4     D(3, 8)

//PORTA
#define DEF_TIM_AF__PA0__TCH_TIM2_CH1     D(1, 2)
#define DEF_TIM_AF__PA1__TCH_TIM2_CH2     D(1, 2)
#define DEF_TIM_AF__PA2__TCH_TIM2_CH3     D(1, 2)
#define DEF_TIM_AF__PA3__TCH_TIM2_CH4     D(1, 2)
#define DEF_TIM_AF__PA5__TCH_TIM2_CH1     D(1, 2)
#define DEF_TIM_AF__PA6__TCH_TIM16_CH1    D(1, 16)
#define DEF_TIM_AF__PA7__TCH_TIM17_CH1    D(1, 17)
#define DEF_TIM_AF__PA12__TCH_TIM16_CH1   D(1, 16)
#define DEF_TIM_AF__PA13__TCH_TIM16_CH1N  D(1, 16)
#define DEF_TIM_AF__PA15__TCH_TIM2_CH1    D(1, 2)

#define DEF_TIM_AF__PA0__TCH_TIM5_CH1     D(2, 5)
#define DEF_TIM_AF__PA1__TCH_TIM5_CH2     D(2, 5)
#define DEF_TIM_AF__PA2__TCH_TIM5_CH3     D(2, 5)
#define DEF_TIM_AF__PA3__TCH_TIM5_CH4     D(2, 5)
#define DEF_TIM_AF__PA4__TCH_TIM3_CH2     D(2, 3)
#define DEF_TIM_AF__PA6__TCH_TIM3_CH1     D(2, 3)
#define DEF_TIM_AF__PA7__TCH_TIM3_CH2     D(2, 3)
#define DEF_TIM_AF__PA15__TCH_TIM8_CH1     D(2, 8)

#define DEF_TIM_AF__PA7__TCH_TIM8_CH1N    D(4, 8)

#define DEF_TIM_AF__PA7__TCH_TIM1_CH1N    D(6, 1)
#define DEF_TIM_AF__PA8__TCH_TIM1_CH1     D(6, 1)
#define DEF_TIM_AF__PA9__TCH_TIM1_CH2     D(6, 1)
#define DEF_TIM_AF__PA10__TCH_TIM1_CH3    D(6, 1)
#define DEF_TIM_AF__PA11__TCH_TIM1_CH1N   D(6, 1)
#define DEF_TIM_AF__PA12__TCH_TIM1_CH2N   D(6, 1)

#define DEF_TIM_AF__PA1__TCH_TIM15_CH1N   D(9, 15)
#define DEF_TIM_AF__PA2__TCH_TIM15_CH1    D(9, 15)
#define DEF_TIM_AF__PA3__TCH_TIM15_CH2    D(9, 15)

#define DEF_TIM_AF__PA9__TCH_TIM2_CH3     D(10, 2)
#define DEF_TIM_AF__PA10__TCH_TIM2_CH4    D(10, 2)
#define DEF_TIM_AF__PA11__TCH_TIM4_CH1    D(10, 4)
#define DEF_TIM_AF__PA12__TCH_TIM4_CH2    D(10, 4)
#define DEF_TIM_AF__PA13__TCH_TIM4_CH3    D(10, 4)

#define DEF_TIM_AF__PA11__TCH_TIM1_CH4    D(11, 1)

//PORTB
#define DEF_TIM_AF__PB3__TCH_TIM2_CH2     D(1, 2)
#define DEF_TIM_AF__PB4__TCH_TIM16_CH1    D(1, 16)
#define DEF_TIM_AF__PB6__TCH_TIM16_CH1N   D(1, 16)
#define DEF_TIM_AF__PB7__TCH_TIM17_CH1N   D(1, 17)
#define DEF_TIM_AF__PB8__TCH_TIM16_CH1    D(1, 16)
#define DEF_TIM_AF__PB9__TCH_TIM17_CH1    D(1, 17)
#define DEF_TIM_AF__PB10__TCH_TIM2_CH3    D(1, 2)
#define DEF_TIM_AF__PB11__TCH_TIM2_CH4    D(1, 2)
#define DEF_TIM_AF__PB14__TCH_TIM15_CH1   D(1, 15)
#define DEF_TIM_AF__PB15__TCH_TIM15_CH2   D(1, 15)

#define DEF_TIM_AF__PB0__TCH_TIM3_CH3     D(2, 3)
#define DEF_TIM_AF__PB1__TCH_TIM3_CH4     D(2, 3)
#define DEF_TIM_AF__PB2__TCH_TIM5_CH1     D(2, 5)
#define DEF_TIM_AF__PB4__TCH_TIM3_CH1     D(2, 3)
#define DEF_TIM_AF__PB5__TCH_TIM3_CH2     D(2, 3)
#define DEF_TIM_AF__PB6__TCH_TIM4_CH1     D(2, 4)
#define DEF_TIM_AF__PB7__TCH_TIM4_CH2     D(2, 4)
#define DEF_TIM_AF__PB8__TCH_TIM4_CH3     D(2, 4)
#define DEF_TIM_AF__PB9__TCH_TIM4_CH4     D(2, 4)
#define DEF_TIM_AF__PB15__TCH_TIM15_CH1N  D(2, 15)

#define DEF_TIM_AF__PB2__TCH_TIM20_CH1    D(3, 20)
#define DEF_TIM_AF__PB5__TCH_TIM8_CH3N    D(3, 8)

#define DEF_TIM_AF__PB0__TCH_TIM8_CH2N    D(4, 8)
#define DEF_TIM_AF__PB1__TCH_TIM8_CH3N    D(4, 8)
#define DEF_TIM_AF__PB3__TCH_TIM8_CH1N    D(4, 8)
#define DEF_TIM_AF__PB4__TCH_TIM8_CH2N    D(4, 8)
#define DEF_TIM_AF__PB15__TCH_TIM1_CH3N   D(4, 1)

#define DEF_TIM_AF__PB0__TCH_TIM1_CH2N    D(6, 1)
#define DEF_TIM_AF__PB0__TCH_TIM1_CH3N    D(6, 1)
#define DEF_TIM_AF__PB13__TCH_TIM1_CH1N   D(6, 1)
#define DEF_TIM_AF__PB14__TCH_TIM1_CH2N   D(6, 1)

#define DEF_TIM_AF__PB5__TCH_TIM17_CH1    D(10, 17)
#define DEF_TIM_AF__PB7__TCH_TIM3_CH4     D(10, 3)
#define DEF_TIM_AF__PB8__TCH_TIM8_CH2     D(10, 8)
#define DEF_TIM_AF__PB9__TCH_TIM8_CH3     D(10, 8)

#define DEF_TIM_AF__PB9__TCH_TIM1_CH3N    D(12, 1)

//PORTC
#define DEF_TIM_AF__PC12__TCH_TIM5_CH2    D(1, 5)

#define DEF_TIM_AF__PC0__TCH_TIM1_CH1     D(2, 1)
#define DEF_TIM_AF__PC1__TCH_TIM1_CH2     D(2, 1)
#define DEF_TIM_AF__PC2__TCH_TIM1_CH3     D(2, 1)
#define DEF_TIM_AF__PC3__TCH_TIM1_CH4     D(2, 1)
#define DEF_TIM_AF__PC6__TCH_TIM3_CH1     D(2, 3)
#define DEF_TIM_AF__PC7__TCH_TIM3_CH2     D(2, 3)
#define DEF_TIM_AF__PC8__TCH_TIM3_CH3     D(2, 3)
#define DEF_TIM_AF__PC9__TCH_TIM3_CH4     D(2, 3)

#define DEF_TIM_AF__PC6__TCH_TIM8_CH1     D(4, 8)
#define DEF_TIM_AF__PC7__TCH_TIM8_CH2     D(4, 8)
#define DEF_TIM_AF__PC8__TCH_TIM8_CH3     D(4, 8)
#define DEF_TIM_AF__PC9__TCH_TIM8_CH4     D(4, 8)
#define DEF_TIM_AF__PC10__TCH_TIM8_CH1N   D(4, 8)
#define DEF_TIM_AF__PC11__TCH_TIM8_CH2N   D(4, 8)
#define DEF_TIM_AF__PC12__TCH_TIM8_CH3N   D(4, 8)
#define DEF_TIM_AF__PC13__TCH_TIM1_CH1N   D(4, 1)

#define DEF_TIM_AF__PC2__TCH_TIM20_CH2    D(6, 20)
#define DEF_TIM_AF__PC5__TCH_TIM1_CH4N    D(6, 1)
#define DEF_TIM_AF__PC8__TCH_TIM20_CH3    D(6, 20)
#define DEF_TIM_AF__PC13__TCH_TIM8_CH4N   D(6, 8)

#endif

