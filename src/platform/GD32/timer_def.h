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
#define BTCH_TIMER0_CH0N BTCH_TIMER0_CH0
#define BTCH_TIMER0_CH1N BTCH_TIMER0_CH1
#define BTCH_TIMER0_CH2N BTCH_TIMER0_CH2

#define BTCH_TIMER7_CH0N BTCH_TIMER7_CH0
#define BTCH_TIMER7_CH1N BTCH_TIMER7_CH1
#define BTCH_TIMER7_CH2N BTCH_TIMER7_CH2

#define BTCH_TIMER12_CH0N BTCH_TIMER12_CH0
#define BTCH_TIMER13_CH0N BTCH_TIMER13_CH0

// channel table D(chan_n, n_type)
#define DEF_TIM_CH_GET(ch) CONCAT2(DEF_TIM_CH__, ch)
#define DEF_TIM_CH__CH_CH0   D(0, 0)
#define DEF_TIM_CH__CH_CH1   D(1, 0)
#define DEF_TIM_CH__CH_CH2   D(2, 0)
#define DEF_TIM_CH__CH_CH3   D(3, 0)
#define DEF_TIM_CH__CH_CH0N  D(0, 1)
#define DEF_TIM_CH__CH_CH1N  D(1, 1)
#define DEF_TIM_CH__CH_CH2N  D(2, 1)

// timer table D(tim_n)
#define DEF_TIM_TIM_GET(tim) CONCAT2(DEF_TIM_TIM__, tim)
#define DEF_TIM_TIM__TIMER_TIMER0  D(0)
#define DEF_TIM_TIM__TIMER_TIMER1  D(1)
#define DEF_TIM_TIM__TIMER_TIMER2  D(2)
#define DEF_TIM_TIM__TIMER_TIMER3  D(3)
#define DEF_TIM_TIM__TIMER_TIMER4  D(4)
#define DEF_TIM_TIM__TIMER_TIMER5  D(5)
#define DEF_TIM_TIM__TIMER_TIMER6  D(6)
#define DEF_TIM_TIM__TIMER_TIMER7  D(7)
#define DEF_TIM_TIM__TIMER_TIMER8  D(8)
#define DEF_TIM_TIM__TIMER_TIMER9  D(9)
#define DEF_TIM_TIM__TIMER_TIMER10 D(10)
#define DEF_TIM_TIM__TIMER_TIMER11 D(11)
#define DEF_TIM_TIM__TIMER_TIMER12 D(12)
#define DEF_TIM_TIM__TIMER_TIMER13 D(13)

#define DEF_TIM_TIM__TIM_TIM15 D(15)
#define DEF_TIM_TIM__TIM_TIM16 D(16)
#define DEF_TIM_TIM__TIM_TIM17 D(17)
#define DEF_TIM_TIM__TIM_TIM18 D(18)
#define DEF_TIM_TIM__TIM_TIM19 D(19)
#define DEF_TIM_TIM__TIM_TIM20 D(20)
#define DEF_TIM_TIM__TIM_TIM21 D(21)
#define DEF_TIM_TIM__TIM_TIM22 D(22)

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

#if defined(GD32F4)

#define DEF_TIMER(tim, chan, pin, out, dmaopt) {                  \
    (void *)tim,                                                \
    TIMER_GET_IO_TAG(pin),                                      \
    DEF_TIM_CHANNEL(CH_ ## chan),                               \
    (DEF_TIM_OUTPUT(CH_ ## chan) | out),                        \
    DEF_TIM_AF(TCH_ ## tim ## _ ## chan, pin)                   \
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

#define DEF_TIM_CHANNEL(ch)                   CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) TIMER_CH_ ## chan_n

#define DEF_TIM_AF(timch, pin)                CONCAT(DEF_TIM_AF__, DEF_TIM_AF_GET(timch, pin))
#define DEF_TIM_AF__D(af_n, tim_n)            GPIO_AF_ ## af_n

#define DEF_TIM_DMA_CHANNEL(variant, timch) \
    CONCAT(DEF_TIM_DMA_CHANNEL__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_CHANNEL__D(dma_n, stream_n, chan_n)    DMA_SUBPERI ## chan_n
#define DEF_TIM_DMA_CHANNEL__NONE                          DMA_SUBPERI0

#define DEF_TIM_DMA_STREAM(variant, timch)                              \
    CONCAT(DEF_TIM_DMA_STREAM__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_STREAM__D(dma_n, stream_n, chan_n)  (dmaResource_t *)DMA ## dma_n ## _CH ## stream_n ## _BASE
#define DEF_TIM_DMA_STREAM__NONE                        NULL

#define DEF_TIM_DMA_HANDLER(variant, timch) \
    CONCAT(DEF_TIM_DMA_HANDLER__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_HANDLER__D(dma_n, stream_n, chan_n) DMA ## dma_n ## _CH ## stream_n ## _HANDLER
#define DEF_TIM_DMA_HANDLER__NONE                       0

/* F4 Channel Mappings */
// D(DMAx, Channel, Sub_Channel)
#define DEF_TIM_DMA__BTCH_TIMER0_CH0    D(1, 6, 0),D(1, 1, 6),D(1, 3, 6)
#define DEF_TIM_DMA__BTCH_TIMER0_CH1    D(1, 6, 0),D(1, 2, 6)
#define DEF_TIM_DMA__BTCH_TIMER0_CH2    D(1, 6, 0),D(1, 6, 6)
#define DEF_TIM_DMA__BTCH_TIMER0_CH3    D(1, 4, 6)
#define DEF_TIM_DMA__BTCH_TIMER0_CH0N  NONE

#define DEF_TIM_DMA__BTCH_TIMER1_CH0    D(0, 5, 3)
#define DEF_TIM_DMA__BTCH_TIMER1_CH1    D(0, 6, 3)
#define DEF_TIM_DMA__BTCH_TIMER1_CH2    D(0, 1, 3)
#define DEF_TIM_DMA__BTCH_TIMER1_CH3    D(0, 7, 3),D(0, 6, 3)

#define DEF_TIM_DMA__BTCH_TIMER2_CH0    D(0, 4, 5)
#define DEF_TIM_DMA__BTCH_TIMER2_CH1    D(0, 5, 5)
#define DEF_TIM_DMA__BTCH_TIMER2_CH2    D(0, 7, 5)
#define DEF_TIM_DMA__BTCH_TIMER2_CH3    D(0, 2, 5)

#define DEF_TIM_DMA__BTCH_TIMER3_CH0    D(0, 0, 2)
#define DEF_TIM_DMA__BTCH_TIMER3_CH1    D(0, 3, 2)
#define DEF_TIM_DMA__BTCH_TIMER3_CH2    D(0, 7, 2)

#define DEF_TIM_DMA__BTCH_TIMER4_CH0    D(0, 2, 6)
#define DEF_TIM_DMA__BTCH_TIMER4_CH1    D(0, 4, 6)
#define DEF_TIM_DMA__BTCH_TIMER4_CH2    D(0, 0, 6)
#define DEF_TIM_DMA__BTCH_TIMER4_CH3    D(0, 1, 6),D(0, 3, 6)

#define DEF_TIM_DMA__BTCH_TIMER7_CH0    D(1, 2, 0),D(1, 2, 7)
#define DEF_TIM_DMA__BTCH_TIMER7_CH1    D(1, 2, 0),D(1, 3, 7)
#define DEF_TIM_DMA__BTCH_TIMER7_CH2    D(1, 2, 0),D(1, 4, 7)
#define DEF_TIM_DMA__BTCH_TIMER7_CH3    D(1, 7, 7)

#define DEF_TIM_DMA__BTCH_TIMER3_CH3    NONE

#define DEF_TIM_DMA__BTCH_TIMER8_CH0    NONE
#define DEF_TIM_DMA__BTCH_TIMER8_CH1    NONE

#define DEF_TIM_DMA__BTCH_TIMER9_CH0    NONE

#define DEF_TIM_DMA__BTCH_TIMER10_CH0   NONE

#define DEF_TIM_DMA__BTCH_TIMER11_CH0   NONE
#define DEF_TIM_DMA__BTCH_TIMER11_CH1   NONE

#define DEF_TIM_DMA__BTCH_TIMER12_CH0   NONE

#define DEF_TIM_DMA__BTCH_TIMER13_CH0   NONE

// TIM_UP table
#define DEF_TIM_DMA__BTCH_TIMER0_UP     D(1, 5, 6)
#define DEF_TIM_DMA__BTCH_TIMER1_UP     D(0, 7, 3)
#define DEF_TIM_DMA__BTCH_TIMER2_UP     D(0, 2, 5)
#define DEF_TIM_DMA__BTCH_TIMER3_UP     D(0, 6, 2)
#define DEF_TIM_DMA__BTCH_TIMER4_UP     D(0, 0, 6)
#define DEF_TIM_DMA__BTCH_TIMER5_UP     D(0, 1, 7)
#define DEF_TIM_DMA__BTCH_TIMER6_UP     D(0, 4, 1)
#define DEF_TIM_DMA__BTCH_TIMER7_UP     D(1, 1, 7)
#define DEF_TIM_DMA__BTCH_TIMER8_UP     NONE
#define DEF_TIM_DMA__BTCH_TIMER9_UP     NONE
#define DEF_TIM_DMA__BTCH_TIMER10_UP    NONE
#define DEF_TIM_DMA__BTCH_TIMER11_UP    NONE
#define DEF_TIM_DMA__BTCH_TIMER12_UP    NONE
#define DEF_TIM_DMA__BTCH_TIMER13_UP    NONE

// AF table
// D(afn, timern)
// NONE
#define DEF_TIM_AF__NONE__TCH_TIMER0_CH0     D(1, 0)
#define DEF_TIM_AF__NONE__TCH_TIMER0_CH1     D(1, 0)
#define DEF_TIM_AF__NONE__TCH_TIMER0_CH2     D(1, 0)
#define DEF_TIM_AF__NONE__TCH_TIMER0_CH3     D(1, 0)
#define DEF_TIM_AF__NONE__TCH_TIMER7_CH0     D(3, 7)
#define DEF_TIM_AF__NONE__TCH_TIMER7_CH1     D(3, 7)
#define DEF_TIM_AF__NONE__TCH_TIMER7_CH2     D(3, 7)
#define DEF_TIM_AF__NONE__TCH_TIMER7_CH3     D(3, 7)

//PORTA
#define DEF_TIM_AF__PA0__TCH_TIMER1_CH0     D(1, 1)
#define DEF_TIM_AF__PA1__TCH_TIMER1_CH1     D(1, 1)
#define DEF_TIM_AF__PA2__TCH_TIMER1_CH2     D(1, 1)
#define DEF_TIM_AF__PA3__TCH_TIMER1_CH3     D(1, 1)
#define DEF_TIM_AF__PA5__TCH_TIMER1_CH0     D(1, 1)
#define DEF_TIM_AF__PA7__TCH_TIMER0_CH0N    D(1, 0)
#define DEF_TIM_AF__PA8__TCH_TIMER0_CH0     D(1, 0)
#define DEF_TIM_AF__PA9__TCH_TIMER0_CH1     D(1, 0)
#define DEF_TIM_AF__PA10__TCH_TIMER0_CH2    D(1, 0)
#define DEF_TIM_AF__PA11__TCH_TIMER0_CH3    D(1, 0)
#define DEF_TIM_AF__PA15__TCH_TIMER1_CH0    D(1, 1)

#define DEF_TIM_AF__PA0__TCH_TIMER4_CH0     D(2, 4)
#define DEF_TIM_AF__PA1__TCH_TIMER4_CH1     D(2, 4)
#define DEF_TIM_AF__PA2__TCH_TIMER4_CH2     D(2, 4)
#define DEF_TIM_AF__PA3__TCH_TIMER4_CH3     D(2, 4)
#define DEF_TIM_AF__PA6__TCH_TIMER2_CH0     D(2, 2)
#define DEF_TIM_AF__PA7__TCH_TIMER2_CH1     D(2, 2)

#define DEF_TIM_AF__PA2__TCH_TIMER8_CH0     D(3, 8)
#define DEF_TIM_AF__PA3__TCH_TIMER8_CH1     D(3, 8)
#define DEF_TIM_AF__PA5__TCH_TIMER7_CH0N   D(3, 7)
#define DEF_TIM_AF__PA7__TCH_TIMER7_CH0N   D(3, 7)

#define DEF_TIM_AF__PA6__TCH_TIMER12_CH0    D(9, 12)
#define DEF_TIM_AF__PA7__TCH_TIMER13_CH0    D(9, 13)

//PORTB
#define DEF_TIM_AF__PB0__TCH_TIMER0_CH1N    D(1, 0)
#define DEF_TIM_AF__PB1__TCH_TIMER0_CH2N    D(1, 0)
#define DEF_TIM_AF__PB2__TCH_TIMER1_CH3     D(1, 1)
#define DEF_TIM_AF__PB3__TCH_TIMER1_CH1     D(1, 1)
#define DEF_TIM_AF__PB8__TCH_TIMER1_CH0     D(1, 1)  
#define DEF_TIM_AF__PB9__TCH_TIMER1_CH1     D(1, 1)
#define DEF_TIM_AF__PB10__TCH_TIMER1_CH2    D(1, 1)
#define DEF_TIM_AF__PB11__TCH_TIMER1_CH3    D(1, 1)
#define DEF_TIM_AF__PB13__TCH_TIMER0_CH0N  D(1, 0)
#define DEF_TIM_AF__PB14__TCH_TIMER0_CH1N  D(1, 0)
#define DEF_TIM_AF__PB15__TCH_TIMER0_CH2N  D(1, 0)

#define DEF_TIM_AF__PB0__TCH_TIMER2_CH2     D(2, 2)
#define DEF_TIM_AF__PB1__TCH_TIMER2_CH3     D(2, 2)
#define DEF_TIM_AF__PB4__TCH_TIMER2_CH0     D(2, 2)
#define DEF_TIM_AF__PB5__TCH_TIMER2_CH1     D(2, 2)
#define DEF_TIM_AF__PB6__TCH_TIMER3_CH0     D(2, 3)
#define DEF_TIM_AF__PB7__TCH_TIMER3_CH1     D(2, 3)
#define DEF_TIM_AF__PB8__TCH_TIMER3_CH2     D(2, 3)
#define DEF_TIM_AF__PB9__TCH_TIMER3_CH3     D(2, 3)

#define DEF_TIM_AF__PB0__TCH_TIMER7_CH1N    D(3, 7)
#define DEF_TIM_AF__PB1__TCH_TIMER7_CH2N    D(3, 7)
#define DEF_TIM_AF__PB8__TCH_TIMER9_CH0     D(3, 9)
#define DEF_TIM_AF__PB9__TCH_TIMER10_CH0    D(3, 10)
#define DEF_TIM_AF__PB14__TCH_TIMER7_CH1N   D(3, 7)
#define DEF_TIM_AF__PB15__TCH_TIMER7_CH2N   D(3, 7)

#define DEF_TIM_AF__PB14__TCH_TIMER11_CH0   D(9, 11)
#define DEF_TIM_AF__PB15__TCH_TIMER11_CH1   D(9, 11)

//PORTC
#define DEF_TIM_AF__PC6__TCH_TIMER2_CH0     D(2, 2)
#define DEF_TIM_AF__PC7__TCH_TIMER2_CH1     D(2, 2)
#define DEF_TIM_AF__PC8__TCH_TIMER2_CH2     D(2, 2)
#define DEF_TIM_AF__PC9__TCH_TIMER2_CH3     D(2, 2)

#define DEF_TIM_AF__PC6__TCH_TIMER7_CH0     D(3, 7)
#define DEF_TIM_AF__PC7__TCH_TIMER7_CH1     D(3, 7)
#define DEF_TIM_AF__PC8__TCH_TIMER7_CH2     D(3, 7)
#define DEF_TIM_AF__PC9__TCH_TIMER7_CH3     D(3, 7)

//PORTD
#define DEF_TIM_AF__PD12__TCH_TIMER3_CH0    D(2, 3)
#define DEF_TIM_AF__PD13__TCH_TIMER3_CH1    D(2, 3)
#define DEF_TIM_AF__PD14__TCH_TIMER3_CH2    D(2, 3)
#define DEF_TIM_AF__PD15__TCH_TIMER3_CH3    D(2, 3)

//PORTE
#define DEF_TIM_AF__PE8__TCH_TIMER0_CH0N    D(1, 0)
#define DEF_TIM_AF__PE9__TCH_TIMER0_CH0     D(1, 0)
#define DEF_TIM_AF__PE10__TCH_TIMER0_CH1N   D(1, 0)
#define DEF_TIM_AF__PE11__TCH_TIMER0_CH1    D(1, 0)
#define DEF_TIM_AF__PE12__TCH_TIMER0_CH2N   D(1, 0)
#define DEF_TIM_AF__PE13__TCH_TIMER0_CH2    D(1, 0)
#define DEF_TIM_AF__PE14__TCH_TIMER0_CH3    D(1, 0)

#define DEF_TIM_AF__PE5__TCH_TIMER8_CH0     D(3, 8)
#define DEF_TIM_AF__PE6__TCH_TIMER8_CH1     D(3, 8)

//PORTF
#define DEF_TIM_AF__PF6__TCH_TIMER9_CH0     D(3, 9)
#define DEF_TIM_AF__PF7__TCH_TIMER10_CH0    D(3, 10)

//PORTH
#define DEF_TIM_AF__PH10__TCH_TIMER4_CH0    D(2, 4)
#define DEF_TIM_AF__PH11__TCH_TIMER4_CH1    D(2, 4)
#define DEF_TIM_AF__PH12__TCH_TIMER4_CH2    D(2, 4)

#define DEF_TIM_AF__PH13__TCH_TIMER7_CH0N   D(3, 7)
#define DEF_TIM_AF__PH14__TCH_TIMER7_CH1N   D(3, 7)
#define DEF_TIM_AF__PH15__TCH_TIMER7_CH2N   D(3, 7)

#define DEF_TIM_AF__PH6__TCH_TIMER11_CH0    D(9, 11)
#define DEF_TIM_AF__PH9__TCH_TIMER11_CH1    D(9, 11)

//PORTI
#define DEF_TIM_AF__PI0__TCH_TIMER4_CH3     D(2, 4)

#define DEF_TIM_AF__PI2__TCH_TIMER7_CH3     D(3, 7)
#define DEF_TIM_AF__PI5__TCH_TIMER7_CH0     D(3, 7)
#define DEF_TIM_AF__PI6__TCH_TIMER7_CH1     D(3, 7)
#define DEF_TIM_AF__PI7__TCH_TIMER7_CH2     D(3, 7)

#else

#endif

#if defined(GD32F4)

#define FULL_TIMER_CHANNEL_COUNT 78
#define USED_TIMERS ( TIM_N(0) | TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(6) | TIM_N(7) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(11) | TIM_N(12) | TIM_N(13))
#define HARDWARE_TIMER_DEFINITION_COUNT 14

#else

#endif
