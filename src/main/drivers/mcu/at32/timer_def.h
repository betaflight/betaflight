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

#define USED_TIMERS  ( BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(8) | BIT(9) | BIT(10) | BIT(11) | BIT(12) | BIT(13) | BIT(14) | BIT(20) )
#define TIMUP_TIMERS ( BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(8) | BIT(20) )
#define FULL_TIMER_CHANNEL_COUNT        109
#define HARDWARE_TIMER_DEFINITION_COUNT 15

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
#define BTCH_TMR1_CH1N BTCH_TMR1_CH1
#define BTCH_TMR1_CH2N BTCH_TMR1_CH2
#define BTCH_TMR1_CH3N BTCH_TMR1_CH3

#define BTCH_TMR8_CH1N BTCH_TMR8_CH1
#define BTCH_TMR8_CH2N BTCH_TMR8_CH2
#define BTCH_TMR8_CH3N BTCH_TMR8_CH3

#define BTCH_TMR11_CH1N BTCH_TMR11_CH1

#define BTCH_TMR20_CH1N BTCH_TMR20_CH1
#define BTCH_TMR20_CH2N BTCH_TMR20_CH2
#define BTCH_TMR20_CH3N BTCH_TMR20_CH3

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
#define DEF_TIM_TIM__TIM_TMR1  D(1)
#define DEF_TIM_TIM__TIM_TMR2  D(2)
#define DEF_TIM_TIM__TIM_TMR3  D(3)
#define DEF_TIM_TIM__TIM_TMR4  D(4)
#define DEF_TIM_TIM__TIM_TMR5  D(5)
#define DEF_TIM_TIM__TIM_TMR6  D(6)
#define DEF_TIM_TIM__TIM_TMR7  D(7)
#define DEF_TIM_TIM__TIM_TMR8  D(8)
#define DEF_TIM_TIM__TIM_TMR9  D(9)
#define DEF_TIM_TIM__TIM_TMR10 D(10)
#define DEF_TIM_TIM__TIM_TMR11 D(11)
#define DEF_TIM_TIM__TIM_TMR12 D(12)
#define DEF_TIM_TIM__TIM_TMR13 D(13)
#define DEF_TIM_TIM__TIM_TMR14 D(14)
#define DEF_TIM_TIM__TIM_TMR15 D(15)
#define DEF_TIM_TIM__TIM_TMR16 D(16)
#define DEF_TIM_TIM__TIM_TMR17 D(17)
#define DEF_TIM_TIM__TIM_TMR18 D(18)
#define DEF_TIM_TIM__TIM_TMR19 D(19)
#define DEF_TIM_TIM__TIM_TMR20 D(20)
#define DEF_TIM_TIM__TIM_TMR21 D(21)
#define DEF_TIM_TIM__TIM_TMR22 D(22)

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
#define DEF_TIM_AF__D(af_n, tim_n)            GPIO_MUX_ ## af_n  /*GPIO_MUX_1 gpio_mux_sel_type */

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
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) chan_n


#define DEF_TIM_DMA_CHANNEL(variant, timch)                              \
    CONCAT(DEF_TIM_DMA_CHANNEL__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_CHANNEL__D(dma_n, channel_n)  (dmaResource_t *)DMA ## dma_n ## _CHANNEL ## channel_n
#define DEF_TIM_DMA_CHANNEL__NONE                        NULL

#define DEF_TIM_DMA_REQUEST(timch) \
    CONCAT(DEF_TIM_DMA_REQ__, DEF_TIM_TCH2BTCH(timch))

#define DEF_TIM_DMA_HANDLER(variant, timch) \
    CONCAT(DEF_TIM_DMA_HANDLER__, DEF_TIM_DMA_GET(variant, timch))
#define DEF_TIM_DMA_HANDLER__D(dma_n, channel_n) DMA ## dma_n ## _CH ## channel_n ## _HANDLER
#define DEF_TIM_DMA_HANDLER__NONE                       0


/* DMA Channel Mappings */
// D(DMAx, Stream)
// at32f43x has DMAMUX that allow arbitrary assignment of peripherals to streams.
#define DEF_TIM_DMA_FULL \
    D(1, 1), D(1, 2), D(1, 3), D(1, 4), D(1, 5), D(1, 6), D(1, 7), \
    D(2, 1), D(2, 2), D(2, 3), D(2, 4), D(2, 5), D(2, 6), D(2, 7)

#define DEF_TIM_DMA__BTCH_TMR1_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR1_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR1_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR1_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TMR2_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR2_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR2_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR2_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TMR3_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR3_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR3_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR3_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TMR4_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR4_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR4_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR4_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TMR5_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR5_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR5_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR5_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TMR8_CH1    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR8_CH2    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR8_CH3    DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR8_CH4    DEF_TIM_DMA_FULL

#define DEF_TIM_DMA__BTCH_TMR9_CH1    NONE
#define DEF_TIM_DMA__BTCH_TMR9_CH2    NONE

#define DEF_TIM_DMA__BTCH_TMR10_CH1   NONE

#define DEF_TIM_DMA__BTCH_TMR11_CH1   NONE
#define DEF_TIM_DMA__BTCH_TMR12_CH1   NONE
#define DEF_TIM_DMA__BTCH_TMR12_CH2   NONE

#define DEF_TIM_DMA__BTCH_TMR13_CH1   NONE
#define DEF_TIM_DMA__BTCH_TMR14_CH1   NONE

#define DEF_TIM_DMA__BTCH_TMR15_CH1   NONE
#define DEF_TIM_DMA__BTCH_TMR15_CH2   NONE

#define DEF_TIM_DMA__BTCH_TMR16_CH1   NONE
#define DEF_TIM_DMA__BTCH_TMR17_CH1   NONE

#define DEF_TIM_DMA__BTCH_TMR20_CH1   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR20_CH2   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR20_CH3   DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR20_CH4   DEF_TIM_DMA_FULL

// TIM_UP table
#define DEF_TIM_DMA__BTCH_TMR1_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR2_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR3_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR4_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR5_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR6_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR7_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR8_UP     DEF_TIM_DMA_FULL
#define DEF_TIM_DMA__BTCH_TMR9_UP     NONE
#define DEF_TIM_DMA__BTCH_TMR10_UP    NONE
#define DEF_TIM_DMA__BTCH_TMR11_UP    NONE
#define DEF_TIM_DMA__BTCH_TMR12_UP    NONE
#define DEF_TIM_DMA__BTCH_TMR13_UP    NONE
#define DEF_TIM_DMA__BTCH_TMR14_UP    NONE
#define DEF_TIM_DMA__BTCH_TMR15_UP    NONE
#define DEF_TIM_DMA__BTCH_TMR16_UP    NONE
#define DEF_TIM_DMA__BTCH_TMR17_UP    NONE
#define DEF_TIM_DMA__BTCH_TMR20_UP    DEF_TIM_DMA_FULL

// TIMx_CHy request table

#define DMA_REQUEST_NONE 255

#define DEF_TIM_DMA_REQ__BTCH_TMR1_CH1    DMAMUX_DMAREQ_ID_TMR1_CH1
#define DEF_TIM_DMA_REQ__BTCH_TMR1_CH2    DMAMUX_DMAREQ_ID_TMR1_CH2
#define DEF_TIM_DMA_REQ__BTCH_TMR1_CH3    DMAMUX_DMAREQ_ID_TMR1_CH3
#define DEF_TIM_DMA_REQ__BTCH_TMR1_CH4    DMAMUX_DMAREQ_ID_TMR1_CH4

#define DEF_TIM_DMA_REQ__BTCH_TMR2_CH1    DMAMUX_DMAREQ_ID_TMR2_CH1
#define DEF_TIM_DMA_REQ__BTCH_TMR2_CH2    DMAMUX_DMAREQ_ID_TMR2_CH2
#define DEF_TIM_DMA_REQ__BTCH_TMR2_CH3    DMAMUX_DMAREQ_ID_TMR2_CH3
#define DEF_TIM_DMA_REQ__BTCH_TMR2_CH4    DMAMUX_DMAREQ_ID_TMR2_CH4

#define DEF_TIM_DMA_REQ__BTCH_TMR3_CH1    DMAMUX_DMAREQ_ID_TMR3_CH1
#define DEF_TIM_DMA_REQ__BTCH_TMR3_CH2    DMAMUX_DMAREQ_ID_TMR3_CH2
#define DEF_TIM_DMA_REQ__BTCH_TMR3_CH3    DMAMUX_DMAREQ_ID_TMR3_CH3
#define DEF_TIM_DMA_REQ__BTCH_TMR3_CH4    DMAMUX_DMAREQ_ID_TMR3_CH4

#define DEF_TIM_DMA_REQ__BTCH_TMR4_CH1    DMAMUX_DMAREQ_ID_TMR4_CH1
#define DEF_TIM_DMA_REQ__BTCH_TMR4_CH2    DMAMUX_DMAREQ_ID_TMR4_CH2
#define DEF_TIM_DMA_REQ__BTCH_TMR4_CH3    DMAMUX_DMAREQ_ID_TMR4_CH3
#define DEF_TIM_DMA_REQ__BTCH_TMR4_CH4    DMAMUX_DMAREQ_ID_TMR4_CH4

#define DEF_TIM_DMA_REQ__BTCH_TMR5_CH1    DMAMUX_DMAREQ_ID_TMR5_CH1
#define DEF_TIM_DMA_REQ__BTCH_TMR5_CH2    DMAMUX_DMAREQ_ID_TMR5_CH2
#define DEF_TIM_DMA_REQ__BTCH_TMR5_CH3    DMAMUX_DMAREQ_ID_TMR5_CH3
#define DEF_TIM_DMA_REQ__BTCH_TMR5_CH4    DMAMUX_DMAREQ_ID_TMR5_CH4

#define DEF_TIM_DMA_REQ__BTCH_TMR8_CH1    DMAMUX_DMAREQ_ID_TMR8_CH1
#define DEF_TIM_DMA_REQ__BTCH_TMR8_CH2    DMAMUX_DMAREQ_ID_TMR8_CH2
#define DEF_TIM_DMA_REQ__BTCH_TMR8_CH3    DMAMUX_DMAREQ_ID_TMR8_CH3
#define DEF_TIM_DMA_REQ__BTCH_TMR8_CH4    DMAMUX_DMAREQ_ID_TMR8_CH4

#define DEF_TIM_DMA_REQ__BTCH_TMR9_CH1    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR9_CH2    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR10_CH1   DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR11_CH1   DMA_REQUEST_NONE

#define DEF_TIM_DMA_REQ__BTCH_TMR12_CH1   DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR12_CH2   DMA_REQUEST_NONE

#define DEF_TIM_DMA_REQ__BTCH_TMR13_CH1   DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR14_CH1   DMA_REQUEST_NONE

#define DEF_TIM_DMA_REQ__BTCH_TMR20_CH1   DMAMUX_DMAREQ_ID_TMR20_CH1
#define DEF_TIM_DMA_REQ__BTCH_TMR20_CH2   DMAMUX_DMAREQ_ID_TMR20_CH2
#define DEF_TIM_DMA_REQ__BTCH_TMR20_CH3   DMAMUX_DMAREQ_ID_TMR20_CH3
#define DEF_TIM_DMA_REQ__BTCH_TMR20_CH4   DMAMUX_DMAREQ_ID_TMR20_CH4

// TIM_UP request table
#define DEF_TIM_DMA_REQ__BTCH_TMR1_UP     DMAMUX_DMAREQ_ID_TMR1_OVERFLOW
#define DEF_TIM_DMA_REQ__BTCH_TMR2_UP     DMAMUX_DMAREQ_ID_TMR2_OVERFLOW
#define DEF_TIM_DMA_REQ__BTCH_TMR3_UP     DMAMUX_DMAREQ_ID_TMR3_OVERFLOW
#define DEF_TIM_DMA_REQ__BTCH_TMR4_UP     DMAMUX_DMAREQ_ID_TMR4_OVERFLOW
#define DEF_TIM_DMA_REQ__BTCH_TMR5_UP     DMAMUX_DMAREQ_ID_TMR5_OVERFLOW
#define DEF_TIM_DMA_REQ__BTCH_TMR8_UP     DMAMUX_DMAREQ_ID_TMR8_OVERFLOW
#define DEF_TIM_DMA_REQ__BTCH_TMR9_UP     DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR10_UP    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR11_UP    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR12_UP    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR13_UP    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR14_UP    DMA_REQUEST_NONE
#define DEF_TIM_DMA_REQ__BTCH_TMR20_UP    DMAMUX_DMAREQ_ID_TMR20_OVERFLOW

// AF table for timer ,default is GPIO_MUX_1  should be check after debug

//NONE d(mux_id, timerid)
#define DEF_TIM_AF__NONE__TCH_TMR1_CH1     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TMR1_CH2     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TMR1_CH3     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TMR1_CH4     D(1, 1)
#define DEF_TIM_AF__NONE__TCH_TMR8_CH1     D(1, 8)
#define DEF_TIM_AF__NONE__TCH_TMR8_CH2     D(1, 8)
#define DEF_TIM_AF__NONE__TCH_TMR8_CH3     D(1, 8)
#define DEF_TIM_AF__NONE__TCH_TMR8_CH4     D(1, 8)

//PORTA MUX 1
#define DEF_TIM_AF__PA0__TCH_TMR2_CH1      D(1, 2)
#define DEF_TIM_AF__PA1__TCH_TMR2_CH2      D(1, 2)
#define DEF_TIM_AF__PA2__TCH_TMR2_CH3      D(1, 2)
#define DEF_TIM_AF__PA3__TCH_TMR2_CH4      D(1, 2)
#define DEF_TIM_AF__PA5__TCH_TMR2_CH1      D(1, 2)
#define DEF_TIM_AF__PA7__TCH_TMR1_CH1N     D(1, 1)
#define DEF_TIM_AF__PA8__TCH_TMR1_CH1      D(1, 1)
#define DEF_TIM_AF__PA9__TCH_TMR1_CH2      D(1, 1)
#define DEF_TIM_AF__PA10__TCH_TMR1_CH3     D(1, 1)
#define DEF_TIM_AF__PA11__TCH_TMR1_CH4     D(1, 1)
#define DEF_TIM_AF__PA15__TCH_TMR2_CH1     D(1, 2)

//PORTA MUX 2
#define DEF_TIM_AF__PA0__TCH_TMR5_CH1      D(2, 5)
#define DEF_TIM_AF__PA1__TCH_TMR5_CH2      D(2, 5)
#define DEF_TIM_AF__PA2__TCH_TMR5_CH3      D(2, 5)
#define DEF_TIM_AF__PA3__TCH_TMR5_CH4      D(2, 5)
#define DEF_TIM_AF__PA6__TCH_TMR3_CH1      D(2, 3)
#define DEF_TIM_AF__PA7__TCH_TMR3_CH2      D(2, 3)

// PORTA MUX 3
#define DEF_TIM_AF__PA0__TCH_TMR8_EXT      D(1, 8)
#define DEF_TIM_AF__PA2__TCH_TMR9_CH1      D(3, 9)
#define DEF_TIM_AF__PA3__TCH_TMR9_CH2      D(3, 9)
#define DEF_TIM_AF__PA5__TCH_TMR8_CH1N     D(3, 8)
#define DEF_TIM_AF__PA7__TCH_TMR8_CH1N     D(3, 8)

// PORTA MUX 9
#define DEF_TIM_AF__PA6__TCH_TMR13_CH1     D(9, 13)
#define DEF_TIM_AF__PA7__TCH_TMR14_CH1     D(9, 14)

// PORTB MUX 1
#define DEF_TIM_AF__PB0__TCH_TMR1_CH2N     D(1, 1)
#define DEF_TIM_AF__PB1__TCH_TMR1_CH3N     D(1, 1)
#define DEF_TIM_AF__PB2__TCH_TMR2_CH4      D(1, 2)
#define DEF_TIM_AF__PB3__TCH_TMR2_CH2      D(1, 2)
#define DEF_TIM_AF__PB8__TCH_TMR2_CH1      D(1, 2)
#define DEF_TIM_AF__PB9__TCH_TMR2_CH2      D(1, 2)
#define DEF_TIM_AF__PB10__TCH_TMR2_CH3     D(1, 2)
#define DEF_TIM_AF__PB11__TCH_TMR2_CH4     D(1, 2)
#define DEF_TIM_AF__PB13__TCH_TMR1_CH1N    D(1, 1)
#define DEF_TIM_AF__PB14__TCH_TMR1_CH2N    D(1, 1)
#define DEF_TIM_AF__PB15__TCH_TMR1_CH3N    D(1, 1)

// PORTB MUX 2
#define DEF_TIM_AF__PB0__TCH_TMR3_CH3      D(2, 3)
#define DEF_TIM_AF__PB1__TCH_TMR3_CH4      D(2, 3)
#define DEF_TIM_AF__PB2__TCH_TMR20_CH1     D(2, 20)
#define DEF_TIM_AF__PB4__TCH_TMR3_CH1      D(2, 3)
#define DEF_TIM_AF__PB5__TCH_TMR3_CH2      D(2, 3)
#define DEF_TIM_AF__PB6__TCH_TMR4_CH1      D(2, 4)
#define DEF_TIM_AF__PB7__TCH_TMR4_CH2      D(2, 4)
#define DEF_TIM_AF__PB8__TCH_TMR4_CH3      D(2, 4)
#define DEF_TIM_AF__PB9__TCH_TMR4_CH4      D(2, 4)
#define DEF_TIM_AF__PB11__TCH_TMR5_CH4     D(2, 5)
#define DEF_TIM_AF__PB12__TCH_TMR5_CH1     D(2, 5)

// PORTB MUX 3
#define DEF_TIM_AF__PB0__TCH_TMR8_CH2N     D(3, 8)
#define DEF_TIM_AF__PB1__TCH_TMR8_CH3N     D(3, 8)
#define DEF_TIM_AF__PB8__TCH_TMR10_CH1     D(3, 10)
#define DEF_TIM_AF__PB9__TCH_TMR11_CH1     D(3, 11)
#define DEF_TIM_AF__PB14__TCH_TMR8_CH2N    D(3, 8)
#define DEF_TIM_AF__PB15__TCH_TMR8_CH3N    D(3, 8)

// PORTB MUX 9
#define DEF_TIM_AF__PB14__TCH_TMR12_CH1    D(9, 12)
#define DEF_TIM_AF__PB15__TCH_TMR12_CH2    D(9, 12)

// PORTC MUX 2
#define DEF_TIM_AF__PC2__TCH_TMR20_CH2     D(2, 20)
#define DEF_TIM_AF__PC6__TCH_TMR3_CH1      D(2, 3)
#define DEF_TIM_AF__PC7__TCH_TMR3_CH2      D(2, 3)
#define DEF_TIM_AF__PC8__TCH_TMR3_CH3      D(2, 3)
#define DEF_TIM_AF__PC9__TCH_TMR3_CH4      D(2, 3)
#define DEF_TIM_AF__PC10__TCH_TMR5_CH2     D(2, 5)
#define DEF_TIM_AF__PC11__TCH_TMR5_CH3     D(2, 5)

// PORTC MUX 3
#define DEF_TIM_AF__PC4__TCH_TMR9_CH1      D(3, 9)
#define DEF_TIM_AF__PC5__TCH_TMR9_CH2      D(3, 9)
#define DEF_TIM_AF__PC6__TCH_TMR8_CH1      D(3, 8)
#define DEF_TIM_AF__PC7__TCH_TMR8_CH2      D(3, 8)
#define DEF_TIM_AF__PC8__TCH_TMR8_CH3      D(3, 8)
#define DEF_TIM_AF__PC9__TCH_TMR8_CH4      D(3, 8)
#define DEF_TIM_AF__PC12__TCH_TMR11_CH1N   D(3, 11)

// PORTD MUX 2
#define DEF_TIM_AF__PD12__TCH_TMR4_CH1     D(2, 4)
#define DEF_TIM_AF__PD13__TCH_TMR4_CH2     D(2, 4)
#define DEF_TIM_AF__PD14__TCH_TMR4_CH3     D(2, 4)
#define DEF_TIM_AF__PD15__TCH_TMR4_CH4     D(2, 4)

// PORTE MUX 1
#define DEF_TIM_AF__PE1__TCH_TMR1_CH2N     D(1, 1)
#define DEF_TIM_AF__PE7__TCH_TMR1_EXT      D(1, 1)
#define DEF_TIM_AF__PE8__TCH_TMR1_CH1N     D(1, 1)
#define DEF_TIM_AF__PE9__TCH_TMR1_CH1      D(1, 1)
#define DEF_TIM_AF__PE10__TCH_TMR1_CH2N    D(1, 1)
#define DEF_TIM_AF__PE11__TCH_TMR1_CH2     D(1, 1)
#define DEF_TIM_AF__PE12__TCH_TMR1_CH3N    D(1, 1)
#define DEF_TIM_AF__PE13__TCH_TMR1_CH3     D(1, 1)
#define DEF_TIM_AF__PE14__TCH_TMR1_CH4     D(1, 1)
#define DEF_TIM_AF__PE15__TCH_TMR1_BRK     D(1, 1)

// PORTE MUX 2
#define DEF_TIM_AF__PE0__TCH_TMR4_EXT      D(2, 4)
#define DEF_TIM_AF__PE2__TCH_TMR3_EXT      D(2, 3)
#define DEF_TIM_AF__PE3__TCH_TMR3_CH1      D(2, 3)
#define DEF_TIM_AF__PE4__TCH_TMR3_CH2      D(2, 3)
#define DEF_TIM_AF__PE5__TCH_TMR3_CH3      D(2, 3)
#define DEF_TIM_AF__PE6__TCH_TMR3_CH4      D(2, 3)

// PORTE MUX 3
#define DEF_TIM_AF__PE5__TCH_TMR9_CH1      D(2, 9)
#define DEF_TIM_AF__PE6__TCH_TMR9_CH2      D(2, 9)

// PORTE MUX 6
#define DEF_TIM_AF__PE0__TCH_TMR20_EXT     D(6, 20)
#define DEF_TIM_AF__PE1__TCH_TMR20_CH4     D(6, 20)
#define DEF_TIM_AF__PE2__TCH_TMR20_CH1     D(6, 20)
#define DEF_TIM_AF__PE3__TCH_TMR20_CH2     D(6, 20)
#define DEF_TIM_AF__PE4__TCH_TMR20_CH1N    D(6, 20)
#define DEF_TIM_AF__PE5__TCH_TMR20_CH2N    D(6, 20)
#define DEF_TIM_AF__PE6__TCH_TMR20_CH3N    D(6, 20)

// PORTF MUX 1
#define DEF_TIM_AF__PF10__TCH_TMR1_EXT     D(2, 1)

// PORTF MUX 2
#define DEF_TIM_AF__PF2__TCH_TMR20_CH3     D(2, 20)
#define DEF_TIM_AF__PF3__TCH_TMR20_CH4     D(2, 20)
#define DEF_TIM_AF__PF4__TCH_TMR20_CH1N    D(2, 20)
#define DEF_TIM_AF__PF5__TCH_TMR20_CH2N    D(2, 20)
#define DEF_TIM_AF__PF6__TCH_TMR20_CH4     D(2, 20)
#define DEF_TIM_AF__PF7__TCH_TMR20_BRK     D(2, 20)
#define DEF_TIM_AF__PF9__TCH_TMR20_BRK     D(2, 20)
#define DEF_TIM_AF__PF11__TCH_TMR20_EST    D(2, 20)
#define DEF_TIM_AF__PF12__TCH_TMR20_CH1    D(2, 20)
#define DEF_TIM_AF__PF13__TCH_TMR20_CH2    D(2, 20)
#define DEF_TIM_AF__PF14__TCH_TMR20_CH3    D(2, 20)
#define DEF_TIM_AF__PF15__TCH_TMR20_CH4    D(2, 20)

// PORTF MUX 3
#define DEF_TIM_AF__PF6__TCH_TMR10_CH1     D(3, 10)
#define DEF_TIM_AF__PF7__TCH_TMR11_CH1     D(3, 11)
#define DEF_TIM_AF__PF11__TCH_TMR8_EST     D(3, 8)
#define DEF_TIM_AF__PF12__TCH_TMR8_BRK     D(3, 8)

// PORTF MUX 9
#define DEF_TIM_AF__PF8__TCH_TMR13_CH1     D(9, 13)
#define DEF_TIM_AF__PF9__TCH_TMR14_CH1     D(9, 14)

// PORTG MUX 2
#define DEF_TIM_AF__PG0__TCH_TMR20_CH1N    D(2, 20)
#define DEF_TIM_AF__PG1__TCH_TMR20_CH2N    D(2, 20)
#define DEF_TIM_AF__PG2__TCH_TMR20_CH3N    D(2, 20)
#define DEF_TIM_AF__PG3__TCH_TMR20_BRK     D(2, 20)
#define DEF_TIM_AF__PG5__TCH_TMR20_EXT     D(2, 20)

// PORTH MUX 2
#define DEF_TIM_AF__PH2__TCH_TMR5_CH1      D(2, 5)
#define DEF_TIM_AF__PH3__TCH_TMR5_CH2      D(2, 5)
