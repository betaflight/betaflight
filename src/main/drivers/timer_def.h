/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <platform.h>
#include "common/utils.h"

#if defined(STM32F1)

#define DEF_TIM(tim, chan, pin, flags, out) {\
    tim,\
    IO_TAG(pin),\
    EXPAND(DEF_CHAN_ ## chan),\
    flags,\
    (DEF_CHAN_ ## chan ## _OUTPUT | out),\
    CONCAT(EXPAND(DEF_TIM_DMA__ ## tim ## _ ## chan), _CHANNEL),\
    CONCAT(EXPAND(DEF_TIM_DMA__ ## tim ## _ ## chan), _HANDLER)\
    }

#define DEF_DMA_CHANNEL(tim, chan) CONCAT(EXPAND(DEF_TIM_DMA__ ## tim ## _ ## chan), _CHANNEL)
#define DEF_DMA_HANDLER(tim, chan) CONCAT(EXPAND(DEF_TIM_DMA__ ## tim ## _ ## chan), _HANDLER)

/* add the DMA mappings here for F1 */
#define DEF_TIM_DMA__TIM1_CH1    DMA1_CH2
#define DEF_TIM_DMA__TIM1_CH2    DMA_NONE
#define DEF_TIM_DMA__TIM1_CH3    DMA1_CH6
#define DEF_TIM_DMA__TIM1_CH4    DMA1_CH4

#define DEF_TIM_DMA__TIM2_CH1    DMA1_CH5
#define DEF_TIM_DMA__TIM2_CH2    DMA1_CH7
#define DEF_TIM_DMA__TIM2_CH3    DMA1_CH1
#define DEF_TIM_DMA__TIM2_CH4    DMA1_CH7

#define DEF_TIM_DMA__TIM3_CH1    DMA1_CH6
#define DEF_TIM_DMA__TIM3_CH2    DMA_NONE
#define DEF_TIM_DMA__TIM3_CH3    DMA1_CH2
#define DEF_TIM_DMA__TIM3_CH4    DMA1_CH3

#define DEF_TIM_DMA__TIM4_CH1    DMA1_CH1
#define DEF_TIM_DMA__TIM4_CH2    DMA1_CH4
#define DEF_TIM_DMA__TIM4_CH3    DMA1_CH5
#define DEF_TIM_DMA__TIM4_CH4    DMA_NONE

#elif defined(STM32F3)

#define DEF_TIM(tim, chan, pin, flags, out) {\
    tim,\
    IO_TAG(pin),\
    EXPAND(DEF_CHAN_ ## chan),\
    flags,\
    (DEF_CHAN_ ## chan ## _OUTPUT | out),\
    EXPAND(GPIO_AF__ ## pin ## _ ## tim ## _ ## chan),\
    CONCAT(EXPAND(DEF_TIM_DMA__ ## tim ## _ ## chan), _CHANNEL),\
    CONCAT(EXPAND(DEF_TIM_DMA__ ## tim ## _ ## chan), _HANDLER)\
    }

#define DEF_DMA_CHANNEL(tim, chan) CONCAT(EXPAND(DEF_TIM_DMA__ ## tim ## _ ## chan), _CHANNEL)
#define DEF_DMA_HANDLER(tim, chan) CONCAT(EXPAND(DEF_TIM_DMA__ ## tim ## _ ## chan), _HANDLER)

/* add the DMA mappings here */
#define DEF_TIM_DMA__TIM1_CH1    DMA1_CH2
#define DEF_TIM_DMA__TIM1_CH2    DMA1_CH3
#define DEF_TIM_DMA__TIM1_CH4    DMA1_CH4
#define DEF_TIM_DMA__TIM1_CH1N   DMA1_CH2
#define DEF_TIM_DMA__TIM1_CH2N   DMA1_CH3
#define DEF_TIM_DMA__TIM1_TRIG   DMA1_CH4
#define DEF_TIM_DMA__TIM1_COM    DMA1_CH4
#define DEF_TIM_DMA__TIM1_UP     DMA1_CH5
#define DEF_TIM_DMA__TIM1_CH3    DMA1_CH6
#define DEF_TIM_DMA__TIM1_CH3N   DMA1_CH6

#define DEF_TIM_DMA__TIM2_CH3    DMA1_CH1
#define DEF_TIM_DMA__TIM2_UP     DMA1_CH2
#define DEF_TIM_DMA__TIM2_CH1    DMA1_CH5
#define DEF_TIM_DMA__TIM2_CH2    DMA1_CH7
#define DEF_TIM_DMA__TIM2_CH4    DMA1_CH7

#define DEF_TIM_DMA__TIM3_CH2    DMA_NONE
#define DEF_TIM_DMA__TIM3_CH3    DMA1_CH2
#define DEF_TIM_DMA__TIM3_CH4    DMA1_CH3
#define DEF_TIM_DMA__TIM3_UP     DMA1_CH3
#define DEF_TIM_DMA__TIM3_CH1    DMA1_CH6
#define DEF_TIM_DMA__TIM3_TRIG   DMA1_CH6

#define DEF_TIM_DMA__TIM4_CH1    DMA1_CH1
#define DEF_TIM_DMA__TIM4_CH2    DMA1_CH4
#define DEF_TIM_DMA__TIM4_CH3    DMA1_CH5
#define DEF_TIM_DMA__TIM4_UP     DMA1_CH7
#define DEF_TIM_DMA__TIM4_CH4    DMA_NONE

#define DEF_TIM_DMA__TIM15_CH1   DMA1_CH5
#define DEF_TIM_DMA__TIM15_CH2   DMA_NONE
#define DEF_TIM_DMA__TIM15_UP    DMA1_CH5
#define DEF_TIM_DMA__TIM15_TRIG  DMA1_CH5
#define DEF_TIM_DMA__TIM15_COM   DMA1_CH5
#define DEF_TIM_DMA__TIM15_CH1N  DMA1_CH5

#ifdef REMAP_TIM16_DMA
#define DEF_TIM_DMA__TIM16_CH1   DMA1_CH6
#define DEF_TIM_DMA__TIM16_CH1N  DMA1_CH6
#define DEF_TIM_DMA__TIM16_UP    DMA1_CH6
#else
#define DEF_TIM_DMA__TIM16_CH1   DMA1_CH3
#define DEF_TIM_DMA__TIM16_CH1N  DMA1_CH3
#define DEF_TIM_DMA__TIM16_UP    DMA1_CH3
#endif

#ifdef REMAP_TIM17_DMA
#define DEF_TIM_DMA__TIM17_CH1   DMA1_CH7
#define DEF_TIM_DMA__TIM17_CH1N  DMA1_CH7
#define DEF_TIM_DMA__TIM17_UP    DMA1_CH7
#else
#define DEF_TIM_DMA__TIM17_CH1   DMA1_CH1
#define DEF_TIM_DMA__TIM17_CH1N  DMA1_CH1
#define DEF_TIM_DMA__TIM17_UP    DMA1_CH1
#endif

#define DEF_TIM_DMA__TIM8_CH3    DMA2_CH1
#define DEF_TIM_DMA__TIM8_CH3N   DMA2_CH1
#define DEF_TIM_DMA__TIM8_UP     DMA2_CH1
#define DEF_TIM_DMA__TIM8_CH4    DMA2_CH2
#define DEF_TIM_DMA__TIM8_TRIG   DMA2_CH2
#define DEF_TIM_DMA__TIM8_COM    DMA2_CH2
#define DEF_TIM_DMA__TIM8_CH1    DMA2_CH3
#define DEF_TIM_DMA__TIM8_CH1N   DMA2_CH3
#define DEF_TIM_DMA__TIM8_CH2    DMA2_CH5
#define DEF_TIM_DMA__TIM8_CH2N   DMA2_CH5


#define GPIO_AF(p, t) CONCAT(GPIO_AF__, p, _, t)

#define GPIO_AF__PA0_TIM2_CH1     GPIO_AF_1
#define GPIO_AF__PA1_TIM2_CH2     GPIO_AF_1
#define GPIO_AF__PA2_TIM2_CH3     GPIO_AF_1
#define GPIO_AF__PA3_TIM2_CH4     GPIO_AF_1
#define GPIO_AF__PA5_TIM2_CH1     GPIO_AF_1
#define GPIO_AF__PA6_TIM16_CH1    GPIO_AF_1
#define GPIO_AF__PA7_TIM17_CH1    GPIO_AF_1
#define GPIO_AF__PA12_TIM16_CH1   GPIO_AF_1
#define GPIO_AF__PA13_TIM16_CH1N  GPIO_AF_1
#define GPIO_AF__PA15_TIM2_CH1    GPIO_AF_1

#define GPIO_AF__PA4_TIM3_CH2     GPIO_AF_2
#define GPIO_AF__PA6_TIM3_CH1     GPIO_AF_2
#define GPIO_AF__PA7_TIM3_CH2     GPIO_AF_2
#define GPIO_AF__PA15_TIM8_CH1    GPIO_AF_2

#define GPIO_AF__PA7_TIM8_CH1N    GPIO_AF_4

#define GPIO_AF__PA14_TIM4_CH2    GPIO_AF_5

#define GPIO_AF__PA7_TIM1_CH1N    GPIO_AF_6
#define GPIO_AF__PA8_TIM1_CH1     GPIO_AF_6
#define GPIO_AF__PA9_TIM1_CH2     GPIO_AF_6
#define GPIO_AF__PA10_TIM1_CH3    GPIO_AF_6
#define GPIO_AF__PA11_TIM1_CH1N   GPIO_AF_6
#define GPIO_AF__PA12_TIM1_CH2N   GPIO_AF_6

#define GPIO_AF__PA1_TIM15_CH1N   GPIO_AF_9
#define GPIO_AF__PA2_TIM15_CH1    GPIO_AF_9
#define GPIO_AF__PA3_TIM15_CH2    GPIO_AF_9

#define GPIO_AF__PA9_TIM2_CH3     GPIO_AF_10
#define GPIO_AF__PA10_TIM2_CH4    GPIO_AF_10
#define GPIO_AF__PA11_TIM4_CH1    GPIO_AF_10
#define GPIO_AF__PA12_TIM4_CH2    GPIO_AF_10
#define GPIO_AF__PA13_TIM4_CH3    GPIO_AF_10
#define GPIO_AF__PA11_TIM1_CH4    GPIO_AF_11

#define GPIO_AF__PB3_TIM2_CH2     GPIO_AF_1
#define GPIO_AF__PB4_TIM16_CH1    GPIO_AF_1
#define GPIO_AF__PB6_TIM16_CH1N   GPIO_AF_1
#define GPIO_AF__PB7_TIM17_CH1N   GPIO_AF_1
#define GPIO_AF__PB8_TIM16_CH1    GPIO_AF_1
#define GPIO_AF__PB9_TIM17_CH1    GPIO_AF_1
#define GPIO_AF__PB10_TIM2_CH3    GPIO_AF_1
#define GPIO_AF__PB11_TIM2_CH4    GPIO_AF_1
#define GPIO_AF__PB14_TIM15_CH1   GPIO_AF_1
#define GPIO_AF__PB15_TIM15_CH2   GPIO_AF_1

#define GPIO_AF__PB0_TIM3_CH3     GPIO_AF_2
#define GPIO_AF__PB1_TIM3_CH4     GPIO_AF_2
#define GPIO_AF__PB4_TIM3_CH1     GPIO_AF_2
#define GPIO_AF__PB5_TIM3_CH2     GPIO_AF_2
#define GPIO_AF__PB6_TIM4_CH1     GPIO_AF_2
#define GPIO_AF__PB7_TIM4_CH2     GPIO_AF_2
#define GPIO_AF__PB8_TIM4_CH3     GPIO_AF_2
#define GPIO_AF__PB9_TIM4_CH4     GPIO_AF_2
#define GPIO_AF__PB15_TIM15_CH1N  GPIO_AF_2

#define GPIO_AF__PB5_TIM8_CH3N    GPIO_AF_3

#define GPIO_AF__PB0_TIM8_CH2N    GPIO_AF_4
#define GPIO_AF__PB1_TIM8_CH3N    GPIO_AF_4
#define GPIO_AF__PB3_TIM8_CH1N    GPIO_AF_4
#define GPIO_AF__PB4_TIM8_CH2N    GPIO_AF_4
#define GPIO_AF__PB15_TIM1_CH3N   GPIO_AF_4

#define GPIO_AF__PB6_TIM8_CH1     GPIO_AF_5

#define GPIO_AF__PB0_TIM1_CH2N    GPIO_AF_6
#define GPIO_AF__PB1_TIM1_CH3N    GPIO_AF_6
#define GPIO_AF__PB13_TIM1_CH1N   GPIO_AF_6
#define GPIO_AF__PB14_TIM1_CH2N   GPIO_AF_6

#define GPIO_AF__PB5_TIM17_CH1    GPIO_AF_10
#define GPIO_AF__PB7_TIM3_CH4     GPIO_AF_10
#define GPIO_AF__PB8_TIM8_CH2     GPIO_AF_10
#define GPIO_AF__PB9_TIM8_CH3     GPIO_AF_10

#define GPIO_AF__PC6_TIM3_CH1     GPIO_AF_2
#define GPIO_AF__PC7_TIM3_CH2     GPIO_AF_2
#define GPIO_AF__PC8_TIM3_CH3     GPIO_AF_2
#define GPIO_AF__PC9_TIM3_CH4     GPIO_AF_2

#define GPIO_AF__PC6_TIM8_CH1     GPIO_AF_4
#define GPIO_AF__PC7_TIM8_CH2     GPIO_AF_4
#define GPIO_AF__PC8_TIM8_CH3     GPIO_AF_4
#define GPIO_AF__PC9_TIM8_CH4     GPIO_AF_4

#define GPIO_AF__PC10_TIM8_CH1N   GPIO_AF_4
#define GPIO_AF__PC11_TIM8_CH2N   GPIO_AF_4
#define GPIO_AF__PC12_TIM8_CH3N   GPIO_AF_4
#define GPIO_AF__PC13_TIM8_CH1N   GPIO_AF_4

#define GPIO_AF__PD3_TIM2_CH1     GPIO_AF_2
#define GPIO_AF__PD4_TIM2_CH2     GPIO_AF_2
#define GPIO_AF__PD6_TIM2_CH4     GPIO_AF_2
#define GPIO_AF__PD7_TIM2_CH3     GPIO_AF_2

#define GPIO_AF__PD12_TIM4_CH1    GPIO_AF_2
#define GPIO_AF__PD13_TIM4_CH2    GPIO_AF_2
#define GPIO_AF__PD14_TIM4_CH3    GPIO_AF_2
#define GPIO_AF__PD15_TIM4_CH4    GPIO_AF_2

#define GPIO_AF__PD1_TIM8_CH4     GPIO_AF_4

#define GPIO_AF__PF9_TIM15_CH1    GPIO_AF_3
#define GPIO_AF__PF10_TIM15_CH2   GPIO_AF_3

#elif defined(STM32F4)

#define DMA_OPT_FIRST     0
#define DMA_OPT_SECOND    0
#define DMA_OPT_THIRD     0

#define DEF_TIM(tim, chan, pin, flags, out, dmaopt) {\
    tim,\
    IO_TAG(pin),\
    EXPAND(DEF_CHAN_ ## chan),\
    flags,\
    (DEF_CHAN_ ## chan ## _OUTPUT | out),\
    EXPAND(GPIO_AF_## tim),\
    CONCAT(EXPAND(DEF_TIM_DMA_STR_ ## dmaopt ## __ ## tim ## _ ## chan), _STREAM),\
    EXPAND(DEF_TIM_DMA_CHN_ ## dmaopt ## __ ## tim ## _ ## chan),\
    CONCAT(EXPAND(DEF_TIM_DMA_STR_ ## dmaopt ## __ ## tim ## _ ## chan), _HANDLER)\
    }

#define DEF_DMA_CHANNEL(tim, chan, dmaopt) EXPAND(DEF_TIM_DMA_CHN_ ## dmaopt ## __ ## tim ## _ ## chan)
#define DEF_DMA_STREAM(tim, chan, dmaopt) CONCAT(EXPAND(DEF_TIM_DMA_STR_ ## dmaopt ## __ ## tim ## _ ## chan), _STREAM)
#define DEF_DMA_HANDLER(tim, chan, dmaopt) CONCAT(EXPAND(DEF_TIM_DMA_STR_ ## dmaopt ## __ ## tim ## _ ## chan), _HANDLER)

/* F4 Stream Mappings */

#define DEF_TIM_DMA_STR_0__TIM1_CH1    DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH1    DMA2_ST1
#define DEF_TIM_DMA_STR_2__TIM1_CH1    DMA2_ST3
#define DEF_TIM_DMA_STR_0__TIM1_CH1N   DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH1N   DMA2_ST1
#define DEF_TIM_DMA_STR_2__TIM1_CH1N   DMA2_ST3
#define DEF_TIM_DMA_STR_0__TIM1_CH2    DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH2    DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM1_CH2N   DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH2N   DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM1_CH3    DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH3    DMA2_ST6
#define DEF_TIM_DMA_STR_0__TIM1_CH3N   DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH3N   DMA2_ST6
#define DEF_TIM_DMA_STR_0__TIM1_CH4    DMA2_ST4

#define DEF_TIM_DMA_STR_0__TIM2_CH1    DMA1_ST5
#define DEF_TIM_DMA_STR_0__TIM2_CH2    DMA1_ST6
#define DEF_TIM_DMA_STR_0__TIM2_CH3    DMA1_ST1
#define DEF_TIM_DMA_STR_0__TIM2_CH4    DMA1_ST7
#define DEF_TIM_DMA_STR_1__TIM2_CH4    DMA1_ST6

#define DEF_TIM_DMA_STR_0__TIM3_CH1    DMA1_ST4
#define DEF_TIM_DMA_STR_0__TIM3_CH2    DMA1_ST5
#define DEF_TIM_DMA_STR_0__TIM3_CH3    DMA1_ST7
#define DEF_TIM_DMA_STR_0__TIM3_CH4    DMA1_ST2

#define DEF_TIM_DMA_STR_0__TIM4_CH1    DMA1_ST0
#define DEF_TIM_DMA_STR_0__TIM4_CH2    DMA1_ST3
#define DEF_TIM_DMA_STR_0__TIM4_CH3    DMA1_ST7

#define DEF_TIM_DMA_STR_0__TIM5_CH1    DMA1_ST2
#define DEF_TIM_DMA_STR_0__TIM5_CH2    DMA1_ST4
#define DEF_TIM_DMA_STR_0__TIM5_CH3    DMA1_ST0
#define DEF_TIM_DMA_STR_0__TIM5_CH4    DMA1_ST1
#define DEF_TIM_DMA_STR_1__TIM5_CH4    DMA1_ST3

#define DEF_TIM_DMA_STR_0__TIM8_CH1    DMA2_ST2
#define DEF_TIM_DMA_STR_1__TIM8_CH1    DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM8_CH1N   DMA2_ST2
#define DEF_TIM_DMA_STR_1__TIM8_CH1N   DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM8_CH2    DMA2_ST3
#define DEF_TIM_DMA_STR_1__TIM8_CH2    DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM8_CH2N   DMA2_ST3
#define DEF_TIM_DMA_STR_1__TIM8_CH2N   DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM8_CH3    DMA2_ST2
#define DEF_TIM_DMA_STR_1__TIM8_CH3    DMA2_ST4
#define DEF_TIM_DMA_STR_0__TIM8_CH3N   DMA2_ST2
#define DEF_TIM_DMA_STR_1__TIM8_CH3N   DMA2_ST4
#define DEF_TIM_DMA_STR_0__TIM8_CH4    DMA2_ST7

#define DEF_TIM_DMA_STR_0__TIM4_CH4    DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM9_CH1    DMA_NONE
#define DEF_TIM_DMA_STR_0__TIM9_CH2    DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM10_CH1   DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM11_CH1   DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM12_CH1   DMA_NONE
#define DEF_TIM_DMA_STR_0__TIM12_CH2   DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM13_CH1   DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM14_CH1   DMA_NONE

/* F4 Channel Mappings */

#define DEF_TIM_DMA_CHN_0__TIM1_CH1    DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH1    DMA_Channel_6
#define DEF_TIM_DMA_CHN_2__TIM1_CH1    DMA_Channel_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH1N   DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH1N   DMA_Channel_6
#define DEF_TIM_DMA_CHN_2__TIM1_CH1N   DMA_Channel_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH2    DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH2    DMA_Channel_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH2N   DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH2N   DMA_Channel_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH3    DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH3    DMA_Channel_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH3N   DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH3N   DMA_Channel_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH4    DMA_Channel_6

#define DEF_TIM_DMA_CHN_0__TIM2_CH1    DMA_Channel_3
#define DEF_TIM_DMA_CHN_0__TIM2_CH2    DMA_Channel_3
#define DEF_TIM_DMA_CHN_0__TIM2_CH3    DMA_Channel_3
#define DEF_TIM_DMA_CHN_0__TIM2_CH4    DMA_Channel_3
#define DEF_TIM_DMA_CHN_1__TIM2_CH4    DMA_Channel_3

#define DEF_TIM_DMA_CHN_0__TIM3_CH1    DMA_Channel_5
#define DEF_TIM_DMA_CHN_0__TIM3_CH2    DMA_Channel_5
#define DEF_TIM_DMA_CHN_0__TIM3_CH3    DMA_Channel_5
#define DEF_TIM_DMA_CHN_0__TIM3_CH4    DMA_Channel_5

#define DEF_TIM_DMA_CHN_0__TIM4_CH1    DMA_Channel_2
#define DEF_TIM_DMA_CHN_0__TIM4_CH2    DMA_Channel_2
#define DEF_TIM_DMA_CHN_0__TIM4_CH3    DMA_Channel_2

#define DEF_TIM_DMA_CHN_0__TIM5_CH1    DMA_Channel_6
#define DEF_TIM_DMA_CHN_0__TIM5_CH2    DMA_Channel_6
#define DEF_TIM_DMA_CHN_0__TIM5_CH3    DMA_Channel_6
#define DEF_TIM_DMA_CHN_0__TIM5_CH4    DMA_Channel_6
#define DEF_TIM_DMA_CHN_1__TIM5_CH4    DMA_Channel_6

#define DEF_TIM_DMA_CHN_0__TIM8_CH1    DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM8_CH1    DMA_Channel_7
#define DEF_TIM_DMA_CHN_0__TIM8_CH1N   DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM8_CH1N   DMA_Channel_7
#define DEF_TIM_DMA_CHN_0__TIM8_CH2    DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM8_CH2    DMA_Channel_7
#define DEF_TIM_DMA_CHN_0__TIM8_CH2N   DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM8_CH2N   DMA_Channel_7
#define DEF_TIM_DMA_CHN_0__TIM8_CH3    DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM8_CH3    DMA_Channel_7
#define DEF_TIM_DMA_CHN_0__TIM8_CH3N   DMA_Channel_0
#define DEF_TIM_DMA_CHN_1__TIM8_CH3N   DMA_Channel_7
#define DEF_TIM_DMA_CHN_0__TIM8_CH4    DMA_Channel_7

#define DEF_TIM_DMA_CHN_0__TIM4_CH4    0

#define DEF_TIM_DMA_CHN_0__TIM9_CH1    0
#define DEF_TIM_DMA_CHN_0__TIM9_CH2    0

#define DEF_TIM_DMA_CHN_0__TIM10_CH1   0

#define DEF_TIM_DMA_CHN_0__TIM11_CH1   0

#define DEF_TIM_DMA_CHN_0__TIM12_CH1   0
#define DEF_TIM_DMA_CHN_0__TIM12_CH2   0

#define DEF_TIM_DMA_CHN_0__TIM13_CH1   0

#define DEF_TIM_DMA_CHN_0__TIM14_CH1   0

#define DMA1_ST0_STREAM                DMA1_Stream0
#define DMA1_ST1_STREAM                DMA1_Stream1
#define DMA1_ST2_STREAM                DMA1_Stream2
#define DMA1_ST3_STREAM                DMA1_Stream3
#define DMA1_ST4_STREAM                DMA1_Stream4
#define DMA1_ST5_STREAM                DMA1_Stream5
#define DMA1_ST6_STREAM                DMA1_Stream6
#define DMA1_ST7_STREAM                DMA1_Stream7
#define DMA2_ST0_STREAM                DMA2_Stream0
#define DMA2_ST1_STREAM                DMA2_Stream1
#define DMA2_ST2_STREAM                DMA2_Stream2
#define DMA2_ST3_STREAM                DMA2_Stream3
#define DMA2_ST4_STREAM                DMA2_Stream4
#define DMA2_ST5_STREAM                DMA2_Stream5
#define DMA2_ST6_STREAM                DMA2_Stream6
#define DMA2_ST7_STREAM                DMA2_Stream7

#elif defined(STM32F7)
#define DEF_TIM(tim, chan, pin, flags, out, dmaopt) {\
    tim,\
    IO_TAG(pin),\
    EXPAND(DEF_CHAN_ ## chan),\
    flags,\
    (DEF_CHAN_ ## chan ## _OUTPUT | out),\
    EXPAND(GPIO_AF__ ## pin ## _ ## tim ## _ ## chan),\
    CONCAT(EXPAND(DEF_TIM_DMA_STR_ ## dmaopt ## __ ## tim ## _ ## chan), _STREAM),\
    EXPAND(DEF_TIM_DMA_CHN_ ## dmaopt ## __ ## tim ## _ ## chan),\
    CONCAT(EXPAND(DEF_TIM_DMA_STR_ ## dmaopt ## __ ## tim ## _ ## chan), _HANDLER)\
    }

#define DEF_DMA_CHANNEL(tim, chan, dmaopt) EXPAND(DEF_TIM_DMA_CHN_ ## dmaopt ## __ ## tim ## _ ## chan)
#define DEF_DMA_STREAM(tim, chan, dmaopt) CONCAT(EXPAND(DEF_TIM_DMA_STR_ ## dmaopt ## __ ## tim ## _ ## chan), _STREAM)
#define DEF_DMA_HANDLER(tim, chan, dmaopt) CONCAT(EXPAND(DEF_TIM_DMA_STR_ ## dmaopt ## __ ## tim ## _ ## chan), _HANDLER)

/* F7 Stream Mappings */

#define DEF_TIM_DMA_STR_0__TIM1_CH1    DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH1    DMA2_ST1
#define DEF_TIM_DMA_STR_2__TIM1_CH1    DMA2_ST3
#define DEF_TIM_DMA_STR_0__TIM1_CH1N   DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH1N   DMA2_ST1
#define DEF_TIM_DMA_STR_2__TIM1_CH1N   DMA2_ST3
#define DEF_TIM_DMA_STR_0__TIM1_CH2    DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH2    DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM1_CH2N   DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH2N   DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM1_CH3    DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH3    DMA2_ST6
#define DEF_TIM_DMA_STR_0__TIM1_CH3N   DMA2_ST6
#define DEF_TIM_DMA_STR_1__TIM1_CH3N   DMA2_ST6
#define DEF_TIM_DMA_STR_0__TIM1_CH4    DMA2_ST4

#define DEF_TIM_DMA_STR_0__TIM2_CH1    DMA1_ST5
#define DEF_TIM_DMA_STR_0__TIM2_CH2    DMA1_ST6
#define DEF_TIM_DMA_STR_0__TIM2_CH3    DMA1_ST1
#define DEF_TIM_DMA_STR_0__TIM2_CH4    DMA1_ST7
#define DEF_TIM_DMA_STR_1__TIM2_CH4    DMA1_ST6

#define DEF_TIM_DMA_STR_0__TIM3_CH1    DMA1_ST4
#define DEF_TIM_DMA_STR_0__TIM3_CH2    DMA1_ST5
#define DEF_TIM_DMA_STR_0__TIM3_CH3    DMA1_ST7
#define DEF_TIM_DMA_STR_0__TIM3_CH4    DMA1_ST2

#define DEF_TIM_DMA_STR_0__TIM4_CH1    DMA1_ST0
#define DEF_TIM_DMA_STR_0__TIM4_CH2    DMA1_ST3
#define DEF_TIM_DMA_STR_0__TIM4_CH3    DMA1_ST7

#define DEF_TIM_DMA_STR_0__TIM5_CH1    DMA1_ST2
#define DEF_TIM_DMA_STR_0__TIM5_CH2    DMA1_ST4
#define DEF_TIM_DMA_STR_0__TIM5_CH3    DMA1_ST0
#define DEF_TIM_DMA_STR_0__TIM5_CH4    DMA1_ST1
#define DEF_TIM_DMA_STR_1__TIM5_CH4    DMA1_ST3

#define DEF_TIM_DMA_STR_0__TIM8_CH1    DMA2_ST2
#define DEF_TIM_DMA_STR_1__TIM8_CH1    DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM8_CH1N   DMA2_ST2
#define DEF_TIM_DMA_STR_1__TIM8_CH1N   DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM8_CH2    DMA2_ST3
#define DEF_TIM_DMA_STR_1__TIM8_CH2    DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM8_CH2N   DMA2_ST3
#define DEF_TIM_DMA_STR_1__TIM8_CH2N   DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM8_CH3    DMA2_ST4
#define DEF_TIM_DMA_STR_1__TIM8_CH3    DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM8_CH3N   DMA2_ST4
#define DEF_TIM_DMA_STR_1__TIM8_CH3N   DMA2_ST2
#define DEF_TIM_DMA_STR_0__TIM8_CH4    DMA2_ST7

#define DEF_TIM_DMA_STR_0__TIM4_CH4    DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM9_CH1    DMA_NONE
#define DEF_TIM_DMA_STR_0__TIM9_CH2    DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM10_CH1   DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM11_CH1   DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM12_CH1   DMA_NONE
#define DEF_TIM_DMA_STR_0__TIM12_CH2   DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM13_CH1   DMA_NONE

#define DEF_TIM_DMA_STR_0__TIM14_CH1   DMA_NONE

/* F7 Channel Mappings */

#define DEF_TIM_DMA_CHN_0__TIM1_CH1    DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH1    DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_2__TIM1_CH1    DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH1N   DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH1N   DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_2__TIM1_CH1N   DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH2    DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH2    DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH2N   DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH2N   DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH3    DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH3    DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH3N   DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_1__TIM1_CH3N   DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_0__TIM1_CH4    DMA_CHANNEL_6

#define DEF_TIM_DMA_CHN_0__TIM2_CH1    DMA_CHANNEL_3
#define DEF_TIM_DMA_CHN_0__TIM2_CH2    DMA_CHANNEL_3
#define DEF_TIM_DMA_CHN_0__TIM2_CH3    DMA_CHANNEL_3
#define DEF_TIM_DMA_CHN_0__TIM2_CH4    DMA_CHANNEL_3
#define DEF_TIM_DMA_CHN_1__TIM2_CH4    DMA_CHANNEL_3

#define DEF_TIM_DMA_CHN_0__TIM3_CH1    DMA_CHANNEL_5
#define DEF_TIM_DMA_CHN_0__TIM3_CH2    DMA_CHANNEL_5
#define DEF_TIM_DMA_CHN_0__TIM3_CH3    DMA_CHANNEL_5
#define DEF_TIM_DMA_CHN_0__TIM3_CH4    DMA_CHANNEL_5

#define DEF_TIM_DMA_CHN_0__TIM4_CH1    DMA_CHANNEL_2
#define DEF_TIM_DMA_CHN_0__TIM4_CH2    DMA_CHANNEL_2
#define DEF_TIM_DMA_CHN_0__TIM4_CH3    DMA_CHANNEL_2

#define DEF_TIM_DMA_CHN_0__TIM5_CH1    DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_0__TIM5_CH2    DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_0__TIM5_CH3    DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_0__TIM5_CH4    DMA_CHANNEL_6
#define DEF_TIM_DMA_CHN_1__TIM5_CH4    DMA_CHANNEL_6

#define DEF_TIM_DMA_CHN_0__TIM8_CH1    DMA_CHANNEL_7
#define DEF_TIM_DMA_CHN_1__TIM8_CH1    DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_0__TIM8_CH1N   DMA_CHANNEL_7
#define DEF_TIM_DMA_CHN_1__TIM8_CH1N   DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_0__TIM8_CH2    DMA_CHANNEL_7
#define DEF_TIM_DMA_CHN_1__TIM8_CH2    DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_0__TIM8_CH2N   DMA_CHANNEL_7
#define DEF_TIM_DMA_CHN_1__TIM8_CH2N   DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_0__TIM8_CH3    DMA_CHANNEL_7
#define DEF_TIM_DMA_CHN_1__TIM8_CH3    DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_0__TIM8_CH3N   DMA_CHANNEL_7
#define DEF_TIM_DMA_CHN_1__TIM8_CH3N   DMA_CHANNEL_0
#define DEF_TIM_DMA_CHN_0__TIM8_CH4    DMA_CHANNEL_7

#define DEF_TIM_DMA_CHN_0__TIM4_CH4    0

#define DEF_TIM_DMA_CHN_0__TIM9_CH1    0
#define DEF_TIM_DMA_CHN_0__TIM9_CH2    0

#define DEF_TIM_DMA_CHN_0__TIM10_CH1   0

#define DEF_TIM_DMA_CHN_0__TIM11_CH1   0

#define DEF_TIM_DMA_CHN_0__TIM12_CH1   0
#define DEF_TIM_DMA_CHN_0__TIM12_CH2   0

#define DEF_TIM_DMA_CHN_0__TIM13_CH1   0

#define DEF_TIM_DMA_CHN_0__TIM14_CH1   0

#define DMA1_ST0_STREAM                DMA1_Stream0
#define DMA1_ST1_STREAM                DMA1_Stream1
#define DMA1_ST2_STREAM                DMA1_Stream2
#define DMA1_ST3_STREAM                DMA1_Stream3
#define DMA1_ST4_STREAM                DMA1_Stream4
#define DMA1_ST5_STREAM                DMA1_Stream5
#define DMA1_ST6_STREAM                DMA1_Stream6
#define DMA1_ST7_STREAM                DMA1_Stream7
#define DMA2_ST0_STREAM                DMA2_Stream0
#define DMA2_ST1_STREAM                DMA2_Stream1
#define DMA2_ST2_STREAM                DMA2_Stream2
#define DMA2_ST3_STREAM                DMA2_Stream3
#define DMA2_ST4_STREAM                DMA2_Stream4
#define DMA2_ST5_STREAM                DMA2_Stream5
#define DMA2_ST6_STREAM                DMA2_Stream6
#define DMA2_ST7_STREAM                DMA2_Stream7

#define GPIO_AF(p, t) CONCAT(GPIO_AF__, p, _, t)

//PORTA
#define GPIO_AF__PA0_TIM2_CH1     GPIO_AF1_TIM2
#define GPIO_AF__PA1_TIM2_CH2     GPIO_AF1_TIM2
#define GPIO_AF__PA2_TIM2_CH3     GPIO_AF1_TIM2
#define GPIO_AF__PA3_TIM2_CH4     GPIO_AF1_TIM2
#define GPIO_AF__PA5_TIM2_CH1     GPIO_AF1_TIM2
#define GPIO_AF__PA7_TIM1_CH1N    GPIO_AF1_TIM1
#define GPIO_AF__PA8_TIM1_CH1     GPIO_AF1_TIM1
#define GPIO_AF__PA9_TIM1_CH2     GPIO_AF1_TIM1
#define GPIO_AF__PA10_TIM1_CH3    GPIO_AF1_TIM1
#define GPIO_AF__PA11_TIM1_CH1N   GPIO_AF1_TIM1
#define GPIO_AF__PA15_TIM2_CH1    GPIO_AF1_TIM2

#define GPIO_AF__PA0_TIM5_CH1     GPIO_AF2_TIM5
#define GPIO_AF__PA1_TIM5_CH2     GPIO_AF2_TIM5
#define GPIO_AF__PA3_TIM5_CH3     GPIO_AF2_TIM5
#define GPIO_AF__PA4_TIM5_CH4     GPIO_AF2_TIM5
#define GPIO_AF__PA6_TIM3_CH1     GPIO_AF2_TIM3
#define GPIO_AF__PA7_TIM3_CH2     GPIO_AF2_TIM3

#define GPIO_AF__PA2_TIM9_CH1     GPIO_AF3_TIM9
#define GPIO_AF__PA3_TIM9_CH2     GPIO_AF3_TIM9
#define GPIO_AF__PA5_TIM8_CH1N    GPIO_AF3_TIM8
#define GPIO_AF__PA7_TIM8_CH1N    GPIO_AF3_TIM8

#define GPIO_AF__PA6_TIM13_CH1    GPIO_AF9_TIM13
#define GPIO_AF__PA7_TIM14_CH1    GPIO_AF9_TIM14

//PORTB
#define GPIO_AF__PB0_TIM1_CH2N    GPIO_AF1_TIM1
#define GPIO_AF__PB1_TIM1_CH2N    GPIO_AF1_TIM1
#define GPIO_AF__PB3_TIM2_CH2     GPIO_AF1_TIM2
#define GPIO_AF__PB10_TIM2_CH3    GPIO_AF1_TIM2
#define GPIO_AF__PB11_TIM2_CH4    GPIO_AF1_TIM2
#define GPIO_AF__PB13_TIM1_CH1N   GPIO_AF1_TIM1
#define GPIO_AF__PB14_TIM1_CH2N   GPIO_AF1_TIM1
#define GPIO_AF__PB15_TIM1_CH3N   GPIO_AF1_TIM1

#define GPIO_AF__PB0_TIM3_CH3     GPIO_AF2_TIM3
#define GPIO_AF__PB1_TIM3_CH4     GPIO_AF2_TIM3
#define GPIO_AF__PB4_TIM3_CH1     GPIO_AF2_TIM3
#define GPIO_AF__PB5_TIM3_CH2     GPIO_AF2_TIM3
#define GPIO_AF__PB6_TIM4_CH1     GPIO_AF2_TIM4
#define GPIO_AF__PB7_TIM4_CH2     GPIO_AF2_TIM4
#define GPIO_AF__PB8_TIM4_CH3     GPIO_AF2_TIM4
#define GPIO_AF__PB9_TIM4_CH4     GPIO_AF2_TIM4

#define GPIO_AF__PB0_TIM8_CH2N    GPIO_AF3_TIM8
#define GPIO_AF__PB1_TIM8_CH3N    GPIO_AF3_TIM8
#define GPIO_AF__PB8_TIM10_CH1    GPIO_AF3_TIM10
#define GPIO_AF__PB9_TIM11_CH1    GPIO_AF3_TIM11
#define GPIO_AF__PB14_TIM8_CH2N   GPIO_AF3_TIM8
#define GPIO_AF__PB15_TIM8_CH3N   GPIO_AF3_TIM8

#define GPIO_AF__PB14_TIM12_CH1   GPIO_AF9_TIM12
#define GPIO_AF__PB15_TIM12_CH2   GPIO_AF9_TIM12

//PORTC
#define GPIO_AF__PC6_TIM3_CH1     GPIO_AF2_TIM3
#define GPIO_AF__PC7_TIM3_CH2     GPIO_AF2_TIM3
#define GPIO_AF__PC8_TIM3_CH3     GPIO_AF2_TIM3
#define GPIO_AF__PC9_TIM3_CH4     GPIO_AF2_TIM3

#define GPIO_AF__PC6_TIM8_CH1     GPIO_AF3_TIM8
#define GPIO_AF__PC7_TIM8_CH2     GPIO_AF3_TIM8
#define GPIO_AF__PC8_TIM8_CH3     GPIO_AF3_TIM8
#define GPIO_AF__PC9_TIM8_CH4     GPIO_AF3_TIM8

//PORTD
#define GPIO_AF__PD12_TIM4_CH1    GPIO_AF2_TIM4
#define GPIO_AF__PD13_TIM4_CH2    GPIO_AF2_TIM4
#define GPIO_AF__PD14_TIM4_CH3    GPIO_AF2_TIM4
#define GPIO_AF__PD15_TIM4_CH4    GPIO_AF2_TIM4

//PORTE
#define GPIO_AF__PE8_TIM1_CH1N    GPIO_AF1_TIM1
#define GPIO_AF__PE9_TIM1_CH1     GPIO_AF1_TIM1
#define GPIO_AF__PE10_TIM1_CH2N   GPIO_AF1_TIM1
#define GPIO_AF__PE11_TIM1_CH2    GPIO_AF1_TIM1
#define GPIO_AF__PE12_TIM1_CH3N   GPIO_AF1_TIM1
#define GPIO_AF__PE13_TIM1_CH3    GPIO_AF1_TIM1
#define GPIO_AF__PE14_TIM1_CH4    GPIO_AF1_TIM1

#define GPIO_AF__PE5_TIM9_CH1     GPIO_AF3_TIM9
#define GPIO_AF__PE6_TIM9_CH2     GPIO_AF3_TIM9

//PORTF
#define GPIO_AF__PF6_TIM10_CH1    GPIO_AF3_TIM10
#define GPIO_AF__PF7_TIM11_CH1    GPIO_AF3_TIM11

//PORTH
#define GPIO_AF__PH10_TIM5_CH1    GPIO_AF2_TIM5
#define GPIO_AF__PH11_TIM5_CH2    GPIO_AF2_TIM5
#define GPIO_AF__PH12_TIM5_CH3    GPIO_AF2_TIM5

#define GPIO_AF__PH13_TIM8_CH1N   GPIO_AF3_TIM8
#define GPIO_AF__PH14_TIM8_CH2N   GPIO_AF3_TIM8
#define GPIO_AF__PH15_TIM8_CH3N   GPIO_AF3_TIM8

#define GPIO_AF__PH6_TIM12_CH1    GPIO_AF9_TIM12
#define GPIO_AF__PH9_TIM12_CH2    GPIO_AF9_TIM12

//PORTI
#define GPIO_AF__PI0_TIM5_CH4     GPIO_AF2_TIM5

#define GPIO_AF__PI2_TIM8_CH4     GPIO_AF3_TIM8
#define GPIO_AF__PI5_TIM8_CH1     GPIO_AF3_TIM8
#define GPIO_AF__PI6_TIM8_CH2     GPIO_AF3_TIM8
#define GPIO_AF__PI7_TIM8_CH3     GPIO_AF3_TIM8

#endif

/**** Common Defines across all targets ****/
#define DMA_NONE_CHANNEL     NULL
#define DMA_NONE_STREAM      NULL


#define DEF_TIM_CHAN(chan) DEF_CHAN_ ## chan
#define DEF_TIM_OUTPUT(chan, out) ( DEF_CHAN_ ## chan ## _OUTPUT | out )

#define DMA_NONE_HANDLER     0

#if defined(STM32F7)
#define DEF_CHAN_CH1         TIM_CHANNEL_1
#define DEF_CHAN_CH2         TIM_CHANNEL_2
#define DEF_CHAN_CH3         TIM_CHANNEL_3
#define DEF_CHAN_CH4         TIM_CHANNEL_4
#define DEF_CHAN_CH1N        TIM_CHANNEL_1
#define DEF_CHAN_CH2N        TIM_CHANNEL_2
#define DEF_CHAN_CH3N        TIM_CHANNEL_3
#define DEF_CHAN_CH4N        TIM_CHANNEL_4
#else
#define DEF_CHAN_CH1         TIM_Channel_1
#define DEF_CHAN_CH2         TIM_Channel_2
#define DEF_CHAN_CH3         TIM_Channel_3
#define DEF_CHAN_CH4         TIM_Channel_4
#define DEF_CHAN_CH1N        TIM_Channel_1
#define DEF_CHAN_CH2N        TIM_Channel_2
#define DEF_CHAN_CH3N        TIM_Channel_3
#define DEF_CHAN_CH4N        TIM_Channel_4
#endif

#define DEF_CHAN_CH1_OUTPUT  TIMER_OUTPUT_NONE
#define DEF_CHAN_CH2_OUTPUT  TIMER_OUTPUT_NONE
#define DEF_CHAN_CH3_OUTPUT  TIMER_OUTPUT_NONE
#define DEF_CHAN_CH4_OUTPUT  TIMER_OUTPUT_NONE
#define DEF_CHAN_CH1N_OUTPUT TIMER_OUTPUT_N_CHANNEL
#define DEF_CHAN_CH2N_OUTPUT TIMER_OUTPUT_N_CHANNEL
#define DEF_CHAN_CH3N_OUTPUT TIMER_OUTPUT_N_CHANNEL
#define DEF_CHAN_CH4N_OUTPUT TIMER_OUTPUT_N_CHANNEL

#define DMA1_CH1_CHANNEL     DMA1_Channel1
#define DMA1_CH2_CHANNEL     DMA1_Channel2
#define DMA1_CH3_CHANNEL     DMA1_Channel3
#define DMA1_CH4_CHANNEL     DMA1_Channel4
#define DMA1_CH5_CHANNEL     DMA1_Channel5
#define DMA1_CH6_CHANNEL     DMA1_Channel6
#define DMA1_CH7_CHANNEL     DMA1_Channel7
#define DMA2_CH1_CHANNEL     DMA2_Channel1
#define DMA2_CH2_CHANNEL     DMA2_Channel2
#define DMA2_CH3_CHANNEL     DMA2_Channel3
#define DMA2_CH4_CHANNEL     DMA2_Channel4
#define DMA2_CH5_CHANNEL     DMA2_Channel5
#define DMA2_CH6_CHANNEL     DMA2_Channel6
#define DMA2_CH7_CHANNEL     DMA2_Channel7
