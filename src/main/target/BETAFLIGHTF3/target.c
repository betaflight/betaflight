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
 //Target code By BorisB and Hector "Hectech FPV" Hind

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
#include "drivers/timer.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3,  IO_TAG(PB7),  TIM_Channel_4, TIM3_IRQn,               0, IOCFG_AF_PP, GPIO_AF_1, NULL, 0 }, // RC_CH1 - PA0  - *TIM2_CH1

    { TIM16, IO_TAG(PA6),  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, IOCFG_AF_PP, GPIO_AF_1, DMA1_Channel3, DMA1_CH3_HANDLER }, // PWM1 - PA6  - TIM3_CH1, TIM8_BKIN, TIM1_BKIN, *TIM16_CH1
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, IOCFG_AF_PP, GPIO_AF_1, DMA1_Channel1, DMA1_CH1_HANDLER }, // PWM2 - PA7  - TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    { TIM8,  IO_TAG(PB8),  TIM_Channel_2, TIM8_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_10,DMA2_Channel5, DMA2_CH5_HANDLER }, // PWM3 - PB8
    { TIM8,  IO_TAG(PB9),  TIM_Channel_3, TIM8_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_10,DMA2_Channel1, DMA2_CH1_HANDLER }, // PWM4 - PB9
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               1, IOCFG_AF_PP, GPIO_AF_1, NULL, 0 }, // PWM5 - PA1
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM2_IRQn,               1, IOCFG_AF_PP, GPIO_AF_1, NULL, 0 }, // PWM6 - PA0
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP, GPIO_AF_9, NULL, 0 }, // PWM7 - PA2
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP, GPIO_AF_9, NULL, 0 }, // PWM8 - PA3

    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_6, NULL, 0 }, // GPIO_TIMER / LED_STRIP
};

