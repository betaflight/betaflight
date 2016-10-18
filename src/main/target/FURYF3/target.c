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

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {

    { TIM2,  IO_TAG(PB3),  TIM_Channel_2, TIM2_IRQn,               0, IOCFG_AF_PP, GPIO_AF_1, NULL, 0 }, // PPM IN
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,               0, IOCFG_AF_PP, GPIO_AF_2, NULL, 0 }, // SS1 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM1,  IO_TAG(PB1),  TIM_Channel_3, TIM1_CC_IRQn,            0, IOCFG_AF_PP, GPIO_AF_6, NULL, 0 }, // SS1 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N

    { TIM3,  IO_TAG(PB7),  TIM_Channel_4, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_10,  DMA1_Channel3, DMA1_CH3_HANDLER }, // PWM4 - S1
    { TIM8,  IO_TAG(PB6),  TIM_Channel_1, TIM8_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_5,   DMA2_Channel3, DMA2_CH3_HANDLER }, // PWM5 - S2
    { TIM8,  IO_TAG(PB5),  TIM_Channel_3, TIM8_CC_IRQn, (1 | TIMER_OUTPUT_N_CHANNEL), IOCFG_AF_PP, GPIO_AF_3,   DMA2_Channel1, DMA2_CH1_HANDLER }, // PWM6 - S3
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2,   DMA1_Channel6, DMA1_CH6_HANDLER }, // PWM7 - S4

    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_6, NULL, 0 }, // GPIO TIMER - LED_STRIP

};

