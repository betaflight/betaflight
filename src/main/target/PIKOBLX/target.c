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

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3,  IO_TAG(PA4),  TIM_Channel_2, TIM3_IRQn,               TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM1  - PA4  - *TIM3_CH2
    { TIM3,  IO_TAG(PA6),  TIM_Channel_1, TIM3_IRQn,               TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM2  - PA6  - *TIM3_CH1, TIM8_BKIN, TIM1_BKIN, TIM16_CH1
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,               TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM3  - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM4  - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               TIM_USE_MOTOR, 1, GPIO_AF_1 }, // PWM5  - PA1  - *TIM2_CH2, TIM15_CH1N
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM2_IRQn,               TIM_USE_MOTOR, 1, GPIO_AF_1 }, // PWM6  - PA2  - *TIM2_CH3, !TIM15_CH1
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     TIM_USE_MOTOR, 1, GPIO_AF_9 }, // PWM7  - PA3  - *TIM15_CH2, TIM2_CH4
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            TIM_USE_MOTOR, 1, GPIO_AF_6 }, // PWM8  - PA8  - *TIM1_CH1, TIM4_ETR
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, TIM_USE_PPM,   0, GPIO_AF_1 }, // PPM   - PA7  - *TIM17_CH1, TIM1_CH1N, TIM8_CH1
};

