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

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, IO_TAG(PA10),  TIM_Channel_3, TIM_USE_PWM | TIM_USE_PPM, 0, GPIO_AF_TIM1, NULL, 0, 0 }, // S1_IN
    { TIM8, IO_TAG(PC6),   TIM_Channel_1, TIM_USE_PWM,   0, GPIO_AF_TIM8 , NULL, 0, 0 }, // S2_IN
    { TIM8, IO_TAG(PC7),   TIM_Channel_2, TIM_USE_PWM,   0, GPIO_AF_TIM8 , NULL, 0, 0 }, // S3_IN
    { TIM8, IO_TAG(PC8),   TIM_Channel_3, TIM_USE_PWM,   0, GPIO_AF_TIM8 , NULL, 0, 0 }, // S4_IN
    { TIM2, IO_TAG(PA15),  TIM_Channel_1, TIM_USE_PWM,   0, GPIO_AF_TIM2 , NULL, 0, 0 }, // S5_IN
    { TIM2, IO_TAG(PB3),   TIM_Channel_2, TIM_USE_PWM,   0, GPIO_AF_TIM2 , NULL, 0, 0 }, // S6_IN
    { TIM5, IO_TAG(PA0),   TIM_Channel_1, TIM_USE_PWM,   0, GPIO_AF_TIM5 , NULL, 0, 0 }, // S7_IN
    { TIM5, IO_TAG(PA1),   TIM_Channel_2, TIM_USE_PWM,   0, GPIO_AF_TIM5 , NULL, 0, 0 }, // S8_IN
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_TIM3 , NULL, 0, 0 }, // S1_OUT
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_TIM3 , NULL, 0, 0 }, // S2_OUT
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_TIM3 , NULL, 0, 0 }, // S3_OUT
    { TIM12, IO_TAG(PB15), TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_TIM12, NULL, 0, 0 }, // S4_OUT
    { TIM3,  IO_TAG(PB5),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_TIM3 , NULL, 0, 0 }, // S5_OUT
    { TIM12, IO_TAG(PB14), TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_TIM12, NULL, 0, 0 }, // S6_OUT
    { TIM10, IO_TAG(PB8),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_TIM10, NULL, 0, 0 }, // S7_OUT
    { TIM11, IO_TAG(PB9),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_TIM11, NULL, 0, 0 }, // S8_OUT
    { TIM4,  IO_TAG(PB7),  TIM_Channel_2, TIM_USE_LED  , 0, GPIO_AF_TIM11, DMA1_Stream3, DMA_Channel_2, DMA1_ST3_HANDLER }, // S8_OUT
};
