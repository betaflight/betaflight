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
#include "drivers/dma.h"
#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM_USE_PPM,   0, GPIO_AF_6, NULL, 0 }, // PWM1 - PA8
    { TIM3,  IO_TAG(PC6),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_2, DMA1_Channel6, DMA1_CH6_HANDLER }, // PWM2 - PC6
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_4, DMA2_Channel5, DMA2_CH5_HANDLER }, // PWM3 - PC7
    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_4, DMA2_Channel1, DMA2_CH1_HANDLER }, // PMW4 - PC8
    { TIM8,  IO_TAG(PC9),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_4, DMA2_Channel2, DMA2_CH2_HANDLER }, // PWM5 - PC9
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM6 - PA0
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM7 - PA1
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM8 - PA2
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM9 - PA3
    { TIM15, IO_TAG(PB14), TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM10 - PB14
    { TIM15, IO_TAG(PB15), TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM11 - PB15
    { TIM16, IO_TAG(PA6),  TIM_Channel_1, TIM_USE_LED,   1, GPIO_AF_1, DMA1_Channel3, DMA1_CH3_HANDLER }, // PWM11 - PB15
};
