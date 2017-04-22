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
#ifndef CRAZYFLIE2_USE_BIG_QUAD_DECK
    { TIM2, IO_TAG(PB11), TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_TIM2, NULL, 0, 0 }, // PWM2 - OUT2 (Motor 2)
    { TIM2, IO_TAG(PA1),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_TIM2, NULL, 0, 0 }, // PWM1 - OUT1 (Motor 1)
    { TIM2, IO_TAG(PA15), TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_TIM2, NULL, 0, 0 }, // PWM3 - OUT3 (Motor 3)
    { TIM4, IO_TAG(PB9),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_TIM4, NULL, 0, 0 }, // PWM4 - OUT4 (Motor 4)
#else
    { TIM3, IO_TAG(PB4), TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_TIM3, NULL, 0, 0 }, // PWM2 - OUT2 (Motor 2)
    { TIM2, IO_TAG(PA2), TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_TIM2, NULL, 0, 0 }, // PWM1 - OUT1 (Motor 1)
    { TIM3, IO_TAG(PB5), TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_TIM3, NULL, 0, 0 }, // PWM3 - OUT3 (Motor 3)
    { TIM2, IO_TAG(PA3), TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_TIM2, NULL, 0, 0 }, // PWM4 - OUT4 (Motor 4)
#endif
};
