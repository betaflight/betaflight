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
    { TIM1,  IO_TAG(PE9),  TIM_Channel_1, TIM1_CC_IRQn,        0, IOCFG_AF_PP, GPIO_AF_TIM1 }, // PPM
    { TIM1,  IO_TAG(PE11), TIM_Channel_2, TIM1_CC_IRQn,        0, IOCFG_AF_PP, GPIO_AF_TIM1 }, // S2_IN
    { TIM1,  IO_TAG(PE13), TIM_Channel_3, TIM1_CC_IRQn,        0, IOCFG_AF_PP, GPIO_AF_TIM1 }, // S3_IN
    { TIM1,  IO_TAG(PE14), TIM_Channel_4, TIM1_CC_IRQn,        0, IOCFG_AF_PP, GPIO_AF_TIM1 }, // S4_IN
    { TIM9,  IO_TAG(PE6),  TIM_Channel_1, TIM1_BRK_TIM9_IRQn,  0, IOCFG_AF_PP, GPIO_AF_TIM9 }, // S5_IN
    { TIM9,  IO_TAG(PE7),  TIM_Channel_2, TIM1_BRK_TIM9_IRQn,  0, IOCFG_AF_PP, GPIO_AF_TIM9 }, // S6_IN

    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,           1, IOCFG_AF_PP, GPIO_AF_TIM2 }, // S1_OUT
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM2_IRQn,           1, IOCFG_AF_PP, GPIO_AF_TIM2 }, // S2_OUT
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, TIM2_IRQn,           1, IOCFG_AF_PP, GPIO_AF_TIM2 }, // S3_OUT
    { TIM3,  IO_TAG(PB5),  TIM_Channel_2, TIM3_IRQn,           1, IOCFG_AF_PP, GPIO_AF_TIM3 }, // S4_OUT
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,           1, IOCFG_AF_PP, GPIO_AF_TIM3 }, // S5_OUT
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,           1, IOCFG_AF_PP, GPIO_AF_TIM3 }, // S6_OUT
};
