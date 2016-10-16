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
    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP }, // S1_IN
    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP }, // S2_IN
    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM8_CC_IRQn,        0, IOCFG_AF_PP }, // S3_IN
    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM8_CC_IRQn,        0, IOCFG_AF_PP }, // S4_IN
    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM8_CC_IRQn,        0, IOCFG_AF_PP }, // S5_IN
    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM8_CC_IRQn,        0, IOCFG_AF_PP }, // S6_IN

    { TIM4,  IO_TAG(PB8),  TIM_CHANNEL_3, TIM4_IRQn,           1, IOCFG_AF_PP }, // S10_OUT 1
    { TIM2,  IO_TAG(PA2),  TIM_CHANNEL_3, TIM2_IRQn,           1, IOCFG_AF_PP }, // S6_OUT  2
    { TIM4,  IO_TAG(PB9),  TIM_CHANNEL_4, TIM4_IRQn,           1, IOCFG_AF_PP }, // S5_OUT  3
    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, TIM2_IRQn,           1, IOCFG_AF_PP }, // S1_OUT  4
    { TIM5,  IO_TAG(PA1),  TIM_CHANNEL_2, TIM5_IRQn,           1, IOCFG_AF_PP }, // S2_OUT
    { TIM9,  IO_TAG(PE6),  TIM_CHANNEL_2, TIM1_BRK_TIM9_IRQn,  1, IOCFG_AF_PP }, // S3_OUT
    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, TIM3_IRQn,           1, IOCFG_AF_PP }, // S4_OUT
    { TIM5,  IO_TAG(PA0),  TIM_CHANNEL_1, TIM5_IRQn,           1, IOCFG_AF_PP }, // S7_OUT
    { TIM2,  IO_TAG(PB3),  TIM_CHANNEL_2, TIM2_IRQn,           1, IOCFG_AF_PP }, // S8_OUT
    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, TIM3_IRQn,           1, IOCFG_AF_PP }, // S9_OUT
};

// ALTERNATE LAYOUT
//const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
//    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP }, // S1_IN
//    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP }, // S2_IN
//    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM8_CC_IRQn,        0, IOCFG_AF_PP }, // S3_IN
//    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM8_CC_IRQn,        0, IOCFG_AF_PP }, // S4_IN
//    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM8_CC_IRQn,        0, IOCFG_AF_PP }, // S5_IN
//    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM8_CC_IRQn,        0, IOCFG_AF_PP }, // S6_IN
//
//    { TIM10, IO_TAG(PB8),  TIM_CHANNEL_1, TIM1_UP_TIM10_IRQn,      1, IOCFG_AF_PP }, // S10_OUT
//    { TIM9,  IO_TAG(PA2),  TIM_CHANNEL_1, TIM1_BRK_TIM9_IRQn,      1, IOCFG_AF_PP }, // S6_OUT
//    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, TIM2_IRQn,               1, IOCFG_AF_PP }, // S1_OUT
//    { TIM11, IO_TAG(PB9),  TIM_CHANNEL_1, TIM1_TRG_COM_TIM11_IRQn, 1, IOCFG_AF_PP }, // S5_OUT
//    { TIM5,  IO_TAG(PA1),  TIM_CHANNEL_2, TIM5_IRQn,               1, IOCFG_AF_PP }, // S2_OUT
//    { TIM9,  IO_TAG(PE6),  TIM_CHANNEL_2, TIM1_BRK_TIM9_IRQn,      1, IOCFG_AF_PP }, // S3_OUT
//    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, TIM3_IRQn,               1, IOCFG_AF_PP }, // S4_OUT
//    { TIM5,  IO_TAG(PA0),  TIM_CHANNEL_1, TIM5_IRQn,               1, IOCFG_AF_PP }, // S7_OUT
//    { TIM2,  IO_TAG(PB3),  TIM_CHANNEL_2, TIM2_IRQn,               1, IOCFG_AF_PP }, // S8_OUT
//    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, TIM3_IRQn,               1, IOCFG_AF_PP }, // S9_OUT
//};
