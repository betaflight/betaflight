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
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] =
{
    { TIM2, IO_TAG(PA0),  TIM_Channel_1, 0, IOCFG_AF_PP, GPIO_AF_1, TIM_USE_PPM }, // // PPM - PA0  - *TIM2_CH1
    { TIM4, IO_TAG(PB9),  TIM_Channel_4, 1, IOCFG_IPD,   GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR },            // PWM1
    { TIM3, IO_TAG(PB0),  TIM_Channel_3, 1, IOCFG_IPD,   GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },            // PWM2
    { TIM3, IO_TAG(PB1),  TIM_Channel_4, 1, IOCFG_IPD,   GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },            // PWM3
    { TIM4, IO_TAG(PB7),  TIM_Channel_2, 1, IOCFG_IPD,   GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },            // PWM4
    { TIM4, IO_TAG(PB8),  TIM_Channel_3, 1, IOCFG_IPD,   GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },            // PWM5
    { TIM4, IO_TAG(PB6),  TIM_Channel_1, 1, IOCFG_IPD,   GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },            // PWM6
    { TIM2, IO_TAG(PB10), TIM_Channel_3, 1, IOCFG_AF_PP, GPIO_AF_1, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },            // PWM7
    { TIM2, IO_TAG(PB11), TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_1, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },            // PWM8
    { TIM1, IO_TAG(PA8),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_6, TIM_USE_MC_SERVO | TIM_USE_ANY | TIM_USE_LED },                 // S1_out
    { TIM2, IO_TAG(PA1),  TIM_Channel_2, 1, IOCFG_IPD,   GPIO_AF_1, TIM_USE_MC_SERVO | TIM_USE_ANY },                 // S2_out
};
