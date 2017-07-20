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

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, 0, IOCFG_AF_PP_PD, GPIO_AF_6, TIM_USE_PPM }, // PWM1 - PA8
    { TIM3,  IO_TAG(PC6),  TIM_Channel_1, 1, IOCFG_AF_PP,    GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR }, // PWM2 - PC6
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, 1, IOCFG_AF_PP,    GPIO_AF_4, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // PWM3 - PC7
    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, 1, IOCFG_AF_PP,    GPIO_AF_4, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // PMW4 - PC8
    { TIM8,  IO_TAG(PC9),  TIM_Channel_4, 1, IOCFG_AF_PP,    GPIO_AF_4, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // PWM5 - PC9
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, 1, IOCFG_AF_PP,    GPIO_AF_1, TIM_USE_MC_MOTOR }, // PWM6 - PA0
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, 1, IOCFG_AF_PP,    GPIO_AF_1, TIM_USE_MC_MOTOR }, // PWM7 - PA1
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, 1, IOCFG_AF_PP,    GPIO_AF_1, TIM_USE_MC_MOTOR }, // PWM8 - PA2
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, 1, IOCFG_AF_PP,    GPIO_AF_1, TIM_USE_MC_MOTOR }, // PWM9 - PA3
    { TIM15, IO_TAG(PB14), TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_1, TIM_USE_MC_MOTOR }, // PWM10 - PB14
    { TIM15, IO_TAG(PB15), TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_1, TIM_USE_MC_MOTOR }, // PWM11 - PB15
    { TIM16, IO_TAG(PA6),  TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_1, TIM_USE_LED },
};

