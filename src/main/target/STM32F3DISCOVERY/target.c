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
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_6, TIM_USE_PWM | TIM_USE_PPM }, // PWM1 - PA8
    { TIM16, IO_TAG(PB8),  TIM_Channel_1, 0, IOCFG_AF_PP_PD, GPIO_AF_1, TIM_USE_PWM | TIM_USE_LED }, // PWM2 - PB8
    { TIM17, IO_TAG(PB9),  TIM_Channel_1, 0, IOCFG_AF_PP_PD, GPIO_AF_1, TIM_USE_PWM }, // PWM3 - PB9
    { TIM8,  IO_TAG(PC6),  TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_4, TIM_USE_PWM }, // PWM4 - PC6
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_4, TIM_USE_PWM }, // PWM5 - PC7
    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, 1, IOCFG_AF_PP_PD, GPIO_AF_4, TIM_USE_PWM }, // PWM6 - PC8
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, 0, IOCFG_AF_PP_PD, GPIO_AF_2, TIM_USE_PWM }, // PWM7 - PB1
    { TIM3,  IO_TAG(PA4),  TIM_Channel_2, 0, IOCFG_AF_PP_PD, GPIO_AF_2, TIM_USE_PWM }, // PWM8 - PA2
    { TIM4,  IO_TAG(PD12), TIM_Channel_1, 0, IOCFG_AF_PP,    GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR }, // PWM9 - PD12
    { TIM4,  IO_TAG(PD13), TIM_Channel_2, 0, IOCFG_AF_PP,    GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR }, // PWM10 - PD13
    { TIM4,  IO_TAG(PD14), TIM_Channel_3, 0, IOCFG_AF_PP,    GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // PWM11 - PD14
    { TIM4,  IO_TAG(PD15), TIM_Channel_4, 0, IOCFG_AF_PP,    GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // PWM12 - PD15
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, 0, IOCFG_AF_PP,    GPIO_AF_1, TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO }, // PWM13 - PA1
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, 0, IOCFG_AF_PP,    GPIO_AF_1, TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO }  // PWM14 - PA2
};

