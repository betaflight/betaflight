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
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, IO_TAG(PA10), TIM_Channel_3, 0, IOCFG_IPD, GPIO_AF_TIM1,    TIM_USE_PPM | TIM_USE_PWM }, // S1_IN
    { TIM8, IO_TAG(PC6),  TIM_Channel_1, 0, IOCFG_IPD, GPIO_AF_TIM8,    TIM_USE_PWM }, // S2_IN
    { TIM8, IO_TAG(PC7),  TIM_Channel_2, 0, IOCFG_IPD, GPIO_AF_TIM8,    TIM_USE_PWM }, // S3_IN
    { TIM8, IO_TAG(PC8),  TIM_Channel_3, 0, IOCFG_IPD, GPIO_AF_TIM8,    TIM_USE_PWM }, // S4_IN
    { TIM2, IO_TAG(PA15), TIM_Channel_1, 0, IOCFG_IPD, GPIO_AF_TIM2,    TIM_USE_PWM }, // S5_IN
    { TIM2, IO_TAG(PB3),  TIM_Channel_2, 0, IOCFG_IPD, GPIO_AF_TIM2,    TIM_USE_PWM }, // S6_IN
    { TIM5, IO_TAG(PA0),  TIM_Channel_1, 0, IOCFG_IPD, GPIO_AF_TIM5,    TIM_USE_PWM }, // S7_IN
    { TIM5, IO_TAG(PA1),  TIM_Channel_2, 0, IOCFG_IPD, GPIO_AF_TIM5,    TIM_USE_PWM }, // S8_IN

    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM3,  TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO }, // S1_OUT
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM3,  TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO }, // S2_OUT
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM3,  TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO }, // S3_OUT
    { TIM12, IO_TAG(PB15), TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM12, TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO }, // S4_OUT
    { TIM3,  IO_TAG(PB5),  TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM3,  TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO }, // S5_OUT
    { TIM12, IO_TAG(PB14), TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM12, TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO }, // S6_OUT
    { TIM10, IO_TAG(PB8),  TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM10, TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_MOTOR }, // S7_OUT
    { TIM11, IO_TAG(PB9),  TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM11, TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_MOTOR }, // S8_OUT
};
