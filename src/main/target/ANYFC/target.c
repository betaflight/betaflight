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
    { TIM12, IO_TAG(PB14), TIM_Channel_1, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM12, TIM_USE_PPM | TIM_USE_PWM }, // S1_IN
    { TIM12, IO_TAG(PB15), TIM_Channel_2, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM12, TIM_USE_PWM }, // S2_IN
    { TIM8,  IO_TAG(PC6),  TIM_Channel_1, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM8,  TIM_USE_PWM }, // S3_IN
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM8,  TIM_USE_PWM }, // S4_IN
    { TIM8,  IO_TAG(PC9),  TIM_Channel_4, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM8,  TIM_USE_PWM }, // S5_IN
    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM8,  TIM_USE_PWM }, // S6_IN

    { TIM2,  IO_TAG(PB3),  TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM2,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S10_OUT 16
    { TIM5,  IO_TAG(PA0),  TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM5,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S6_OUT 12
    { TIM5,  IO_TAG(PA1),  TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM5,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S5_OUT 11
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM3,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_MOTOR }, // S1_OUT 7
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM3,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_MOTOR }, // S2_OUT 8
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM2,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S3_OUT 9
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM2,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S4_OUT 10
    { TIM2,  IO_TAG(PA15), TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM2,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S7_OUT 13
    { TIM3,  IO_TAG(PB5),  TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM3,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S8_OUT 14
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM3,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S9_OUT 15
};
