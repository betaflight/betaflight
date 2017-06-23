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

#define TIM_EN      TIMER_OUTPUT_ENABLED
#define TIM_EN_N    TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_N_CHANNEL

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, 0, IOCFG_AF_PP_PD, GPIO_AF9_TIM12,    TIM_USE_PWM | TIM_USE_PPM }, // S1_IN
    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, 0, IOCFG_AF_PP_PD, GPIO_AF9_TIM12,    TIM_USE_PWM }, // S2_IN
    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, 0, IOCFG_AF_PP_PD, GPIO_AF3_TIM8,     TIM_USE_PWM }, // S3_IN
    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, 0, IOCFG_AF_PP_PD, GPIO_AF3_TIM8,     TIM_USE_PWM }, // S4_IN
    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, 0, IOCFG_AF_PP_PD, GPIO_AF3_TIM8,     TIM_USE_PWM }, // S5_IN
    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, 0, IOCFG_AF_PP_PD, GPIO_AF3_TIM8,     TIM_USE_PWM }, // S6_IN

    { TIM4,  IO_TAG(PB8),  TIM_CHANNEL_3, TIM_EN,   IOCFG_AF_PP_PD, GPIO_AF2_TIM4, TIM_USE_MC_MOTOR |   TIM_USE_FW_SERVO }, // S10_OUT PWM16
    { TIM2,  IO_TAG(PA2),  TIM_CHANNEL_3, TIM_EN,   IOCFG_AF_PP_PD, GPIO_AF1_TIM2, TIM_USE_MC_MOTOR |   TIM_USE_FW_SERVO }, // S6_OUT  PWM12
    { TIM4,  IO_TAG(PB7),  TIM_CHANNEL_2, TIM_EN,   IOCFG_AF_PP_PD, GPIO_AF2_TIM4, TIM_USE_MC_MOTOR |   TIM_USE_FW_SERVO }, // S5_OUT  PWM11
    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, TIM_EN,   IOCFG_AF_PP_PD, GPIO_AF1_TIM2, TIM_USE_MC_MOTOR |   TIM_USE_FW_MOTOR }, // S1_OUT  PWM7
    { TIM5,  IO_TAG(PA1),  TIM_CHANNEL_2, TIM_EN,   IOCFG_AF_PP_PD, GPIO_AF2_TIM5, TIM_USE_MC_MOTOR |   TIM_USE_FW_MOTOR }, // S2_OUT  PWM8
    { TIM1,  IO_TAG(PB0),  TIM_CHANNEL_2, TIM_EN_N, IOCFG_AF_PP_PD, GPIO_AF1_TIM1, TIM_USE_MC_MOTOR |   TIM_USE_FW_SERVO }, // S3_OUT PWM9
    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, TIM_EN,   IOCFG_AF_PP_PD, GPIO_AF2_TIM3, TIM_USE_MC_MOTOR |   TIM_USE_FW_SERVO }, // S4_OUT  PWM10
    { TIM5,  IO_TAG(PA0),  TIM_CHANNEL_1, TIM_EN,   IOCFG_AF_PP_PD, GPIO_AF2_TIM5, TIM_USE_MC_MOTOR |   TIM_USE_FW_SERVO }, // S7_OUT  PWM13
    { TIM1,  IO_TAG(PB1),  TIM_CHANNEL_3, TIM_EN_N, IOCFG_AF_PP_PD, GPIO_AF1_TIM1, TIM_USE_MC_MOTOR |   TIM_USE_FW_SERVO }, // S8_OUT PWM14
    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, TIM_EN,   IOCFG_AF_PP_PD, GPIO_AF2_TIM3, TIM_USE_MC_MOTOR |   TIM_USE_FW_SERVO }, // S9_OUT  PWM15
};

