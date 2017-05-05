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
    { TIM1,  IO_TAG(PA8),  TIM_CHANNEL_1, 0,      IOCFG_AF_PP_PD, GPIO_AF1_TIM1,  TIM_USE_PPM | TIM_USE_PWM | TIM_USE_LED           }, // PWM1  - PA8  RC1
    { TIM3,  IO_TAG(PB1),  TIM_CHANNEL_4, 0,      IOCFG_AF_PP_PD, GPIO_AF2_TIM3,  TIM_USE_PWM | TIM_USE_MC_SERVO | TIM_USE_MC_CHNFW }, // PWM2  - PB1  RC2
    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, 0,      IOCFG_AF_PP_PD, GPIO_AF9_TIM12, TIM_USE_PWM | TIM_USE_MC_SERVO | TIM_USE_MC_CHNFW }, // PWM3  - PA15 RC3
    { TIM4,  IO_TAG(PB8),  TIM_CHANNEL_3, 0,      IOCFG_AF_PP_PD, GPIO_AF2_TIM4,  TIM_USE_PWM | TIM_USE_MC_SERVO | TIM_USE_MC_CHNFW }, // PWM4  - PB8  RC4
    { TIM4,  IO_TAG(PB9),  TIM_CHANNEL_4, 0,      IOCFG_AF_PP_PD, GPIO_AF2_TIM4,  TIM_USE_PWM | TIM_USE_MC_SERVO | TIM_USE_MC_CHNFW }, // PWM5  - PB9  RC5
    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM_EN, IOCFG_AF_PP_PD, GPIO_AF3_TIM8,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO               }, // PWM6  - PC6  OUT1
    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM_EN, IOCFG_AF_PP_PD, GPIO_AF3_TIM8,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO               }, // PWM7  - PC7  OUT2
    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, TIM_EN, IOCFG_AF_PP_PD, GPIO_AF9_TIM12, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO               }, // PWM9  - PA14 OUT3
    { TIM3,  IO_TAG(PB0),  TIM_CHANNEL_3, TIM_EN, IOCFG_AF_PP_PD, GPIO_AF2_TIM3,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO               }, // PWM10 - PB0  OUT4
    { TIM5,  IO_TAG(PA0),  TIM_CHANNEL_1, TIM_EN, IOCFG_AF_PP_PD, GPIO_AF2_TIM5,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO               }, // PWM11 - PA0  OUT5
    { TIM5,  IO_TAG(PA1),  TIM_CHANNEL_2, TIM_EN, IOCFG_AF_PP_PD, GPIO_AF2_TIM5,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO               }, // PWM12 - PA1  OUT6
    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM_EN, IOCFG_AF_PP_PD, GPIO_AF3_TIM8,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO               }, // PWM13 - PC8  OUT7
    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM_EN, IOCFG_AF_PP_PD, GPIO_AF3_TIM8,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO               }, // PWM14 - PC9  OUT8
};
