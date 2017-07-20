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
    { TIM16, IO_TAG(PA6),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_1,    TIM_USE_MC_MOTOR |                    TIM_USE_FW_MOTOR }, // PWM1
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_1,    TIM_USE_MC_MOTOR |                    TIM_USE_FW_MOTOR }, // PWM2
    { TIM4,  IO_TAG(PA11), TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_10,   TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO }, // PWM3
    { TIM4,  IO_TAG(PA12), TIM_Channel_2, 1, IOCFG_AF_PP, GPIO_AF_10,   TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO }, // PWM4
    { TIM4,  IO_TAG(PB8),  TIM_Channel_3, 1, IOCFG_AF_PP, GPIO_AF_2,    TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO }, // PWM5
    { TIM4,  IO_TAG(PB9),  TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_2,    TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO }, // PWM6
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_9,    TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO }, // PWM7
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, 1, IOCFG_AF_PP, GPIO_AF_9,    TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO }, // PWM8

    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_6,    TIM_USE_LED }, // GPIO_TIMER / LED_STRIP

    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, 0, IOCFG_AF_PP, GPIO_AF_1,    TIM_USE_PPM | TIM_USE_PWM },        // RC_CH1
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, 0, IOCFG_AF_PP, GPIO_AF_1,    TIM_USE_PWM},                       // RC_CH2
    { TIM2,  IO_TAG(PB11), TIM_Channel_4, 0, IOCFG_AF_PP, GPIO_AF_1,    TIM_USE_PWM },                      // RC_CH3
    { TIM2,  IO_TAG(PB10), TIM_Channel_3, 0, IOCFG_AF_PP, GPIO_AF_1,    TIM_USE_PWM },                      // RC_CH4
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, 0, IOCFG_AF_PP, GPIO_AF_2,    TIM_USE_PWM | TIM_USE_MC_CHNFW },   // RC_CH5
    { TIM3,  IO_TAG(PB5),  TIM_Channel_2, 0, IOCFG_AF_PP, GPIO_AF_2,    TIM_USE_PWM | TIM_USE_MC_CHNFW },   // RC_CH6
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, 0, IOCFG_AF_PP, GPIO_AF_2,    TIM_USE_PWM | TIM_USE_MC_CHNFW },   // RC_CH7
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, 0, IOCFG_AF_PP, GPIO_AF_2,    TIM_USE_PWM | TIM_USE_MC_CHNFW },   // RC_CH8
};

