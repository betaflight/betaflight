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
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_6, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // PWM3 - PA8
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_1, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR | TIM_USE_MC_SERVO }, // PWM2 - PA7
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, 1, IOCFG_AF_PP, GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // PWM4 - PB0
    { TIM3,  IO_TAG(PA4),  TIM_Channel_2, 1, IOCFG_AF_PP, GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR }, // PWM1 - PA4
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // PWM5 - PB1
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, 0, IOCFG_AF_PP, GPIO_AF_1, TIM_USE_PPM }, // PWM6 - PPM
    { TIM16, IO_TAG(PB8),  TIM_Channel_1, 0, IOCFG_AF_PP, GPIO_AF_1, TIM_USE_ANY }
};
