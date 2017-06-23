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
    { TIM2, IO_TAG(PA0), TIM_Channel_1, 0, IOCFG_IPD, TIM_USE_PWM | TIM_USE_PPM },   // PWM1 - RC1
    { TIM2, IO_TAG(PA1), TIM_Channel_2, 0, IOCFG_IPD, TIM_USE_PWM },   // PWM2 - RC2
    { TIM2, IO_TAG(PA2), TIM_Channel_3, 0, IOCFG_IPD, TIM_USE_PWM },   // PWM3 - RC3
    { TIM2, IO_TAG(PA3), TIM_Channel_4, 0, IOCFG_IPD, TIM_USE_PWM },   // PWM4 - RC4
    { TIM1, IO_TAG(PA8), TIM_Channel_1, 1, IOCFG_IPD, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR },   // PWM9 - OUT1
    { TIM1, IO_TAG(PA11),TIM_Channel_4, 1, IOCFG_IPD, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR },   // PWM10 - OUT2
    { TIM4, IO_TAG(PB6), TIM_Channel_1, 0, IOCFG_IPD, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },   // PWM11 - OUT3
    { TIM4, IO_TAG(PB7), TIM_Channel_2, 0, IOCFG_IPD, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },   // PWM12 - OUT4
    { TIM4, IO_TAG(PB8), TIM_Channel_3, 0, IOCFG_IPD, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },   // PWM13 - OUT5
    { TIM4, IO_TAG(PB9), TIM_Channel_4, 0, IOCFG_IPD, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },   // PWM14 - OUT6
    { TIM3, IO_TAG(PA6), TIM_Channel_1, 0, IOCFG_IPD, TIM_USE_PWM | TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },   // PWM5 - RC5
    { TIM3, IO_TAG(PA7), TIM_Channel_2, 0, IOCFG_IPD, TIM_USE_PWM | TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },   // PWM6 - RC6
    { TIM3, IO_TAG(PB0), TIM_Channel_3, 0, IOCFG_IPD, TIM_USE_PWM | TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },   // PWM7 - RC7
    { TIM3, IO_TAG(PB1), TIM_Channel_4, 0, IOCFG_IPD, TIM_USE_PWM | TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },   // PWM8 - RC8
};

