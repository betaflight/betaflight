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

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8), // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_PWM_INPUT << 8),
    PWM10 | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
        0xFFFF
};

const uint16_t airPWM[] = {
        0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2, IO_TAG(PA0), TIM_Channel_1, TIM2_IRQn, 0, IOCFG_IPD },          // PWM1 - RC1
    { TIM2, IO_TAG(PA1), TIM_Channel_2, TIM2_IRQn, 0, IOCFG_IPD },          // PWM2 - RC2
    { TIM2, IO_TAG(PA2), TIM_Channel_3, TIM2_IRQn, 0, IOCFG_IPD },          // PWM3 - RC3
    { TIM2, IO_TAG(PA3), TIM_Channel_4, TIM2_IRQn, 0, IOCFG_IPD },          // PWM4 - RC4
    { TIM3, IO_TAG(PA6), TIM_Channel_1, TIM3_IRQn, 0, IOCFG_IPD },          // PWM5 - RC5
    { TIM3, IO_TAG(PA7), TIM_Channel_2, TIM3_IRQn, 0, IOCFG_IPD },          // PWM6 - RC6
    { TIM3, IO_TAG(PB0), TIM_Channel_3, TIM3_IRQn, 0, IOCFG_IPD },          // PWM7 - RC7
    { TIM3, IO_TAG(PB1), TIM_Channel_4, TIM3_IRQn, 0, IOCFG_IPD },          // PWM8 - RC8
    { TIM1, IO_TAG(PA8), TIM_Channel_1, TIM1_CC_IRQn, 1, IOCFG_IPD },       // PWM9 - OUT1
    { TIM1, IO_TAG(PA11),TIM_Channel_4, TIM1_CC_IRQn, 1, IOCFG_IPD },       // PWM10 - OUT2
    { TIM4, IO_TAG(PB6), TIM_Channel_1, TIM4_IRQn, 0, IOCFG_IPD },          // PWM11 - OUT3
    { TIM4, IO_TAG(PB7), TIM_Channel_2, TIM4_IRQn, 0, IOCFG_IPD },          // PWM12 - OUT4
    { TIM4, IO_TAG(PB8), TIM_Channel_3, TIM4_IRQn, 0, IOCFG_IPD },          // PWM13 - OUT5
    { TIM4, IO_TAG(PB9), TIM_Channel_4, TIM4_IRQn, 0, IOCFG_IPD }           // PWM14 - OUT6
};

