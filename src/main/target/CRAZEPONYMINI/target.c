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

const uint16_t multiPPM[] = {
    PWM2 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #1 (M2)
    PWM3 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #2 (M3)
    PWM1 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #3 (M1)
    PWM4 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #4 (M4)
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM2 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #1
    PWM3 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #2
    PWM1 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #3
    PWM4 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #4
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM2 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #1
    PWM3 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #2
    PWM1 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #3
    PWM4 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #4
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM2 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #1
    PWM3 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #2
    PWM1 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #3
    PWM4 | (MAP_TO_MOTOR_OUTPUT << 8), // motor #4
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2, IO_TAG(PA0), TIM_Channel_1, TIM2_IRQn, 0, IOCFG_IPD, },          // PWM1 - OUT1
    { TIM2, IO_TAG(PA1), TIM_Channel_2, TIM2_IRQn, 0, IOCFG_IPD, },          // PWM2 - OUT2
    { TIM2, IO_TAG(PA2), TIM_Channel_3, TIM2_IRQn, 0, IOCFG_IPD, },          // PWM3 - OUT3
    { TIM2, IO_TAG(PA3), TIM_Channel_4, TIM2_IRQn, 0, IOCFG_IPD, },          // PWM4 - OUT4
};

