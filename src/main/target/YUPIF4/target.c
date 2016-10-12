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
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS1
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS2
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS3
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS4
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS5
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS6
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS1
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS2
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS3
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS4
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS5
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),  // MS6
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),  // Motor 1
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),  // Motor 2
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),  // servo 1
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),  // servo 2
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),  // servo 3
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),  // servo 4
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),  // Motor 1
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),  // Motor 2
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),  // servo 1
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),  // servo 2
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),  // servo 3
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),  // servo 4
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM8, IO_TAG(PC8), TIM_Channel_3, TIM8_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM8 }, // PPM IN
    { TIM2, IO_TAG(PA0), TIM_Channel_1, TIM2_IRQn,     1, IOCFG_AF_PP, GPIO_AF_TIM2 }, // MS1
    { TIM2, IO_TAG(PA1), TIM_Channel_2, TIM2_IRQn,     1, IOCFG_AF_PP, GPIO_AF_TIM2 }, // MS2
    { TIM2, IO_TAG(PA2), TIM_Channel_3, TIM2_IRQn,     1, IOCFG_AF_PP, GPIO_AF_TIM2 }, // MS3
    { TIM2, IO_TAG(PA3), TIM_Channel_4, TIM2_IRQn,     1, IOCFG_AF_PP, GPIO_AF_TIM2 }, // MS4
    { TIM3, IO_TAG(PB0), TIM_Channel_3, TIM3_IRQn,     1, IOCFG_AF_PP, GPIO_AF_TIM3 }, // MS5
    { TIM3, IO_TAG(PB1), TIM_Channel_4, TIM3_IRQn,     1, IOCFG_AF_PP, GPIO_AF_TIM3 }, // MS6
 };
