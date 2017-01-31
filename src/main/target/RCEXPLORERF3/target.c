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
    PWM6  | (MAP_TO_PPM_INPUT << 8),    // PPM input
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17 - can be switched to servo
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    0xFFFF
};

const uint16_t multiPWM[] = {
        PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
        PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17 - can be switched to servo
        PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
        PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
        PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),    // PPM input
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8), // TIM3
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8), // TIM3
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8), // TIM3
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8), // TIM3
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8), // TIM3
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8), // TIM3
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3,  IO_TAG(PA4),  TIM_Channel_2, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2}, // PWM1 - PA4
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, IOCFG_AF_PP, GPIO_AF_1}, // PWM2 - PA7
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_6}, // PWM3 - PA8
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2}, // PWM4 - PB0
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2}, // PWM5 - PB1
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               0, IOCFG_AF_PP, GPIO_AF_1}, // PWM6 - PPM
};
