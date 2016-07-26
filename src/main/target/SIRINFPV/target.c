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
    PWM7  | (MAP_TO_PPM_INPUT << 8),
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM7  | (MAP_TO_PPM_INPUT << 8),
    PWM1  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {

    { TIM4,  IO_TAG(PB6), TIM_Channel_1, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2 }, // PWM1 - PB6
    { TIM4,  IO_TAG(PB7), TIM_Channel_2, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2 }, // PWM2 - PB6
    { TIM4,  IO_TAG(PB8), TIM_Channel_3, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2 },  // PWM3 - PB8
    { TIM4,  IO_TAG(PB9), TIM_Channel_4, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2 },  // PWM4 - PB9

    { TIM3,  IO_TAG(PB0), TIM_Channel_3, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2 }, // PWM5 - PB0  - *TIM3_CH3
    { TIM3,  IO_TAG(PB1), TIM_Channel_4, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2 }, // PWM6 - PB1  - *TIM3_CH4

    { TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM2_IRQn,              0, IOCFG_AF_PP, GPIO_AF_1 }, // RC_CH3 - PB11 - *TIM2_CH4, USART3_RX (AF7)y
};


