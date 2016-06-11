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
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),     // Swap to servo if needed
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1 or servo #1 (swap to servo if needed)
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2 or servo #2 (swap to servo if needed)
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1 or #3
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #4 or #6
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #4
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #5
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #8
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #4
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            0, Mode_AF_PP_PD, GPIO_PinSource8,  GPIO_AF_6}, // PA8 - AF6
    { TIM1,  GPIOA, Pin_9,  TIM_Channel_2, TIM1_CC_IRQn,            0, Mode_AF_PP_PD, GPIO_PinSource9,  GPIO_AF_6}, // PA9 - AF6
    { TIM1,  GPIOA, Pin_10, TIM_Channel_3, TIM1_CC_IRQn,            0, Mode_AF_PP_PD, GPIO_PinSource10, GPIO_AF_6}, // PA10 - AF6
    { TIM3,  GPIOB, Pin_4,  TIM_Channel_1, TIM3_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource4,  GPIO_AF_2}, // PB4 - AF2
    { TIM4,  GPIOB, Pin_6,  TIM_Channel_1, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource6,  GPIO_AF_2}, // PB6 - AF2 - not working yet
    { TIM4,  GPIOB, Pin_7,  TIM_Channel_2, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource7,  GPIO_AF_2}, // PB7 - AF2 - not working yet
    { TIM4,  GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource8,  GPIO_AF_2}, // PB8 - AF2
    { TIM4,  GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource9,  GPIO_AF_2}, // PB9 - AF2

    { TIM2,  GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_2}, // PA0 - untested
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_2}, // PA1 - untested
    { TIM15, GPIOA, Pin_2,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_9}, // PA2 - untested
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_9}, // PA3 - untested
    { TIM16, GPIOA, Pin_6,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_1}, // PA6 - untested
    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_1}  // PA7 - untested
};

