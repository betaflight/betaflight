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

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT    << 8),  // PPM input

    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1 Out1
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2 Out 2
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1 Out 3
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #2 Out 4
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #3 Out 5
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #4 Out 6
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #5 Out 7
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #6 Out 8
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #7
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #8
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // PPM / UART2 RX
    { TIM8,  IO_TAG(PA15), TIM_Channel_1, TIM8_CC_IRQn,            0, IOCFG_AF_PP, GPIO_AF_2},  // PPM

    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM2_IRQn,               1, IOCFG_AF_PP, GPIO_AF_1},  // PWM1
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               1, IOCFG_AF_PP, GPIO_AF_1},  // PWM2
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP, GPIO_AF_9},  // PWM3
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP, GPIO_AF_9},  // PWM4
    { TIM16, IO_TAG(PA6),  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, IOCFG_AF_PP, GPIO_AF_1 }, // PWM5 - PA6  - TIM3_CH1, TIM8_BKIN, TIM1_BKIN, *TIM16_CH1
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, IOCFG_AF_PP, GPIO_AF_1 }, // PWM6 - PA7  - TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1  
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2},  // PWM7
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2},  // PWM8

    // UART3 RX/TX
    { TIM2,  IO_TAG(PB10), TIM_Channel_3, TIM2_IRQn,               1, IOCFG_AF_PP, GPIO_AF_1}, // RC_CH4 - PB10 - *TIM2_CH3, USART3_TX (AF7)
    { TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM2_IRQn,               1, IOCFG_AF_PP, GPIO_AF_1}, // RC_CH3 - PB11 - *TIM2_CH4, USART3_RX (AF7)

    // IR / LED Strip Pad
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_6},  // GPIO_TIMER / LED_STRIP
};