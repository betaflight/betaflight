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
#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    //PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    //PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM5, IO_TAG(PA3),    TIM_Channel_4, TIM5_IRQn,       0, IOCFG_AF_PP_PD, GPIO_AF_TIM5 },

    { TIM3, IO_TAG(PC6),    TIM_Channel_1, TIM3_IRQn,       0, IOCFG_AF_PP_PD, GPIO_AF_TIM3 },
    { TIM8, IO_TAG(PC7),    TIM_Channel_2, TIM8_CC_IRQn,    0, IOCFG_AF_PP_PD, GPIO_AF_TIM8 },
    { TIM8, IO_TAG(PC8),    TIM_Channel_3, TIM8_CC_IRQn,    0, IOCFG_AF_PP_PD, GPIO_AF_TIM8 },
    { TIM8, IO_TAG(PC9),    TIM_Channel_4, TIM8_CC_IRQn,    0, IOCFG_AF_PP_PD, GPIO_AF_TIM8 },
    
    { TIM2, IO_TAG(PA15),   TIM_Channel_1, TIM2_IRQn,       0, IOCFG_AF_PP_PD, GPIO_AF_TIM2 },
    { TIM1, IO_TAG(PA8),    TIM_Channel_1, TIM1_CC_IRQn,    0, IOCFG_AF_PP_PD, GPIO_AF_TIM1 },

    { TIM4, IO_TAG(PB6),    TIM_Channel_1, TIM4_IRQn,       0, IOCFG_AF_PP_PD, GPIO_AF_TIM1 }
};

