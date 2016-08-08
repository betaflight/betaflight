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
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    // prevent crashing, but do nothing
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    // prevent crashing, but do nothing
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn, 0, IOCFG_AF_PP_PD,   GPIO_AF_6 }, // PWM1 - PA8

    { TIM4,  IO_TAG(PB8),  TIM_Channel_3, TIM4_IRQn,    1, IOCFG_AF_PP,      GPIO_AF_2 }, // PWM2 - PB8
    { TIM4,  IO_TAG(PB9),  TIM_Channel_4, TIM4_IRQn,    1, IOCFG_AF_PP,      GPIO_AF_2 }, // PWM3 - PB9

    { TIM2,  IO_TAG(PA10), TIM_Channel_4, TIM2_IRQn,    1, IOCFG_AF_PP,      GPIO_AF_10 }, // PMW4 - PA10
    { TIM2,  IO_TAG(PA9),  TIM_Channel_3, TIM2_IRQn,    1, IOCFG_AF_PP,      GPIO_AF_10 }, // PWM5 - PA9
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM2_IRQn,    1, IOCFG_AF_PP,      GPIO_AF_1 },  // PWM6 - PA0
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,    1, IOCFG_AF_PP,      GPIO_AF_1 },  // PWM7 - PA1

    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,    1, IOCFG_AF_PP_PD,   GPIO_AF_2 },  // PWM8 - PB1
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,    1, IOCFG_AF_PP_PD,   GPIO_AF_2 },  // PWM9 - PB0
};

