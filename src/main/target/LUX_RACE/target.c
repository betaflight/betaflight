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
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8), // Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8), // Swap to servo if needed
    0xFFFF
};

const uint16_t multiPWM[] = {
    // prevent crashing, but do nothing
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8), // Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8), // Swap to servo if needed
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8), // Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8), // Swap to servo if needed
    0xFFFF
};

const uint16_t airPWM[] = {
    // prevent crashing, but do nothing
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8), // Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8), // Swap to servo if needed
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            0, IOCFG_AF_PP_PD, GPIO_AF_6 }, // PWM1 - PA8
    { TIM3,  IO_TAG(PC6),  TIM_Channel_1, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2 }, // PWM2 - PC6
    { TIM3,  IO_TAG(PC7),  TIM_Channel_2, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2 }, // PWM3 - PC7
    { TIM3,  IO_TAG(PC8),  TIM_Channel_3, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2 }, // PMW4 - PC8
    { TIM3,  IO_TAG(PC9),  TIM_Channel_4, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2 }, // PWM5 - PC9
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1 }, // PWM6 - PA0
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1 }, // PWM7 - PA1
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1 }, // PWM8 - PA2
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1 }, // PWM9 - PA3
    { TIM15, IO_TAG(PB14), TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP_PD, GPIO_AF_1 }, // PWM10 - PB14
    { TIM15, IO_TAG(PB15), TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP_PD, GPIO_AF_1 }, // PWM11 - PB15
};

