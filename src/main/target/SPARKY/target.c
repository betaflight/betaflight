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
    PWM11 | (MAP_TO_PPM_INPUT << 8), // PPM input

    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM15
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM15
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM2
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8), // TIM2
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    // TODO
    0xFFFF
};

const uint16_t airPWM[] = {
    // TODO
    0xFFFF
};


const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // 6 3-pin headers
    { TIM15, IO_TAG(PB15), TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP,    GPIO_AF_1 }, // PWM1  - PB15 - TIM1_CH3N, TIM15_CH1N, *TIM15_CH2
    { TIM15, IO_TAG(PB14), TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP,    GPIO_AF_1 }, // PWM2  - PB14 - TIM1_CH2N, *TIM15_CH1
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            1, IOCFG_AF_PP,    GPIO_AF_6 }, // PWM3  - PA8  - *TIM1_CH1, TIM4_ETR
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_2 }, // PWM4  - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM3,  IO_TAG(PA6),  TIM_Channel_1, TIM3_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_2 }, // PWM5  - PA6  - *TIM3_CH1, TIM8_BKIN, TIM1_BKIN, TIM16_CH1
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM2_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_1 }, // PWM6  - PA2  - *TIM2_CH3, !TIM15_CH1
    // PWM7 - PMW10
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_2 }, // PWM7  - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, IOCFG_AF_PP,    GPIO_AF_1 }, // PWM8  - PA7  - !TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    { TIM3,  IO_TAG(PA4),  TIM_Channel_2, TIM3_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_2 }, // PWM9  - PA4  - *TIM3_CH2
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_1 }, // PWM10 - PA1  - *TIM2_CH2, TIM15_CH1N
    // PPM PORT - Also USART2 RX (AF5)
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, TIM2_IRQn,               0, IOCFG_AF_PP_PD, GPIO_AF_1 } // PPM   - PA3  - TIM2_CH4, TIM15_CH2 - PWM13
};

