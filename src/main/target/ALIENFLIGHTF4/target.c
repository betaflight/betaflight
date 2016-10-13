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

#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, IO_TAG(PA8), TIM_Channel_1, TIM1_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM1 },            // PWM1  - PA8  RC1
    { TIM1, IO_TAG(PB0), TIM_Channel_2, TIM1_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM1 },            // PWM2  - PB0  RC2
    { TIM1, IO_TAG(PB1), TIM_Channel_3, TIM1_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM1 },            // PWM3  - PB1  RC3
    { TIM8, IO_TAG(PB14),TIM_Channel_2, TIM8_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM8 },            // PWM4  - PA14 RC4
    { TIM8, IO_TAG(PB15),TIM_Channel_3, TIM8_CC_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM8 },            // PWM5  - PA15 RC5

    { TIM4, IO_TAG(PB8), TIM_Channel_3, TIM4_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM4 },               // PWM6  - PB8  OUT1
    { TIM4, IO_TAG(PB9), TIM_Channel_4, TIM4_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM4 },               // PWM7  - PB9  OUT2
    { TIM5, IO_TAG(PA0), TIM_Channel_1, TIM5_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM5 },               // PWM8  - PA0  OUT3
    { TIM5, IO_TAG(PA1), TIM_Channel_2, TIM5_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM5 },               // PWM9  - PA1  OUT4
    { TIM3, IO_TAG(PC6), TIM_Channel_1, TIM3_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM3 },               // PWM10 - PC6  OUT5
    { TIM3, IO_TAG(PC7), TIM_Channel_2, TIM3_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM3 },               // PWM11 - PC7  OUT6
    { TIM3, IO_TAG(PC8), TIM_Channel_3, TIM3_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM3 },               // PWM13 - PC8  OUT7
    { TIM3, IO_TAG(PC9), TIM_Channel_4, TIM3_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM3 },               // PWM13 - PC9  OUT8
};

