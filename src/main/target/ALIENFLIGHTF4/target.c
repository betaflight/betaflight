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

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM1, CH1, PA8,  TIM_USE_PWM | TIM_USE_PPM, 0, 0), // PWM1  - PA8  RC1
    DEF_TIM(TIM1, CH2, PB0,  TIM_USE_PWM,               0, 0), // PWM2  - PB0  RC2
    DEF_TIM(TIM1, CH3, PB1,  TIM_USE_PWM,               0, 0), // PWM3  - PB1  RC3
    DEF_TIM(TIM8, CH2, PB14, TIM_USE_PWM,               0, 0), // PWM4  - PA14 RC4
    DEF_TIM(TIM8, CH3, PB15, TIM_USE_PWM,               0, 0), // PWM5  - PA15 RC5
    DEF_TIM(TIM4, CH3, PB8,  TIM_USE_MOTOR,             1, 0), // PWM6  - PB8  OUT1
    DEF_TIM(TIM4, CH4, PB9,  TIM_USE_MOTOR,             1, 0), // PWM7  - PB9  OUT2
    DEF_TIM(TIM5, CH1, PA0,  TIM_USE_MOTOR,             1, 0), // PWM8  - PA0  OUT3
    DEF_TIM(TIM5, CH2, PA1,  TIM_USE_MOTOR,             1, 0), // PWM9  - PA1  OUT4
    DEF_TIM(TIM3, CH1, PC6,  TIM_USE_MOTOR,             1, 0), // PWM10 - PC6  OUT5
    DEF_TIM(TIM3, CH2, PC7,  TIM_USE_MOTOR,             1, 0), // PWM11 - PC7  OUT6
    DEF_TIM(TIM3, CH3, PC8,  TIM_USE_MOTOR,             1, 0), // PWM13 - PC8  OUT7
    DEF_TIM(TIM3, CH4, PC9,  TIM_USE_MOTOR,             1, 0), // PWM13 - PC9  OUT8
};
