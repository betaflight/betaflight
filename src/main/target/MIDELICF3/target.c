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
#if defined(MIDELICF3V2)
    DEF_TIM(TIM15, CH1, PA2,  TIM_USE_MOTOR,             1), // PWM1 - PA2
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_MOTOR,             1), // PWM2 - PA3
    DEF_TIM(TIM4,  CH3, PA13, TIM_USE_MOTOR,             1), // PWM3 - PA12
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MOTOR,             1), // PWM4 - PA11
    DEF_TIM(TIM16, CH1, PB8,  TIM_USE_MOTOR,             1), // PWM5 - PB8
    DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_MOTOR,             1), // PWM6 - PB9	
    DEF_TIM(TIM3,  CH2, PB5,  TIM_USE_LED,               1), // GPIO_TIMER / LED_STRIP
#else
    DEF_TIM(TIM15, CH1, PA2,  TIM_USE_MOTOR,             1), // PWM1 - PA2
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_MOTOR,             1), // PWM2 - PA3
    DEF_TIM(TIM4,  CH2, PA12, TIM_USE_MOTOR,             1), // PWM3 - PA12
    DEF_TIM(TIM4,  CH1, PA11, TIM_USE_MOTOR,             1), // PWM4 - PA11
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_MOTOR,             1), // PWM5 - PB8
    DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_MOTOR,             1), // PWM6 - PB9	
    DEF_TIM(TIM3,  CH2, PB5,  TIM_USE_LED,               1), // GPIO_TIMER / LED_STRIP
#endif
};
