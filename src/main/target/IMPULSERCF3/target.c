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
#include "drivers/timer_def.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM2, CH1,PA15, TIM_USE_PPM,                 TIMER_INPUT_ENABLED), // PPM IN
    DEF_TIM(TIM16,CH1, PB4, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED), // PWM1
    DEF_TIM(TIM17,CH1, PB5, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED), // PWM2
    DEF_TIM(TIM8,CH3N, PB1, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM3
    DEF_TIM(TIM8,CH2N, PB0, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM4
    DEF_TIM(TIM16,CH1, PB8, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED), // PWM5
    DEF_TIM(TIM17,CH1, PB9, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED), // PWM6
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_LED,                 TIMER_OUTPUT_ENABLED), // LED_STRIP
};
