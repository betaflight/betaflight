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
 //Target code By BorisB and Hector "Hectech FPV" Hind

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM4, CH2, PB7, TIM_USE_PPM,                 TIMER_INPUT_ENABLED), // PPM IN
    DEF_TIM(TIM16,CH1, PA6, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED), // PWM1
    DEF_TIM(TIM8, CH1N,PA7, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM2
    DEF_TIM(TIM8, CH2, PB8, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED), // PWM3
    DEF_TIM(TIM17,CH1, PB9, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED), // PWM4
    DEF_TIM(TIM1, CH2N,PB0, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM5
    DEF_TIM(TIM8, CH3N,PB1, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM6
    DEF_TIM(TIM2, CH1, PA0, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED), // PWM7
    DEF_TIM(TIM2, CH3, PA2, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED), // PWM8
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_MOTOR | TIM_USE_LED, TIMER_OUTPUT_ENABLED), // LED_STRIP --requires resource remap with dshot (remap to motor 5??)--
};
