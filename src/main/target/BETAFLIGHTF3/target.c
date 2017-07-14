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
    DEF_TIM(TIM4, CH2, PB7, TIM_USE_PPM,                 TIMER_INPUT_ENABLED),                          // PPM
    DEF_TIM(TIM16,CH1, PA6, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM1 (1,4)
    DEF_TIM(TIM8, CH1N,PA7, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM2 (2,3)
    DEF_TIM(TIM8, CH2, PB8, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM3 (2,5)
    DEF_TIM(TIM17,CH1, PB9, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM4 (1,1)

#ifdef BFF3_USE_HEXA_DSHOT
    // For HEXA dshot
    DEF_TIM(TIM1, CH2N,PB0, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM5 (1,3)
    DEF_TIM(TIM8, CH3N,PB1, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM6 (2,1)
#else
    // For softserial
    DEF_TIM(TIM3, CH3, PB0, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM5 (1,2) !LED
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM6 (1,3)
#endif

    DEF_TIM(TIM2, CH4, PA3, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM7/UART2_RX
    DEF_TIM(TIM2, CH3, PA2, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM8/UART2_TX

    // When using softserial config, LED will have DMA conflict with PB0 (SOFTSERIAL1_RX).
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_MOTOR | TIM_USE_LED, TIMER_OUTPUT_ENABLED),                         // LED  (1,2)
};
