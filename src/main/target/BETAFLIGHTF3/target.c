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
    DEF_TIM(TIM4, CH2, PB7, TIM_USE_PPM,                 TIMER_INPUT_ENABLED),                          // PPM  DMA(1,4)

    // Motors 1-4
    DEF_TIM(TIM16,CH1, PA6, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM1 DMA(1,6)
    DEF_TIM(TIM8, CH1N,PA7, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED | TIMER_OUTPUT_INVERTED), // PWM2 DMA(2,3)
    DEF_TIM(TIM8, CH2, PB8, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM3 DMA(2,5)
    DEF_TIM(TIM17,CH1, PB9, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM4 DMA(1,7)

    // Motors 5-6 or SoftSerial
    DEF_TIM(TIM3, CH3, PB0, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM5 DMA(1,2) !LED
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM6 DMA(1,3)

    // Motors 7-8 or UART2
    DEF_TIM(TIM2, CH4, PA3, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM7/UART2_RX
    DEF_TIM(TIM2, CH3, PA2, TIM_USE_MOTOR,               TIMER_OUTPUT_ENABLED),                         // PWM8/UART2_TX

    // No LED for Hexa-Dshot; DMA conflict with Motor 5 (PB0); consider PPM if not used.
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_MOTOR | TIM_USE_LED, TIMER_OUTPUT_ENABLED),                         // LED  DMA(1,2)
};
