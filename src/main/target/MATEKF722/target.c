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
    DEF_TIM(TIM9, CH2, PA3, TIM_USE_PPM, 0, 0),      // PPM

    DEF_TIM(TIM3, CH1, PC6, TIM_USE_MOTOR, 0, 0),   // S1   DMA1_ST4
    DEF_TIM(TIM8, CH2, PC7, TIM_USE_MOTOR, 0, 0),   // S2   DMA2_ST3
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_MOTOR, 0, 0),   // S3   DMA2_ST4
    DEF_TIM(TIM8, CH4, PC9, TIM_USE_MOTOR, 0, 0),   // S4   DMA2_ST7

    DEF_TIM(TIM3, CH4, PB1, TIM_USE_MOTOR, 0, 0),   // S5   DMA1_ST2
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_MOTOR, 0, 0),   // S6   DMA2_ST6

    DEF_TIM(TIM4, CH3, PB8, TIM_USE_MOTOR, 0, 0),  // pwm/S7  DMA1_ST7
    DEF_TIM(TIM5, CH3, PA2, TIM_USE_MOTOR, 0, 0),  // TX2/S8  DMA1_ST0

    DEF_TIM(TIM5, CH1, PA0,  TIM_USE_PWM, 0, 0), // TX4
    DEF_TIM(TIM5, CH2, PA1,  TIM_USE_PWM, 0, 0), // RX4

    DEF_TIM(TIM2, CH1, PA15, TIM_USE_LED, 0, 0),  // LED STRIP  DMA1_ST5
};
