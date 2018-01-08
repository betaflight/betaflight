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
#if defined(FF_PIKOF4OSD)
    DEF_TIM(TIM5, CH4, PA3, TIM_USE_MOTOR, 0, 0 ), // S1_OUT - DMA1_ST7
    DEF_TIM(TIM3, CH3, PB1, TIM_USE_MOTOR, 0, 0 ), // S2_OUT - DMA1_ST1
    DEF_TIM(TIM5, CH3, PA2, TIM_USE_MOTOR, 0, 0 ), // S3_OUT - DMA1_ST6
    DEF_TIM(TIM3, CH4, PB0, TIM_USE_MOTOR, 0, 0 ), // S4_OUT - DMA1_ST2
    DEF_TIM(TIM8, CH2N, PB14, TIM_USE_MOTOR, 0, 0 ), // PA14 RC4  - DMA2_ST6, *DMA2_ST2
    DEF_TIM(TIM1, CH3N, PB15, TIM_USE_MOTOR, 0, 0 ), // PA15 RC5  - DMA2_ST6, DMA2_ST6
#else
    DEF_TIM(TIM2, CH4, PA3, TIM_USE_MOTOR, 0, 1 ), // S1_OUT - DMA1_ST6
    DEF_TIM(TIM3, CH3, PB0, TIM_USE_MOTOR, 0, 0 ), // S2_OUT - DMA1_ST7
    DEF_TIM(TIM2, CH3, PA2, TIM_USE_MOTOR, 0, 0 ), // S3_OUT - DMA1_ST1
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_MOTOR, 0, 0 ), // S4_OUT - DMA1_ST2
#endif
    DEF_TIM(TIM4, CH2, PB7, TIM_USE_LED,   0, 0 ), // LED    - DMA1_ST3
};
