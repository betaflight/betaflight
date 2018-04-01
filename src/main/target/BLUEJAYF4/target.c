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

/*
 * - Support HEXA-Dshot
 * - S5_OUT, S6_OUT are N-channels, which are only capable of outputs. (E.g., motors, servos, LED strip, PWM outputs.)
 * - Input is only available on DEBUG pad. See comment on SOFTSERIAL1 in target.h for details.
 */
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM3, CH2,  PC7, TIM_USE_PPM,                 0, 0 ), // PPM IN
    DEF_TIM(TIM5, CH1,  PA0, TIM_USE_MOTOR,               0, 0 ), // S1_OUT - DMA1_ST2
    DEF_TIM(TIM5, CH2,  PA1, TIM_USE_MOTOR,               0, 0 ), // S2_OUT - DMA1_ST4
    DEF_TIM(TIM5, CH3,  PA2, TIM_USE_MOTOR,               0, 0 ), // S3_OUT - DMA1_ST0
    DEF_TIM(TIM5, CH4,  PA3, TIM_USE_MOTOR,               0, 1 ), // S4_OUT - DMA1_ST3 (Could be DMA1_ST1 with dmaopt=0)
    DEF_TIM(TIM1, CH2N, PB0, TIM_USE_MOTOR | TIM_USE_LED, 0, 0 ), // S5_OUT - DMA2_ST6
    DEF_TIM(TIM8, CH3N, PB1, TIM_USE_MOTOR,               0, 0 ), // S6_OUT - DMA2_ST2
    DEF_TIM(TIM2, CH2,  PB3, TIM_USE_NONE,                0, 0 ), // DEBUG  - DMA1_ST6
};
