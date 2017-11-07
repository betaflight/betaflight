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

// DSHOT will work for motor 1-6.

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {

    DEF_TIM(TIM10, CH1,  PB8, TIM_USE_ANY,                 0, 0), // USE FOR CAMERA CONTROL

    DEF_TIM(TIM2,  CH4,  PA3, TIM_USE_MOTOR,               0, 1), // PWM1  - DMA1_ST6     D(1, 7, 3),D(1, 6, 3)
    DEF_TIM(TIM8,  CH3,  PC8, TIM_USE_MOTOR,               0, 1), // PWM2  - DMA2_ST2     D(2, 4, 7),D(2, 2, 0)
    DEF_TIM(TIM2,  CH3,  PA2, TIM_USE_MOTOR,               0, 0), // PWM3  - DMA1_ST1     D(1, 1, 3)
    DEF_TIM(TIM3,  CH4,  PC9, TIM_USE_MOTOR,               0, 0), // PWM4  - DMA1_ST2     D(1, 2, 5)
    DEF_TIM(TIM1,  CH1,  PA8, TIM_USE_MOTOR,               0, 2), // PWM5  - DMA2_ST3     D(2, 6, 0),D(2, 1, 6),D(2, 3, 6)
    DEF_TIM(TIM4,  CH1,  PB6, TIM_USE_MOTOR,               0, 0), // PWM6  - DMA1_ST0     D(1, 0, 2)

    DEF_TIM(TIM1,  CH3N, PB1, TIM_USE_MOTOR | TIM_USE_LED, 0, 0), // S5_OUT - DMA2_ST6  D(2, 6, 0),D(2, 6, 6)


};
