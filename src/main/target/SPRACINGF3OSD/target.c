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

    DEF_TIM(TIM3,  CH3, PB0, TIM_USE_MOTOR,       0), // DMA1_Channel2
    DEF_TIM(TIM3,  CH4, PB1, TIM_USE_MOTOR,       0), // DMA1_Channel4
    DEF_TIM(TIM16, CH1, PB4, TIM_USE_MOTOR,       0), // DMA1_Channel3 or DMA1_Channel6 with Remap (need remap to free SPI1_TX for Flash)
    DEF_TIM(TIM17, CH1, PB5, TIM_USE_MOTOR,       0), // DMA1_Channel1 or DMA1_Channel7 with Remap (need remap, ADC1 is on DMA1_Channel1)

    DEF_TIM(TIM1,  CH1, PA8, TIM_USE_TRANSPONDER, 0),
};
