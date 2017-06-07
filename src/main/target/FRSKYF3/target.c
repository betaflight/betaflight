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
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {

	{ TIM8,  IO_TAG(PB9),  TIM_Channel_3, TIM_USE_MOTOR,	 1, GPIO_AF_10,   DMA2_Channel1, DMA2_CH1_HANDLER}, // PPM1
    
	{ TIM8,  IO_TAG(PB8),  TIM_Channel_2, TIM_USE_MOTOR,   1, GPIO_AF_10, DMA2_Channel5, DMA2_CH5_HANDLER}, // PPM2
	
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_2,  DMA1_Channel3, DMA1_CH3_HANDLER },  // PWM3

    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_1, DMA1_Channel7, DMA1_CH7_HANDLER},  // PWM4

    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 },  // PWM5

    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 },  // PWM6
    
	{ TIM2, IO_TAG(PA0),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_1,  DMA1_Channel5,DMA1_CH5_HANDLER  },  // PWM7
	{ TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_2, DMA1_Channel2, DMA1_CH2_HANDLER },  // PWM8


	{ TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM_USE_LED|TIM_USE_TRANSPONDER,	 1, GPIO_AF_6, NULL, 0 },  // LED_STRIP

};



