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

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {

    { TIM8, IO_TAG(PC9), TIM_Channel_4, TIM8_CC_IRQn,       0, IOCFG_AF_PP, GPIO_AF_TIM8, NULL,         0,             0  },          // PPM_IN

    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, TIM2_IRQn,        1, IOCFG_AF_PP, GPIO_AF_TIM2,  DMA1_Stream6, DMA_Channel_3, DMA1_ST6_HANDLER  },  // S1_OUT
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,        1, IOCFG_AF_PP, GPIO_AF_TIM3,  DMA1_Stream7, DMA_Channel_5, DMA1_ST7_HANDLER  },  // S2_OUT
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,        1, IOCFG_AF_PP, GPIO_AF_TIM3,  DMA1_Stream2, DMA_Channel_5, DMA1_ST2_HANDLER  },  // S3_OUT
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM2_IRQn,        1, IOCFG_AF_PP, GPIO_AF_TIM2,  DMA1_Stream1, DMA_Channel_3, DMA1_ST1_HANDLER  },  // S4_OUT

//  { TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5 },    // LED Strip
};

