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
    { TIM16, IO_TAG(PB8),  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      0, IOCFG_AF_PP_PD, GPIO_AF_1, NULL, 0 }, 
    { TIM17, IO_TAG(PB9),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 0, IOCFG_AF_PP_PD, GPIO_AF_1, NULL, 0 }, 
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            1, IOCFG_AF_PP_PD, GPIO_AF_6, DMA1_Channel2, DMA1_CH2_HANDLER }, 
    { TIM8,  IO_TAG(PC6),  TIM_Channel_1, TIM8_CC_IRQn,            1, IOCFG_AF_PP_PD, GPIO_AF_4, DMA2_Channel3, DMA2_CH3_HANDLER }, 
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, TIM8_CC_IRQn,            1, IOCFG_AF_PP_PD, GPIO_AF_4, DMA2_Channel5, DMA2_CH5_HANDLER }, 
    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, TIM8_CC_IRQn,            1, IOCFG_AF_PP_PD, GPIO_AF_4, DMA2_Channel1, DMA2_CH1_HANDLER }, 
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               0, IOCFG_AF_PP_PD, GPIO_AF_2, NULL, 0 }, 
    { TIM3,  IO_TAG(PA4),  TIM_Channel_2, TIM3_IRQn,               0, IOCFG_AF_PP_PD, GPIO_AF_2, NULL, 0 }, 
    { TIM4,  IO_TAG(PD12), TIM_Channel_1, TIM4_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_2, NULL, 0 }, 
    { TIM4,  IO_TAG(PD13), TIM_Channel_2, TIM4_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_2, NULL, 0 }, 
    { TIM4,  IO_TAG(PD14), TIM_Channel_3, TIM4_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_2, NULL, 0 }, 
    { TIM4,  IO_TAG(PD15), TIM_Channel_4, TIM4_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_2, NULL, 0 }, 
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_1, NULL, 0 }, 
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM2_IRQn,               0, IOCFG_AF_PP,    GPIO_AF_1, NULL, 0 }  
};

