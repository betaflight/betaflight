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
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM4,  IO_TAG(PB7),  TIM_Channel_2, TIM_USE_PPM, 0, GPIO_AF_2, NULL, 0 }, // RC PPM - PB7  - TIM17_CH1N AF1, TIM4_CH2 AF2, TIM8_BKIN AF5, TIM3_CH4 AF10

    { TIM16, IO_TAG(PA6),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_1, DMA1_Channel6, DMA1_CH6_HANDLER }, // PWM1 - PA6  - TIM3_CH1 AF2, TIM8_BKIN AF4, TIM1_BKIN AF6, *TIM16_CH1 AF1
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_1, DMA1_Channel7, DMA1_CH7_HANDLER }, // PWM2 - PA7  - TIM3_CH2 AF2, TIM8_CH1 AF4, TIM1_CH1N AF6, *TIM17_CH1 AF1
    { TIM8,  IO_TAG(PB8),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_10,DMA2_Channel5, DMA2_CH5_HANDLER }, // PWM3 - PB8 -  TIM16_CH1 AF1, TIM4_CH3 AF2, TIM8_CH2 AF10, TIM1_BKIN AF12
    { TIM8,  IO_TAG(PB9),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_10,DMA2_Channel1, DMA2_CH1_HANDLER }, // PWM4 - PB9 -  TIM17_CH1 AF1, TIM4_CH4 AF2, TIM8_CH3 AF10
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_2, NULL, 0 }, // SOFTSERIAL RX - TIM3_CH3 AF2, TIM8_CH2N AF4, TIM1_CH2N AF6
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_2, NULL, 0 }, // SOFTSERIAL TX - TIM3_CH4 AF2, TIM8_CH3N AF4, TIM1_CH3N AF6
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM6 - PA0 - TIM2_CH1 AF1, TIM8_BKIN AF9, TIM8_ETR AF10
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_9, NULL, 0 }, // PWM7 - PA2 - TIM2_CH3 AF1, TIM15_CH1 AF9

    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM_USE_LED, 1, GPIO_AF_6, DMA1_Channel2, DMA1_CH2_HANDLER }, // GPIO_TIMER / LED_STRIP
};
