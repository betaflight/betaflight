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
    // PPM Pad
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, TIM_USE_PPM,   0, GPIO_AF_2, NULL, 0 }, // PPM - PB4
    // PB5 / TIM3 CH2 is connected to USBPresent

    { TIM8,  IO_TAG(PB8),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_10, DMA2_Channel5, DMA2_CH5_HANDLER },  // PWM1 - PB8
    { TIM8,  IO_TAG(PB9),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_10, DMA2_Channel1, DMA2_CH1_HANDLER },  // PWM2 - PB9
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_1,  DMA1_Channel7, DMA1_CH7_HANDLER },  // PWM3 - PA3
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_9,  DMA1_Channel5, DMA1_CH5_HANDLER },  // PWM4 - PA2

    // UART3 RX/TX
    //{ TIM2,  IO_TAG(PB10), TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM5  - PB10 - TIM2_CH3 / UART3_TX (AF7)
    //{ TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM6 - PB11 - TIM2_CH4 / UART3_RX (AF7)
    { TIM4,  IO_TAG(PB7),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_2, NULL, 0 },  // PWM7 - PB7
    { TIM4,  IO_TAG(PB6),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_2, NULL, 0 },  // PWM8 - PB6
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM_USE_LED|TIM_USE_TRANSPONDER,   1, GPIO_AF_6, DMA1_Channel2, DMA1_CH2_HANDLER },  // GPIO_TIMER / LED_STRIP
};
