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
    { TIM9,  IO_TAG(PA3),   TIM_Channel_2, TIM_USE_PPM | TIM_USE_PWM,   0, GPIO_AF_TIM9, NULL, 0, 0 }, // PPM / PWM1 / UART2 RX
    { TIM9,  IO_TAG(PA2),   TIM_Channel_1, TIM_USE_PWM,                 0, GPIO_AF_TIM9, NULL, 0, 0 }, // PPM / PWM2 / UART2 TX

    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, TIM_USE_MOTOR,                1, GPIO_AF_TIM8,  DMA2_Stream4, DMA_Channel_7, DMA2_ST4_HANDLER }, // ESC 1
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, TIM_USE_MOTOR,                1, GPIO_AF_TIM8,  DMA2_Stream3, DMA_Channel_7, DMA2_ST3_HANDLER }, // ESC 2
    { TIM8,  IO_TAG(PC9),  TIM_Channel_4, TIM_USE_MOTOR,                1, GPIO_AF_TIM8,  DMA2_Stream7, DMA_Channel_7, DMA2_ST7_HANDLER }, // ESC 3
    { TIM8,  IO_TAG(PC6),  TIM_Channel_1, TIM_USE_MOTOR,                1, GPIO_AF_TIM8,  DMA2_Stream2, DMA_Channel_7, DMA2_ST2_HANDLER }, // ESC 4

#if (SPRACINGF4NEO_REV >= 3)
    { TIM4,  IO_TAG(PB6),  TIM_Channel_1, TIM_USE_MOTOR,                1, GPIO_AF_TIM4, DMA1_Stream0, DMA_Channel_2, DMA1_ST0_HANDLER }, // ESC 5 / Conflicts with USART5_RX / SPI3_RX - SPI3_RX can be mapped to DMA1_ST3_CH0
    { TIM4,  IO_TAG(PB7),  TIM_Channel_2, TIM_USE_MOTOR,                1, GPIO_AF_TIM4, DMA1_Stream3, DMA_Channel_2, DMA1_ST3_HANDLER }, // ESC 6 / Conflicts with USART3_RX
#else
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM_USE_MOTOR,                1, GPIO_AF_TIM3, DMA1_Stream7, DMA_Channel_5, DMA1_ST7_HANDLER }, // ESC 5
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM_USE_MOTOR,                1, GPIO_AF_TIM3, DMA1_Stream2, DMA_Channel_5, DMA1_ST2_HANDLER }, // ESC 6
#endif
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM_USE_LED,                  1, GPIO_AF_TIM2, DMA1_Stream6, DMA_Channel_3, DMA1_ST6_HANDLER }, // LED Strip
    // Additional 2 PWM channels available on UART3 RX/TX pins
    // However, when using led strip the timer cannot be used, but no code appears to prevent that right now
    { TIM2,  IO_TAG(PB10), TIM_Channel_3, TIM_USE_MOTOR,                1, GPIO_AF_TIM2, DMA1_Stream1, DMA_Channel_3, DMA1_ST1_HANDLER }, // ESC 7 / Shared with UART3 TX PIN and SPI3 TX (OSD)
    { TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM_USE_MOTOR,                1, GPIO_AF_TIM2, DMA1_Stream6, DMA_Channel_3, DMA1_ST6_HANDLER }, // ESC 8 / Shared with UART3 RX PIN

    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM_USE_TRANSPONDER,          1, GPIO_AF_TIM1, DMA2_Stream6, DMA_Channel_0, DMA2_ST6_HANDLER }, // Transponder
    // Additional 2 PWM channels available on UART1 RX/TX pins
    // However, when using transponder the timer cannot be used, but no code appears to prevent that right now
    { TIM1,  IO_TAG(PA9),  TIM_Channel_2, TIM_USE_SERVO | TIM_USE_PWM,  1, GPIO_AF_TIM1, NULL, 0, 0 }, // PWM 3
    { TIM1,  IO_TAG(PA10), TIM_Channel_3, TIM_USE_SERVO | TIM_USE_PWM,  1, GPIO_AF_TIM1, NULL, 0, 0 }, // PWM 4
};
