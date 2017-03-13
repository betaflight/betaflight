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

#if defined(USE_DSHOT)
// DSHOT TEST
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, TIM_USE_PWM | TIM_USE_PPM,   0,  GPIO_AF9_TIM12,  NULL,         0,             0  }, // S2_IN
    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM_USE_PWM,   0,  GPIO_AF3_TIM8,  NULL,         0,             0  }, // S3_IN
    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM_USE_PWM,   0,  GPIO_AF3_TIM8,  NULL,         0,             0  }, // S4_IN
    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM_USE_PWM,   0,  GPIO_AF3_TIM8,  NULL,         0,             0  }, // S5_IN
    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM_USE_PWM,   0,  GPIO_AF3_TIM8,  NULL,         0,             0  }, // S6_IN

    { TIM4,  IO_TAG(PB8),  TIM_CHANNEL_3, TIM_USE_MOTOR, 1,  GPIO_AF2_TIM4, DMA1_Stream7, DMA_CHANNEL_2, DMA1_ST7_HANDLER  }, // S10_OUT 1
    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, TIM_USE_MOTOR, 1,  GPIO_AF1_TIM2, DMA1_Stream6, DMA_CHANNEL_3, DMA1_ST6_HANDLER  }, // S1_OUT  4
    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, TIM_USE_MOTOR, 1,  GPIO_AF2_TIM3, DMA1_Stream5, DMA_CHANNEL_5, DMA1_ST5_HANDLER  }, // S4_OUT
    { TIM4,  IO_TAG(PB9),  TIM_CHANNEL_4, TIM_USE_MOTOR, 1,  GPIO_AF2_TIM4,  NULL,         0,             0  }, // S5_OUT  3
    { TIM9,  IO_TAG(PE6),  TIM_CHANNEL_2, TIM_USE_MOTOR, 1,  GPIO_AF3_TIM9,  NULL,         0,             0  }, // S3_OUT
    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, TIM_USE_MOTOR, 1,  GPIO_AF2_TIM3,  NULL,         0,             0  }, // S9_OUT
};
#else
// STANDARD LAYOUT
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, TIM_USE_PWM | TIM_USE_PPM, 0, GPIO_AF9_TIM12, NULL, 0, 0 }, // S2_IN
    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM_USE_PWM, 0, GPIO_AF3_TIM8, NULL, 0, 0 }, // S3_IN
    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM_USE_PWM, 0, GPIO_AF3_TIM8, NULL, 0, 0 }, // S4_IN
    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM_USE_PWM, 0, GPIO_AF3_TIM8, NULL, 0, 0 }, // S5_IN
    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM_USE_PWM, 0, GPIO_AF3_TIM8, NULL, 0, 0 }, // S6_IN

    { TIM4,  IO_TAG(PB8),  TIM_CHANNEL_3, TIM_USE_MOTOR | TIM_USE_LED, 1, GPIO_AF2_TIM4, DMA1_Stream7, DMA_CHANNEL_2, DMA1_ST7_HANDLER  }, // S10_OUT 1
    { TIM4,  IO_TAG(PB9),  TIM_CHANNEL_4, TIM_USE_MOTOR, 1, GPIO_AF2_TIM4, NULL, 0, 0 }, // S5_OUT  3
    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, TIM_USE_MOTOR, 1, GPIO_AF1_TIM2, NULL, 0, 0 }, // S1_OUT  4
    { TIM9,  IO_TAG(PE6),  TIM_CHANNEL_2, TIM_USE_MOTOR, 1, GPIO_AF3_TIM9, NULL, 0, 0 }, // S3_OUT
    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, TIM_USE_MOTOR, 1, GPIO_AF2_TIM3, NULL, 0, 0 }, // S4_OUT
    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, TIM_USE_MOTOR, 1, GPIO_AF2_TIM3, NULL, 0, 0 }, // S9_OUT
};
#endif
