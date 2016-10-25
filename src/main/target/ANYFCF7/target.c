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

// DSHOT TEST
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP ,  GPIO_AF9_TIM12,  NULL,         0,             0  }, // S1_IN
    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP ,  GPIO_AF9_TIM12,  NULL,         0,             0  }, // S2_IN
    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8,  NULL,         0,             0  }, // S3_IN
    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8,  NULL,         0,             0  }, // S4_IN
    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8,  NULL,         0,             0  }, // S5_IN
    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8,  NULL,         0,             0  }, // S6_IN

    { TIM4,  IO_TAG(PB8),  TIM_CHANNEL_3, TIM4_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM4, DMA1_Stream7, DMA_CHANNEL_5, DMA1_ST7_HANDLER  }, // S10_OUT 1
    { TIM2,  IO_TAG(PA2),  TIM_CHANNEL_3, TIM2_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF1_TIM2, DMA1_Stream1, DMA_CHANNEL_3, DMA1_ST1_HANDLER  }, // S6_OUT  2
    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, TIM2_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF1_TIM2, DMA1_Stream6, DMA_CHANNEL_3, DMA1_ST6_HANDLER  }, // S1_OUT  4
    { TIM5,  IO_TAG(PA1),  TIM_CHANNEL_2, TIM5_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM5, DMA1_Stream4, DMA_CHANNEL_6, DMA1_ST4_HANDLER  }, // S2_OUT
    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, TIM3_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM3, DMA1_Stream5, DMA_CHANNEL_5, DMA1_ST5_HANDLER  }, // S4_OUT
    { TIM5,  IO_TAG(PA0),  TIM_CHANNEL_1, TIM5_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM5, DMA1_Stream2, DMA_CHANNEL_6, DMA1_ST2_HANDLER  }, // S7_OUT
    { TIM4,  IO_TAG(PB9),  TIM_CHANNEL_4, TIM4_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM4,  NULL,         0,             0  }, // S5_OUT  3
    { TIM9,  IO_TAG(PE6),  TIM_CHANNEL_2, TIM1_BRK_TIM9_IRQn,  1, IOCFG_AF_PP ,  GPIO_AF3_TIM9,  NULL,         0,             0  }, // S3_OUT
    { TIM2,  IO_TAG(PB3),  TIM_CHANNEL_2, TIM2_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF1_TIM2,  NULL,         0,             0  }, // S8_OUT
    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, TIM3_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM3,  NULL,         0,             0  }, // S9_OUT
};

/* STANDARD LAYOUT
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP ,  GPIO_AF9_TIM12}, // S1_IN
    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP ,  GPIO_AF9_TIM12}, // S2_IN
    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8}, // S3_IN
    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8}, // S4_IN
    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8}, // S5_IN
    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8}, // S6_IN

    { TIM4,  IO_TAG(PB8),  TIM_CHANNEL_3, TIM4_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM4}, // S10_OUT 1
    { TIM2,  IO_TAG(PA2),  TIM_CHANNEL_3, TIM2_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF1_TIM2}, // S6_OUT  2
    { TIM4,  IO_TAG(PB9),  TIM_CHANNEL_4, TIM4_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM4}, // S5_OUT  3
    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, TIM2_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF1_TIM2}, // S1_OUT  4
    { TIM5,  IO_TAG(PA1),  TIM_CHANNEL_2, TIM5_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM5}, // S2_OUT
    { TIM9,  IO_TAG(PE6),  TIM_CHANNEL_2, TIM1_BRK_TIM9_IRQn,  1, IOCFG_AF_PP ,  GPIO_AF3_TIM9}, // S3_OUT
    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, TIM3_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM3}, // S4_OUT
    { TIM5,  IO_TAG(PA0),  TIM_CHANNEL_1, TIM5_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM5}, // S7_OUT
    { TIM2,  IO_TAG(PB3),  TIM_CHANNEL_2, TIM2_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF1_TIM2}, // S8_OUT
    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, TIM3_IRQn,           1, IOCFG_AF_PP ,  GPIO_AF2_TIM3}, // S9_OUT
};
*/

// ALTERNATE LAYOUT
//const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
//    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP ,  GPIO_AF9_TIM12}, // S1_IN
//    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP ,  GPIO_AF9_TIM12}, // S2_IN
//    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8}, // S3_IN
//    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8}, // S4_IN
//    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8}, // S5_IN
//    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM8_CC_IRQn,        0, IOCFG_AF_PP ,  GPIO_AF3_TIM8}, // S6_IN
//
//    { TIM10, IO_TAG(PB8),  TIM_CHANNEL_1, TIM1_UP_TIM10_IRQn,      1, IOCFG_AF_PP ,  GPIO_AF3_TIM10}, // S10_OUT
//    { TIM9,  IO_TAG(PA2),  TIM_CHANNEL_1, TIM1_BRK_TIM9_IRQn,      1, IOCFG_AF_PP ,  GPIO_AF3_TIM9}, // S6_OUT
//    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, TIM2_IRQn,               1, IOCFG_AF_PP ,  GPIO_AF1_TIM2}, // S1_OUT
//    { TIM11, IO_TAG(PB9),  TIM_CHANNEL_1, TIM1_TRG_COM_TIM11_IRQn, 1, IOCFG_AF_PP ,  GPIO_AF3_TIM11}, // S5_OUT
//    { TIM5,  IO_TAG(PA1),  TIM_CHANNEL_2, TIM5_IRQn,               1, IOCFG_AF_PP ,  GPIO_AF2_TIM5}, // S2_OUT
//    { TIM9,  IO_TAG(PE6),  TIM_CHANNEL_2, TIM1_BRK_TIM9_IRQn,      1, IOCFG_AF_PP ,  GPIO_AF3_TIM9}, // S3_OUT
//    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, TIM3_IRQn,               1, IOCFG_AF_PP ,  GPIO_AF2_TIM3}, // S4_OUT
//    { TIM5,  IO_TAG(PA0),  TIM_CHANNEL_1, TIM5_IRQn,               1, IOCFG_AF_PP ,  GPIO_AF2_TIM5}, // S7_OUT
//    { TIM2,  IO_TAG(PB3),  TIM_CHANNEL_2, TIM2_IRQn,               1, IOCFG_AF_PP ,  GPIO_AF1_TIM2}, // S8_OUT
//    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, TIM3_IRQn,               1, IOCFG_AF_PP ,  GPIO_AF2_TIM3}, // S9_OUT
//};
