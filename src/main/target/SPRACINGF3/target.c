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

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM2_IRQn,               0, IOCFG_AF_PP, GPIO_AF_1 }, // RC_CH1 - PA0  - *TIM2_CH1
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               0, IOCFG_AF_PP, GPIO_AF_1 }, // RC_CH2 - PA1  - *TIM2_CH2, TIM15_CH1N
    // Production boards swapped RC_CH3/4 swapped to make it easier to use SerialRX using supplied cables - compared to first prototype.
    { TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM2_IRQn,               0, IOCFG_AF_PP, GPIO_AF_1 }, // RC_CH3 - PB11 - *TIM2_CH4, UART3_RX (AF7)
    { TIM2,  IO_TAG(PB10), TIM_Channel_3, TIM2_IRQn,               0, IOCFG_AF_PP, GPIO_AF_1 }, // RC_CH4 - PB10 - *TIM2_CH3, UART3_TX (AF7)
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, TIM3_IRQn,               0, IOCFG_AF_PP, GPIO_AF_2 }, // RC_CH5 - PB4  - *TIM3_CH1
    { TIM3,  IO_TAG(PB5),  TIM_Channel_2, TIM3_IRQn,               0, IOCFG_AF_PP, GPIO_AF_2 }, // RC_CH6 - PB5  - *TIM3_CH2
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,               0, IOCFG_AF_PP, GPIO_AF_2 }, // RC_CH7 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               0, IOCFG_AF_PP, GPIO_AF_2 }, // RC_CH8 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N

    { TIM16, IO_TAG(PA6),  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, IOCFG_AF_PP, GPIO_AF_1 }, // PWM1 - PA6  - TIM3_CH1, TIM8_BKIN, TIM1_BKIN, *TIM16_CH1
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, IOCFG_AF_PP, GPIO_AF_1 }, // PWM2 - PA7  - TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    { TIM4,  IO_TAG(PA11), TIM_Channel_1, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_10 },// PWM3 - PA11
    { TIM4,  IO_TAG(PA12), TIM_Channel_2, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_10 },// PWM4 - PA12
    { TIM4,  IO_TAG(PB8),  TIM_Channel_3, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2 }, // PWM5 - PB8
    { TIM4,  IO_TAG(PB9),  TIM_Channel_4, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2 }, // PWM6 - PB9
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP, GPIO_AF_9 }, // PWM7 - PA2
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP, GPIO_AF_9 }, // PWM8 - PA3

    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            1, IOCFG_AF_PP, GPIO_AF_6 }, // GPIO_TIMER / LED_STRIP
};

