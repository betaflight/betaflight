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
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, 1, IOCFG_AF_PP, GPIO_AF_9, TIM_USE_PWM | TIM_USE_PPM  }, // PWM1 / PPM / UART2 RX
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_9, TIM_USE_PWM }, // PWM2

    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },  // ESC1
    { TIM3,  IO_TAG(PC7),  TIM_Channel_2, 1, IOCFG_AF_PP, GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },  // ESC2
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, 1, IOCFG_AF_PP, GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },  // ESC3
    { TIM3,  IO_TAG(PC6),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },  // ESC4

    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, 1, IOCFG_AF_PP, GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR },  // ESC5 (FW motor)
    { TIM8,  IO_TAG(PC9),  TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_2, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR },  // ESC6 (FW motor)

    { TIM2,  IO_TAG(PB10), TIM_Channel_3, 1, IOCFG_AF_PP, GPIO_AF_1, TIM_USE_PWM },  // PWM3 - PB10 - *TIM2_CH3, UART3_TX (AF7)
    { TIM2,  IO_TAG(PB11), TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_1, TIM_USE_PWM },  // PWM4 - PB11 - *TIM2_CH4, UART3_RX (AF7)

    // with DSHOT DMA1-CH3 conflicts with TIM3_CH4 / ESC1.
    //{ TIM16,  IO_TAG(PB8), TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_1 },    // What's PB8 ???

    { TIM1,   IO_TAG(PA8), TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_6, TIM_USE_LED }
};

