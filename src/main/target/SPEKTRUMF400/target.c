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
    DEF_TIM(TIM3, CH4,  PB1,  TIM_USE_MOTOR, 0, 0), // pin 27 - M1 - DMA1_ST2
    DEF_TIM(TIM3, CH3,  PB0,  TIM_USE_MOTOR, 0, 0), // pin 26 - M2 - DMA1_ST7
    DEF_TIM(TIM3, CH2,  PB5,  TIM_USE_MOTOR, 0, 0), // pin 57 - M3 - DMA1_ST5
    DEF_TIM(TIM3, CH1,  PB4,  TIM_USE_MOTOR, 0, 0), // pin 56 - M4 - DMA1_ST4
    DEF_TIM(TIM8, CH1,  PC6,  TIM_USE_LED,   0, 0), // LED_STRIP - DMA2_ST2

    // Backdoor timers
    DEF_TIM(TIM1, CH2,  PA9,  TIM_USE_NONE , 0, 0), // UART1_TX T1C2
    DEF_TIM(TIM1, CH3,  PA10, TIM_USE_NONE , 0, 0), // UART1_RX T1C3

    DEF_TIM(TIM2, CH3,  PA2,  TIM_USE_NONE , 0, 0), // UART2_TX T5C3,T9C1,T2C3
    DEF_TIM(TIM9, CH2,  PA3,  TIM_USE_NONE , 0, 0), // UART2_RX T5C4,T9C2,T2C4

    DEF_TIM(TIM5, CH1,  PA0,  TIM_USE_NONE , 0, 0), // UART4_TX T5C1
    DEF_TIM(TIM5, CH2,  PA1,  TIM_USE_NONE , 0, 0), // UART4_RX T5C2,T2C2
};
