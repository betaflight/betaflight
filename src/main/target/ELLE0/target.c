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
    DEF_TIM(TIM2, CH3, PA2, TIM_USE_PWM | TIM_USE_PPM, 0, 0 ), // PPM IN  DMA1_ST1 (shared with RX1)
    DEF_TIM(TIM8, CH1, PC6, TIM_USE_MOTOR,             0, 0 ), // SERVO1  DMA2_ST2
    DEF_TIM(TIM8, CH2, PC7, TIM_USE_MOTOR,             0, 0 ), // SERVO2  DMA2_ST3
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_MOTOR,             0, 1 ), // SERVO3  DMA2_ST4
    DEF_TIM(TIM8, CH4, PC9, TIM_USE_MOTOR,             0, 0 ), // SERVO4  DMA2_ST7
    DEF_TIM(TIM5, CH1, PA0, TIM_USE_MOTOR,             0, 0 ), // SERVO5  DMA1_ST2
    DEF_TIM(TIM5, CH2, PA1, TIM_USE_MOTOR,             0, 0 ), // SERVO6  DMA1_ST4
    DEF_TIM(TIM4, CH3, PB8, TIM_USE_MOTOR,             0, 0 ), // SERVO7  DMA1_ST7
    DEF_TIM(TIM4, CH4, PB9, TIM_USE_MOTOR,             0, 0 ), // SERVO8  DMA1_ST3
};

// Telemetry
//UART1 RX: DMA2_ST5
//UART1 TX: DMA2_ST7

// RX1
//UART2 RX: DMA1_ST5
//UART2 TX: DMA1_ST6

// I2C
//UART3 RX: DMA1_ST1
//UART3 TX: DMA1_ST3
