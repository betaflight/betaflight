/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"

#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM9, CH2, PA3,  TIM_USE_PPM,   0, 0), // PPM/RX2

    DEF_TIM(TIM2, CH3, PB10, TIM_USE_MOTOR, 0, 0), // S1_OUT - DMA1_ST1
    DEF_TIM(TIM4, CH1, PB6,  TIM_USE_MOTOR, 0, 0), // S2_OUT - DMA1_ST0
    DEF_TIM(TIM4, CH2, PB7,  TIM_USE_MOTOR, 0, 0), // S3_OUT - DMA1_ST3
    DEF_TIM(TIM4, CH3, PB8,  TIM_USE_MOTOR, 0, 0), // S4_OUT - DMA1_ST7

    DEF_TIM(TIM5, CH1, PA0,  TIM_USE_LED,   0, 0), // 2812LED - DMA1_ST2

    DEF_TIM(TIM9, CH1, PA2,  TIM_USE_PWM,   0, 0 ), // TX2
    DEF_TIM(TIM1, CH2, PA9,  TIM_USE_PWM,   0, 0 ), // TX1
    DEF_TIM(TIM1, CH3, PA10, TIM_USE_PWM,   0, 0 ), // RX1
};
