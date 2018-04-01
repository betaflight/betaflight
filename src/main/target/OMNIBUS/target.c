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

    // PPM Pad
    DEF_TIM(TIM3,  CH1, PB4, TIM_USE_PPM,                     0), // PPM - PB4
    // PB5 / TIM3 CH2 is connected to USBPresent
    DEF_TIM(TIM8,  CH2, PB8, TIM_USE_MOTOR,                   0), // PWM1 - PB8
    DEF_TIM(TIM8,  CH3, PB9, TIM_USE_MOTOR,                   0), // PWM2 - PB9
    DEF_TIM(TIM2,  CH4, PA3, TIM_USE_MOTOR,                   0), // PWM3 - PA3
    DEF_TIM(TIM15, CH1, PA2, TIM_USE_MOTOR,                   0), // PWM4 - PA2

    // UART3 RX/TX
    //{ TIM2,  IO_TAG(PB10), TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM5  - PB10 - TIM2_CH3 / UART3_TX (AF7)
    //{ TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM6 - PB11 - TIM2_CH4 / UART3_RX (AF7)
    DEF_TIM(TIM4,  CH2, PB7, TIM_USE_MOTOR,                   0), // PWM7 - PB7
    DEF_TIM(TIM4,  CH1, PB6, TIM_USE_MOTOR,                   0), // PWM8 - PB6
    DEF_TIM(TIM1,  CH1, PA8, TIM_USE_LED|TIM_USE_TRANSPONDER, 0), // GPIO_TIMER / LED_STRIP

};
