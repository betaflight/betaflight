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
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM2,  CH3,  PB10, TIM_USE_PPM,   0,     0), // T2C3

    DEF_TIM(TIM5,  CH2,  PA1,  TIM_USE_MOTOR, 0, 0), // T2C2(1,6,3), T5C2(1,4,6)
    DEF_TIM(TIM1,  CH1N, PA7,  TIM_USE_MOTOR, 0, 0), // T1C1N(2,3,6), T3C2(1,5,5)
    DEF_TIM(TIM4,  CH3,  PB8,  TIM_USE_MOTOR, 0, 0), // T4C3(1,7,2), T10C1(X)
    DEF_TIM(TIM3,  CH4,  PB1,  TIM_USE_MOTOR, 0, 0), // T1C3N(2,6,6), T3C4(1,7,5)

    DEF_TIM(TIM2,  CH1,  PA0,  TIM_USE_LED,   0, 0), // T2C1(1,5,3)

    // Backdoor timers on UART2 (Too bad that UART1 collides with TIM4)
    DEF_TIM(TIM9,  CH1,  PA2,  TIM_USE_NONE,  0, 0), // UART2_TX
    DEF_TIM(TIM9,  CH2,  PA3,  TIM_USE_NONE,  0, 0), // UART2_RX
};
