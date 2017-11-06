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

// Production boards swapped RC_CH3/4 swapped to make it easier to use SerialRX using supplied cables - compared to first prototype.
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM2,  CH1, PA0,  TIM_USE_PWM |TIM_USE_PPM, 0), // RC_CH1 - PA0  - *TIM2_CH1
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_PWM,              0), // RC_CH2 - PA1  - *TIM2_CH2, TIM15_CH1N
    DEF_TIM(TIM2,  CH4, PB11, TIM_USE_PWM,              0), // RC_CH3 - PB11 - *TIM2_CH4, UART3_RX (AF7)
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_PWM,              0), // RC_CH4 - PB10 - *TIM2_CH3, UART3_TX (AF7)
    DEF_TIM(TIM3,  CH1, PB4,  TIM_USE_PWM,              0), // RC_CH5 - PB4  - *TIM3_CH1
    DEF_TIM(TIM3,  CH2, PB5,  TIM_USE_PWM,              0), // RC_CH6 - PB5  - *TIM3_CH2
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_PWM,              0), // RC_CH7 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_PWM,              0), // RC_CH8 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    DEF_TIM(TIM16, CH1, PA6,  TIM_USE_MOTOR,            0), // PWM1 - PA6  - TIM3_CH1, TIM8_BKIN, TIM1_BKIN, *TIM16_CH1
    DEF_TIM(TIM17, CH1, PA7,  TIM_USE_MOTOR,            0), // PWM2 - PA7  - TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    DEF_TIM(TIM4,  CH1, PA11, TIM_USE_MOTOR,            0), // PWM3 - PA11
    DEF_TIM(TIM4,  CH2, PA12, TIM_USE_MOTOR,            0), // PWM4 - PA12
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_MOTOR,            0), // PWM5 - PB8
    DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_MOTOR,            0), // PWM6 - PB9
    DEF_TIM(TIM15, CH1, PA2,  TIM_USE_MOTOR,            0), // PWM7 - PA2
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_MOTOR,            0), // PWM8 - PA3
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_LED,              0), // GPIO_TIMER / LED_STRIP
};
