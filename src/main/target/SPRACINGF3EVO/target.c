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
    // PPM / UART2 RX
    DEF_TIM(TIM8,  CH1, PA15, TIM_USE_PPM,                       0), // PPM
#if defined(AIORACERF3)
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,                     0), // PWM1
    DEF_TIM(TIM17, CH1, PA7,  TIM_USE_MOTOR,                     0), // PWM2
    DEF_TIM(TIM15, CH1, PA2,  TIM_USE_MOTOR,                     0), // PWM3
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_MOTOR,                     0), // PWM4
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,                     0), // PWM5
    DEF_TIM(TIM3,  CH1, PA6,  TIM_USE_MOTOR,                     0), // PWM6
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_MOTOR,                     0), // PWM7
    DEF_TIM(TIM2,  CH1, PA0,  TIM_USE_MOTOR,                     0), // PWM8
#elif defined(SPRACINGF3MQ_REV) && (SPRACINGF3MQ_REV <= 1)
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,                     0), // PWM1
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,                     0), // PWM2
    DEF_TIM(TIM3,  CH2, PA7,  TIM_USE_MOTOR,                     0), // PWM3
    DEF_TIM(TIM16, CH1, PA6,  TIM_USE_MOTOR,                     0), // PWM4 [TIM16_CH1  (D1_CH3 / D1_CH6)] [TIM3_CH1 (D1_CH6)]
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_MOTOR,                     0), // PWM5 [TIM2_CH2 (D1_CH7)] [TIM15_CH1N (D1_CH5)]
    DEF_TIM(TIM2,  CH3, PA2,  TIM_USE_MOTOR,                     0), // PWM6 [TIM2_CH3 (D1_CH1)] [TIM15_CH1  (D1_CH5)]
    DEF_TIM(TIM2,  CH1, PA0,  TIM_USE_MOTOR,                     0), // PWM7 [TIM2_CH1 (D1_CH5)]
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_MOTOR,                     0), // PWM8 [TIM2_CH4 (D1_CH7)]
#else // SPRACINGF3EVO / SPRACINGF3MQ
    DEF_TIM(TIM2,  CH1, PA0,  TIM_USE_MOTOR,                     0), // PWM1 [TIM2_CH1 (D1_CH5)]
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_MOTOR,                     0), // PWM2 [TIM2_CH2 (D1_CH7)] [TIM15_CH1N (D1_CH5)]
    DEF_TIM(TIM2,  CH3, PA2,  TIM_USE_MOTOR,                     0), // PWM3 [TIM2_CH3 (D1_CH1)] [TIM15_CH1  (D1_CH5)]
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_MOTOR,                     0), // PWM4 [TIM2_CH4 (D1_CH7)]
    DEF_TIM(TIM16, CH1, PA6,  TIM_USE_MOTOR,                     0), // PWM5 [TIM16_CH1  (D1_CH3 / D1_CH6)] [TIM3_CH1 (D1_CH6)]
    DEF_TIM(TIM3,  CH2, PA7,  TIM_USE_MOTOR,                     0), // PWM6
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,                     0), // PWM7
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,                     0), // PWM8
#endif
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_MOTOR,                     0), // RC_CH4 - PB10 - *TIM2_CH3, UART3_TX (AF7)
    DEF_TIM(TIM2,  CH4, PB11, TIM_USE_MOTOR,                     0), // RC_CH3 - PB11 - *TIM2_CH4, UART3_RX (AF7)
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_LED | TIM_USE_TRANSPONDER, 0), // LED_STRIP / TRANSPONDER
};
