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

#if defined(OMNIBUSF4SD) || defined(EXUAVF4PRO)
    DEF_TIM(TIM10, CH1, PB8,  TIM_USE_PWM | TIM_USE_PPM,   0, 0), // PPM
    DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_PWM,                 0, 0), // S2_IN
#else
    DEF_TIM(TIM12, CH1, PB14, TIM_USE_PWM | TIM_USE_PPM,   0, 0), // PPM
    DEF_TIM(TIM12, CH2, PB15, TIM_USE_PWM,                 0, 0), // S2_IN
#endif

    DEF_TIM(TIM8,  CH1, PC6,  TIM_USE_PWM,                 0, 0), // S3_IN, UART6_TX
    DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_PWM,                 0, 0), // S4_IN, UART6_RX
    DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_PWM,                 0, 0), // S5_IN
    DEF_TIM(TIM8,  CH4, PC9,  TIM_USE_PWM,                 0, 0), // S6_IN

    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,               0, 0), // S1_OUT D1_ST7
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,               0, 0), // S2_OUT D1_ST2
    DEF_TIM(TIM2,  CH4, PA3,  TIM_USE_MOTOR,               0, 1), // S3_OUT D1_ST6
    DEF_TIM(TIM2,  CH3, PA2,  TIM_USE_MOTOR,               0, 0), // S4_OUT D1_ST1

#if defined(OMNIBUSF4SD) || defined(EXUAVF4PRO)
    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_MOTOR,               0, 0), // S5_OUT
    DEF_TIM(TIM4,  CH1, PB6,  TIM_USE_LED,                 0, 0), // LED strip for F4 V2 / F4-Pro-0X and later (RCD_CS for F4)
#else
    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_MOTOR | TIM_USE_LED, 0, 0), // S5_OUT
#endif
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MOTOR,               0, 0), // S6_OUT
    DEF_TIM(TIM1,  CH2, PA9,  TIM_USE_NONE,                0, 0), // UART1_TX
    DEF_TIM(TIM1,  CH3, PA10, TIM_USE_NONE,                0, 0), // UART1_RX
};
