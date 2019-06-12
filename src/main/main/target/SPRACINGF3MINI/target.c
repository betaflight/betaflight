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
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
#ifdef TINYBEEF3
    // PPM / UART2 RX
    DEF_TIM(TIM8,  CH1, PA15, TIM_USE_PPM,                       0), // PPM
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_MOTOR,                     0), // PWM1
    DEF_TIM(TIM2,  CH1, PA0,  TIM_USE_MOTOR,                     0), // PWM2
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,                     0), // PWM3
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,                     0), // PWM4
    DEF_TIM(TIM2,  CH3, PA2,  TIM_USE_MOTOR,                     0), // PWM5
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_MOTOR,                     0), // PWM6
    DEF_TIM(TIM3,  CH1, PA6,  TIM_USE_MOTOR,                     0), // PWM7
    DEF_TIM(TIM3,  CH2, PA7,  TIM_USE_MOTOR,                     0), // PWM8
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_MOTOR,                     0), // RC_CH4 - PB10 - *TIM2_CH3, UART3_TX (AF7)
    DEF_TIM(TIM2,  CH4, PB11, TIM_USE_MOTOR,                     0), // RC_CH3 - PB11 - *TIM2_CH4, UART3_RX (AF7)
#else
    // PPM Pad
#if defined(SPRACINGF3MINI_REV) && (SPRACINGF3MINI_REV <= 1)
    DEF_TIM(TIM3,  CH2, PB5,  TIM_USE_PPM,                       0), // PPM - PB5
    // PB4 / TIM3 CH1 is connected to USBPresent
#else
    DEF_TIM(TIM3,  CH1, PB4,  TIM_USE_PPM,                       0), // PPM - PB4
    // PB5 / TIM3 CH2 is connected to USBPresent
#endif

    DEF_TIM(TIM16, CH1, PA6,  TIM_USE_MOTOR,                     0), // PWM1 - PA6
    DEF_TIM(TIM17, CH1, PA7,  TIM_USE_MOTOR,                     0), // PWM2 - PA7
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_MOTOR,                     0), // PWM3 - PB8
    DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_MOTOR,                     0), // PWM4 - PB9
    DEF_TIM(TIM15, CH1, PA2,  TIM_USE_MOTOR,                     0), // PWM5 - PA2
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_MOTOR,                     0), // PWM6 - PA3
    DEF_TIM(TIM2,  CH1, PA0,  TIM_USE_MOTOR,                     0), // PWM7 - PA0
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_MOTOR,                     0), // PWM8 - PA1
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_NONE,                      0), // PWM9  - PB10 - TIM2_CH3 / UART3_TX (AF7)
    DEF_TIM(TIM2,  CH4, PB11, TIM_USE_NONE,                      0), // PWM10 - PB11 - TIM2_CH4 / UART3_RX (AF7)
#endif
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_LED | TIM_USE_TRANSPONDER, 0), // LED_STRIP / TRANSPONDER
};
