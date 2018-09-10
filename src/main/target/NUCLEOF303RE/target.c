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
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_PPM,         0), // PWM1 / PPM / UART2 RX
    DEF_TIM(TIM15, CH1, PA2,  TIM_USE_PWM,         0), // PWM2

    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,       0), // ESC1

#ifdef USE_DSHOT
    DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_MOTOR,       0), // ESC2
#else
    DEF_TIM(TIM3,  CH2, PC7,  TIM_USE_MOTOR,       0), // ESC2
#endif

    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,       0), // ESC3

#ifdef USE_DSHOT
    DEF_TIM(TIM8,  CH1, PC6,  TIM_USE_MOTOR,       0), // ESC4
#else
    DEF_TIM(TIM3,  CH1, PC6,  TIM_USE_MOTOR,       0), // ESC4
#endif

#ifndef USE_DSHOT
    // with DSHOT TIM8 is used for DSHOT and cannot be used for PWM
    DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_MOTOR,       0), // ESC5
    DEF_TIM(TIM8,  CH4, PC9,  TIM_USE_MOTOR,       0), // ESC6
#endif

    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_MOTOR,       0), // PWM3 - PB10 - *TIM2_CH3, UART3_TX (AF7)
    DEF_TIM(TIM2,  CH4, PB11, TIM_USE_MOTOR,       0), // PWM4 - PB11 - *TIM2_CH4, UART3_RX (AF7)

    // with DSHOT DMA1-CH3 conflicts with TIM3_CH4 / ESC1.
    DEF_TIM(TIM16, CH1, PB8,  TIM_USE_TRANSPONDER, 0),

    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_LED,         0),
};
