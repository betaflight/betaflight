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
    DEF_TIM(TIM3,  CH2, PA4, TIM_USE_MOTOR, 0), // PWM1  - PA4  - *TIM3_CH2
    DEF_TIM(TIM3,  CH1, PA6, TIM_USE_MOTOR, 0), // PWM2  - PA6  - *TIM3_CH1, TIM8_BKIN, TIM1_BKIN, TIM16_CH1
    DEF_TIM(TIM3,  CH3, PB0, TIM_USE_MOTOR, 0), // PWM3  - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    DEF_TIM(TIM3,  CH4, PB1, TIM_USE_MOTOR, 0), // PWM4  - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    DEF_TIM(TIM2,  CH2, PA1, TIM_USE_MOTOR, 0), // PWM5  - PA1  - *TIM2_CH2, TIM15_CH1N
    DEF_TIM(TIM2,  CH3, PA2, TIM_USE_MOTOR, 0), // PWM6  - PA2  - *TIM2_CH3, !TIM15_CH1
    DEF_TIM(TIM15, CH2, PA3, TIM_USE_MOTOR, 0), // PWM7  - PA3  - *TIM15_CH2, TIM2_CH4
    DEF_TIM(TIM1,  CH1, PA8, TIM_USE_MOTOR, 0), // PWM8  - PA8  - *TIM1_CH1, TIM4_ETR
    DEF_TIM(TIM17, CH1, PA7, TIM_USE_PPM,   0), // PPM   - PA7  - *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    DEF_TIM(TIM16, CH1, PB8, TIM_USE_LED,   0), // LED   - PB8  - *TIM16_CH1, TIM4_CH3, TIM8_CH2
};
