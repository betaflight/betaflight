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

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // INPUTS CH1-8
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MOTOR, 0), // PWM1 - PA8
    DEF_TIM(TIM16, CH1, PB8,  TIM_USE_PWM,   0), // PWM2 - PB8
    DEF_TIM(TIM17, CH1, PB9,  TIM_USE_PWM,   0), // PWM3 - PB9
    DEF_TIM(TIM8,  CH1, PC6,  TIM_USE_MOTOR, 0), // PWM4 - PC6
    DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_MOTOR, 0), // PWM5 - PC7
    DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_MOTOR, 0), // PWM6 - PC8
    DEF_TIM(TIM15, CH1, PF9,  TIM_USE_PWM,   0), // PWM7 - PF9
    DEF_TIM(TIM15, CH2, PF10, TIM_USE_PWM,   0), // PWM8 - PF10
    DEF_TIM(TIM4,  CH1, PD12, TIM_USE_PWM,   0), // PWM9 - PD12
    DEF_TIM(TIM4,  CH2, PD13, TIM_USE_PWM,   0), // PWM10 - PD13
    DEF_TIM(TIM4,  CH3, PD14, TIM_USE_PWM,   0), // PWM11 - PD14
    DEF_TIM(TIM4,  CH4, PD15, TIM_USE_PWM,   0), // PWM12 - PD15
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_PWM,   0), // PWM13 - PA1
    DEF_TIM(TIM2,  CH3, PA2,  TIM_USE_PWM,   0), // PWM14 - PA2
    DEF_TIM(TIM2,  CH4, PA3,  TIM_USE_PWM,   0), // PWM15 - PA3
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_PWM,   0), // PWM16 - PB0
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_PWM,   0), // PWM17 - PB1
    DEF_TIM(TIM3,  CH2, PA4,  TIM_USE_PWM,   0), // PWM18 - PA4
};
