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

    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_PPM,   0), // PWM1 - PA8
    DEF_TIM(TIM3,  CH1, PC6,  TIM_USE_MOTOR, 0), // PWM2 - PC6
    DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_MOTOR, 0), // PWM3 - PC7
    DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_MOTOR, 0), // PMW4 - PC8
    DEF_TIM(TIM8,  CH4, PC9,  TIM_USE_MOTOR, 0), // PWM5 - PC9
#ifndef LUXV2_RACE
    DEF_TIM(TIM2,  CH1, PA0,  TIM_USE_MOTOR, 0), // PWM6 - PA0
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_MOTOR, 0), // PWM7 - PA1
    DEF_TIM(TIM2,  CH3, PA2,  TIM_USE_MOTOR, 0), // PWM8 - PA2
    DEF_TIM(TIM2,  CH4, PA3,  TIM_USE_MOTOR, 0), // PWM9 - PA3
    DEF_TIM(TIM15, CH1, PB14, TIM_USE_MOTOR, 0), // PWM10 - PB14
    DEF_TIM(TIM15, CH2, PB15, TIM_USE_MOTOR, 0), // PWM11 - PB15
#endif
    DEF_TIM(TIM16, CH1, PA6,  TIM_USE_LED,   0),

};
