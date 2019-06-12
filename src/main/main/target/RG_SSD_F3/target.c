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
    DEF_TIM(TIM2,  CH4,  PA3, TIM_USE_PPM,         0),
    // Main outputs 6 PWM
    DEF_TIM(TIM4,  CH1,  PB6, TIM_USE_MOTOR,       0),
    DEF_TIM(TIM4,  CH2,  PB7, TIM_USE_MOTOR,       0),
    DEF_TIM(TIM8,  CH2,  PB8, TIM_USE_MOTOR,       0),
    DEF_TIM(TIM8,  CH3,  PB9, TIM_USE_MOTOR,       0),
    DEF_TIM(TIM3,  CH1,  PC6, TIM_USE_MOTOR,       0),
    DEF_TIM(TIM3,  CH2,  PC7, TIM_USE_MOTOR,       0),
    // Additional outputs
    DEF_TIM(TIM16, CH1,  PA6, TIM_USE_TRANSPONDER, 0),
    DEF_TIM(TIM1 , CH1N, PA7, TIM_USE_LED,         0),
};
