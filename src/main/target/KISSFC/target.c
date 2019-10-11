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
#ifdef KISSCC
    DEF_TIM(TIM1,  CH1,  PA8,  TIM_USE_MOTOR,               0),
    DEF_TIM(TIM8,  CH2N, PB0,  TIM_USE_MOTOR,               0),
    DEF_TIM(TIM1,  CH2N, PB14, TIM_USE_MOTOR,               0),
    DEF_TIM(TIM15, CH1N, PB15, TIM_USE_MOTOR,               0),

    DEF_TIM(TIM3,  CH1,  PA6,  TIM_USE_MOTOR,               0),
    DEF_TIM(TIM17, CH1,  PA7,  TIM_USE_MOTOR,               0),
#else
    DEF_TIM(TIM1,  CH2N, PB14, TIM_USE_MOTOR,               TIMER_OUTPUT_INVERTED),
    DEF_TIM(TIM8,  CH2N, PB0,  TIM_USE_MOTOR,               TIMER_OUTPUT_INVERTED),
    DEF_TIM(TIM15, CH1N, PB15, TIM_USE_MOTOR,               TIMER_OUTPUT_INVERTED),
    DEF_TIM(TIM1,  CH1,  PA8,  TIM_USE_MOTOR,               TIMER_OUTPUT_INVERTED),
    DEF_TIM(TIM3,  CH1,  PA6,  TIM_USE_MOTOR | TIM_USE_LED, TIMER_OUTPUT_INVERTED),
    DEF_TIM(TIM17, CH1,  PA7,  TIM_USE_MOTOR,               TIMER_OUTPUT_INVERTED),
#endif
    DEF_TIM(TIM4,  CH3,  PA13, TIM_USE_PWM,                 0),
    DEF_TIM(TIM2,  CH2,  PB3,  TIM_USE_PWM | TIM_USE_PPM,   0),
    DEF_TIM(TIM2,  CH1,  PA15, TIM_USE_PWM,                 0),
    DEF_TIM(TIM2,  CH3,  PA2,  TIM_USE_PWM,                 0),
    DEF_TIM(TIM2,  CH4,  PB11, TIM_USE_PWM,                 0),
};
