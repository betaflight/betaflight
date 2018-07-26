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

    DEF_TIM(TIM11, CH1,  PB9, TIM_USE_CAMERA_CONTROL, 0, 0), // CAMERA_CONTROL_PIN

    DEF_TIM(TIM3,  CH4,  PB1, TIM_USE_MOTOR,           0, 0),  // M1_OUT - D1_ST2_CH5
    DEF_TIM(TIM8,  CH2N, PB0, TIM_USE_MOTOR,           0, 0),  // M2_OUT - D2_ST3_CH7
    DEF_TIM(TIM8,  CH4,  PC9, TIM_USE_MOTOR,           0, 0),  // M3_OUT - D2_ST7_CH7
    DEF_TIM(TIM1,  CH1,  PA8, TIM_USE_MOTOR,           0, 1),  // M4_OUT - D2_ST1_CH6
    DEF_TIM(TIM4,  CH3,  PB8, TIM_USE_LED ,            0,  0)    // LED & MOTOR5 D1_ST7_CH2
};
