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

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM4,  CH1,  PB6,  TIM_USE_MOTOR,               0, 0), // M1
    DEF_TIM(TIM4,  CH2,  PB7,  TIM_USE_MOTOR,               0, 0), // M2
    DEF_TIM(TIM4,  CH3,  PB8,  TIM_USE_MOTOR,               0, 0), // M3
    DEF_TIM(TIM4,  CH4,  PB9,  TIM_USE_MOTOR,               0, 0), // M4
    DEF_TIM(TIM3,  CH3,  PB0,  TIM_USE_MOTOR,               0, 0), // M5
    DEF_TIM(TIM3,  CH4,  PB1,  TIM_USE_MOTOR,               0, 0), // M6

    DEF_TIM(TIM3,  CH1,  PC6,  TIM_USE_ANY,                 0, 0), // UART6_TX, M7
    DEF_TIM(TIM3,  CH2,  PC7,  TIM_USE_ANY,                 0, 0), // UART6_RX, M8

    DEF_TIM(TIM1,  CH1,  PA8,  TIM_USE_PPM,                 0, 0),
    DEF_TIM(TIM2,  CH1,  PA15, TIM_USE_LED,                 0, 0),
    DEF_TIM(TIM8,  CH4,  PC9,  TIM_USE_CAMERA_CONTROL,      0, 0)
};
