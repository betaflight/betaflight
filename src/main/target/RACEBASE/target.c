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
    DEF_TIM(TIM2,  CH1, PA0, TIM_USE_PPM,   0),
    DEF_TIM(TIM8,  CH1, PB6, TIM_USE_MOTOR, 0), // PWM1 - DMA2 CH3
    DEF_TIM(TIM3,  CH4, PB7, TIM_USE_MOTOR, 0), // PWM2 - DMA1 CH3
    DEF_TIM(TIM8,  CH2, PB8, TIM_USE_MOTOR, 0), // PMW3 - DMA2 CH5
    DEF_TIM(TIM17, CH1, PB9, TIM_USE_MOTOR, 0), // PWM4  -DMA1 CH1
    DEF_TIM(TIM1,  CH1, PA8, TIM_USE_LED,   0), // LED  - DMA1 CH2
};
