/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <platform.h>

#include "drivers/io.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	
	DEF_TIM(TIM8,  CH2, PB8, TIM_USE_MOTOR, 0), // PWM1 - PB8
    DEF_TIM(TIM8,  CH3, PB9, TIM_USE_MOTOR, 0), // PWM2 - PB9
    DEF_TIM(TIM2,  CH4, PA3, TIM_USE_MOTOR, 0), // PWM3 - PA3
    DEF_TIM(TIM15, CH1, PA2, TIM_USE_MOTOR, 0), // PWM4 - PA2
	
};
