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

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM8,  IO_TAG(PC6),  TIM_Channel_1, TIM_USE_PWM,   0, GPIO_AF_TIM8,  NULL,         0,             0  },  // S3_IN
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, TIM_USE_PWM,   0, GPIO_AF_TIM8,  NULL,         0,             0  },  // S4_IN
	{ TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_TIM3  }, // S1_OUT
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_TIM3  }, // S2_OUT
    { TIM9,  IO_TAG(PA3),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_TIM9  }, // S3_OUT
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_TIM2  }, // S4_OUT
};

