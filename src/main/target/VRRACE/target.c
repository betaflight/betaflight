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
    DEF_TIM(TIM1, CH1, PE9,  TIM_USE_PWM | TIM_USE_PPM, 0, 0), // PPM
    DEF_TIM(TIM1, CH2, PE11, TIM_USE_PWM,               0, 0), // S2_IN
    DEF_TIM(TIM1, CH3, PE13, TIM_USE_PWM,               0, 0), // S3_IN
    DEF_TIM(TIM1, CH4, PE14, TIM_USE_PWM,               0, 0), // S4_IN
    DEF_TIM(TIM9, CH1, PE6,  TIM_USE_PWM,               0, 0), // S5_IN
    DEF_TIM(TIM9, CH2, PE7,  TIM_USE_PWM,               0, 0), // S6_IN
    DEF_TIM(TIM2, CH2, PA1,  TIM_USE_MOTOR,             0, 0), // S1_OUT
    DEF_TIM(TIM2, CH3, PA2,  TIM_USE_MOTOR,             0, 0), // S2_OUT
    DEF_TIM(TIM2, CH4, PA3,  TIM_USE_MOTOR,             0, 0), // S3_OUT
    DEF_TIM(TIM3, CH2, PB5,  TIM_USE_MOTOR,             0, 0), // S4_OUT
    DEF_TIM(TIM3, CH3, PB0,  TIM_USE_MOTOR,             0, 0), // S5_OUT
    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_MOTOR,             0, 0), // S6_OUT
};
