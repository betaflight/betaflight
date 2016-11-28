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
    { TIM4, IO_TAG(PB6), TIM_Channel_1, TIM_USE_PWM, 0, },   // S1_IN
    { TIM3, IO_TAG(PB5), TIM_Channel_2, TIM_USE_PWM, 0, },   // S2_IN - SoftSerial TX - GPIO_PartialRemap_TIM3 / Sonar trigger
    { TIM3, IO_TAG(PB0), TIM_Channel_3, TIM_USE_PWM, 0, },   // S3_IN - SoftSerial RX / Sonar echo / RSSI ADC
    { TIM3, IO_TAG(PB1), TIM_Channel_4, TIM_USE_PWM, 0, },   // S4_IN - Current
    { TIM2, IO_TAG(PA0), TIM_Channel_1, TIM_USE_PWM, 0, },   // S5_IN - Vbattery
    { TIM2, IO_TAG(PA1), TIM_Channel_2, TIM_USE_PWM | TIM_USE_PPM, 0, },   // S6_IN - PPM IN
    { TIM4, IO_TAG(PB9), TIM_Channel_4, TIM_USE_MOTOR, 1, }, // S1_OUT
    { TIM4, IO_TAG(PB8), TIM_Channel_3, TIM_USE_MOTOR, 1, }, // S2_OUT
    { TIM4, IO_TAG(PB7), TIM_Channel_2, TIM_USE_MOTOR, 1, }, // S3_OUT
    { TIM1, IO_TAG(PA8), TIM_Channel_1, TIM_USE_MOTOR, 1, }, // S4_OUT
    { TIM3, IO_TAG(PB4), TIM_Channel_1, TIM_USE_MOTOR, 1, }, // S5_OUT - GPIO_PartialRemap_TIM3 - LED Strip
    { TIM2, IO_TAG(PA2), TIM_Channel_3, TIM_USE_MOTOR, 1, }  // S6_OUT
};

