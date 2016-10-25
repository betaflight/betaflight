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
    { TIM4, IO_TAG(PB6), TIM_Channel_1, TIM4_IRQn,    0, IOCFG_IPD },   // S1_IN
    { TIM3, IO_TAG(PB5), TIM_Channel_2, TIM3_IRQn,    0, IOCFG_IPD },   // S2_IN - SoftSerial TX - GPIO_PartialRemap_TIM3 / Sonar trigger
    { TIM3, IO_TAG(PB0), TIM_Channel_3, TIM3_IRQn,    0, IOCFG_IPD },   // S3_IN - SoftSerial RX / Sonar echo / RSSI ADC
    { TIM3, IO_TAG(PB1), TIM_Channel_4, TIM3_IRQn,    0, IOCFG_IPD },   // S4_IN - Current
    { TIM2, IO_TAG(PA0), TIM_Channel_1, TIM2_IRQn,    0, IOCFG_IPD },   // S5_IN - Vbattery
    { TIM2, IO_TAG(PA1), TIM_Channel_2, TIM2_IRQn,    0, IOCFG_IPD },   // S6_IN - PPM IN
    { TIM4, IO_TAG(PB9), TIM_Channel_4, TIM4_IRQn,    1, IOCFG_AF_PP }, // S1_OUT
    { TIM4, IO_TAG(PB8), TIM_Channel_3, TIM4_IRQn,    1, IOCFG_AF_PP }, // S2_OUT
    { TIM4, IO_TAG(PB7), TIM_Channel_2, TIM4_IRQn,    1, IOCFG_AF_PP }, // S3_OUT
    { TIM1, IO_TAG(PA8), TIM_Channel_1, TIM1_CC_IRQn, 1, IOCFG_AF_PP }, // S4_OUT
    { TIM3, IO_TAG(PB4), TIM_Channel_1, TIM3_IRQn,    1, IOCFG_AF_PP }, // S5_OUT - GPIO_PartialRemap_TIM3 - LED Strip
    { TIM2, IO_TAG(PA2), TIM_Channel_3, TIM2_IRQn,    1, IOCFG_AF_PP }  // S6_OUT
};

