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

/*
 * Timer channel definitions for the HPM6360 target.
 */

#pragma once
#include "platform.h"
#include "common/utils.h"

#define DEF_TIM_CH_GET(ch) CONCAT2(DEF_TIM_CH__, ch)
#define DEF_TIM_CH__CH_CH0  D(0, 0)
#define DEF_TIM_CH__CH_CH1  D(1, 0)
#define DEF_TIM_CH__CH_CH2  D(2, 0)
#define DEF_TIM_CH__CH_CH3  D(3, 0)
#define DEF_TIM_CH__CH_CH4  D(4, 0)
#define DEF_TIM_CH__CH_CH5  D(5, 0)
#define DEF_TIM_CH__CH_CH6  D(6, 0)
#define DEF_TIM_CH__CH_CH7  D(7, 0)
#define DEF_TIM_CH__CH_CH8  D(8, 0)
#define DEF_TIM_CH__CH_CH9  D(9, 0)
#define DEF_TIM_CH__CH_CH10  D(10, 0)
#define DEF_TIM_CH__CH_CH11  D(11, 0)

#define USED_TIMERS  ( BIT(1) | BIT(2) )
#define HARDWARE_TIMER_DEFINITION_COUNT    2

#define TIMER_GET_IO_TAG(pin) DEFIO_TAG(pin)

#define DEF_TIM_CHANNEL(ch)                   CONCAT(DEF_TIM_CHANNEL__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_CHANNEL__D(chan_n, n_channel) chan_n
#define DEF_TIM_OUTPUT(ch)         CONCAT(DEF_TIM_OUTPUT__, DEF_TIM_CH_GET(ch))
#define DEF_TIM_OUTPUT__D(chan_n, n_channel) PP_IIF(n_channel, TIMER_OUTPUT_N_CHANNEL, TIMER_OUTPUT_NONE)
