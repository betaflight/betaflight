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
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3, IO_TAG(PB0),  TIM_Channel_3, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM3, TIM_USE_PPM },  // PPM shared uart6 pc7
    { TIM1, IO_TAG(PE14), TIM_Channel_4, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM1, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },  // S1_OUT
    { TIM1, IO_TAG(PE13), TIM_Channel_3, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM1, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },  // S2_OUT
    { TIM1, IO_TAG(PE11), TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM1, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },  // S3_OUT
    { TIM1, IO_TAG(PE9),  TIM_Channel_1, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM1, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO },  // S4_OUT
    { TIM4, IO_TAG(PD13), TIM_Channel_2, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM4, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR },  // S5_OUT
    { TIM4, IO_TAG(PD14), TIM_Channel_3, 1, IOCFG_AF_PP_PD, GPIO_AF_TIM4, TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR },  // S6_OUT
};
