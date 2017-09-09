/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {

    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, 0, IOCFG_AF_PP_PD, GPIO_AF1_TIM2, TIM_USE_PPM         | TIM_USE_PWM       }, //PPM Input on UART2 RX
    { TIM3,  IO_TAG(PB0),  TIM_CHANNEL_3, 1, IOCFG_AF_PP_PD, GPIO_AF2_TIM3, TIM_USE_MC_MOTOR    | TIM_USE_FW_MOTOR  }, //Motor 1
    { TIM3,  IO_TAG(PB1),  TIM_CHANNEL_4, 1, IOCFG_AF_PP_PD, GPIO_AF2_TIM3, TIM_USE_MC_MOTOR    | TIM_USE_FW_MOTOR  }, //Motor 2
    { TIM1,  IO_TAG(PE9),  TIM_CHANNEL_1, 1, IOCFG_AF_PP_PD, GPIO_AF1_TIM1, TIM_USE_MC_MOTOR    | TIM_USE_FW_SERVO  }, //Motor 3
    { TIM1,  IO_TAG(PE11), TIM_CHANNEL_2, 1, IOCFG_AF_PP_PD, GPIO_AF1_TIM1, TIM_USE_MC_MOTOR    | TIM_USE_FW_SERVO  }, //Motor 4

    // DEF_TIM(TIM4,  CH1, PD12,  TIM_USE_LED, 1,  0 ), // LED
};
