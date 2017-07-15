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

    { TIM1,  IO_TAG(PE13), TIM_CHANNEL_3, 0, IOCFG_AF_PP_PD, GPIO_AF1_TIM1, TIM_USE_PPM         | TIM_USE_PWM       }, //PPM Input
    { TIM3,  IO_TAG(PB0),  TIM_CHANNEL_3, 1, IOCFG_AF_PP_PD, GPIO_AF2_TIM3, TIM_USE_MC_MOTOR    | TIM_USE_FW_MOTOR  }, //Motor 1
    { TIM3,  IO_TAG(PB1),  TIM_CHANNEL_4, 1, IOCFG_AF_PP_PD, GPIO_AF2_TIM3, TIM_USE_MC_MOTOR    | TIM_USE_FW_MOTOR  }, //Motor 2
    { TIM1,  IO_TAG(PE9),  TIM_CHANNEL_1, 1, IOCFG_AF_PP_PD, GPIO_AF1_TIM1, TIM_USE_MC_MOTOR    | TIM_USE_FW_SERVO  }, //Motor 3
    { TIM1,  IO_TAG(PE11), TIM_CHANNEL_2, 1, IOCFG_AF_PP_PD, GPIO_AF1_TIM1, TIM_USE_MC_MOTOR    | TIM_USE_FW_SERVO  }, //Motor 4
    // DEF_TIM(TIM1,  CH3, PE13, TIM_USE_PPM, 0, 1), // RC1 / PPM

    // DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR, 1, 0), // M1
    // DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR, 1, 0), // M2
    // DEF_TIM(TIM1,  CH1, PE9,  TIM_USE_MOTOR, 1, 2), // M3
    // DEF_TIM(TIM1,  CH2, PE11, TIM_USE_MOTOR, 1, 1), // M4

    // DEF_TIM(TIM4,  CH1, PD12,  TIM_USE_LED, 1,  0 ), // LED
};
