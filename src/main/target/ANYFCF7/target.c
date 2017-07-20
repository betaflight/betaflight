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
    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, 0, IOCFG_AF_PP_PD,      GPIO_AF9_TIM12, TIM_USE_PPM | TIM_USE_PWM }, // S1_IN 1
    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, 0, IOCFG_AF_PP_PD,      GPIO_AF9_TIM12, TIM_USE_PWM }, // S2_IN 2
    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, 0, IOCFG_AF_PP_PD,      GPIO_AF3_TIM8,  TIM_USE_PWM | TIM_USE_MC_SERVO | TIM_USE_MC_CHNFW }, // S3_IN 3
    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, 0, IOCFG_AF_PP_PD,      GPIO_AF3_TIM8,  TIM_USE_PWM | TIM_USE_MC_SERVO | TIM_USE_MC_CHNFW }, // S4_IN 4
    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, 0, IOCFG_AF_PP_PD,      GPIO_AF3_TIM8,  TIM_USE_PWM | TIM_USE_MC_SERVO | TIM_USE_MC_CHNFW}, // S6_IN 6
    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, 0, IOCFG_AF_PP_PD,      GPIO_AF3_TIM8,  TIM_USE_PWM | TIM_USE_MC_SERVO | TIM_USE_MC_CHNFW}, // S5_IN 5

    { TIM4,  IO_TAG(PB8),  TIM_CHANNEL_3, 1, IOCFG_AF_PP_PD, GPIO_AF2_TIM4,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO  }, // S10_OUT16
    { TIM2,  IO_TAG(PA2),  TIM_CHANNEL_3, 1, IOCFG_AF_PP_PD, GPIO_AF1_TIM2,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO  }, // S6_OUT 12
    { TIM4,  IO_TAG(PB9),  TIM_CHANNEL_4, 1, IOCFG_AF_PP_PD, GPIO_AF2_TIM4,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO  }, // S5_OUT 11
    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, 1, IOCFG_AF_PP_PD, GPIO_AF1_TIM2,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_MOTOR }, // S1_OUT 7
    { TIM5,  IO_TAG(PA1),  TIM_CHANNEL_2, 1, IOCFG_AF_PP_PD, GPIO_AF2_TIM5,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_MOTOR }, // S2_OUT 8
    { TIM9,  IO_TAG(PE6),  TIM_CHANNEL_2, 1, IOCFG_AF_PP_PD, GPIO_AF3_TIM9,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S3_OUT 9
    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, 1, IOCFG_AF_PP_PD, GPIO_AF2_TIM3,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S4_OUT 10
    { TIM5,  IO_TAG(PA0),  TIM_CHANNEL_1, 1, IOCFG_AF_PP_PD, GPIO_AF2_TIM5,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S7_OUT 13
    { TIM2,  IO_TAG(PB3),  TIM_CHANNEL_2, 1, IOCFG_AF_PP_PD, GPIO_AF1_TIM2,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S8_OUT 14
    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, 1, IOCFG_AF_PP_PD, GPIO_AF2_TIM3,  TIM_USE_MC_MOTOR |                      TIM_USE_FW_SERVO }, // S9_OUT 15
};

// ALTERNATE LAYOUT
//const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
//    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP_PD,      GPIO_AF9_TIM12 }, // S1_IN
//    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP_PD,      GPIO_AF9_TIM12 }, // S2_IN
//    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM8_CC_IRQn,        0, IOCFG_AF_PP_PD,      GPIO_AF3_TIM8  }, // S3_IN
//    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM8_CC_IRQn,        0, IOCFG_AF_PP_PD,      GPIO_AF3_TIM8  }, // S4_IN
//    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM8_CC_IRQn,        0, IOCFG_AF_PP_PD,      GPIO_AF3_TIM8  }, // S5_IN
//    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM8_CC_IRQn,        0, IOCFG_AF_PP_PD,      GPIO_AF3_TIM8  }, // S6_IN
//
//    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, TIM2_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF1_TIM2 }, // S1_OUT
//    { TIM5,  IO_TAG(PA1),  TIM_CHANNEL_2, TIM5_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF2_TIM5 }, // S2_OUT
//    { TIM9,  IO_TAG(PE6),  TIM_CHANNEL_2, TIM1_BRK_TIM9_IRQn,  1, IOCFG_AF_PP_PD, GPIO_AF3_TIM9 }, // S3_OUT
//    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, TIM3_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF2_TIM3 }, // S4_OUT
//    { TIM11, IO_TAG(PB9),  TIM_CHANNEL_1, TIM1_TRG_COM_TIM11_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF3_TIM11 }, // S5_OUT
//    { TIM9,  IO_TAG(PA2),  TIM_CHANNEL_1, TIM1_BRK_TIM9_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF3_TIM9 }, // S6_OUT
//    { TIM5,  IO_TAG(PA0),  TIM_CHANNEL_1, TIM5_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF2_TIM5 }, // S7_OUT
//    { TIM2,  IO_TAG(PB3),  TIM_CHANNEL_2, TIM2_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF1_TIM2 }, // S8_OUT
//    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, TIM3_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF2_TIM3 }, // S9_OUT
//    { TIM10,  IO_TAG(PB8),  TIM_CHANNEL_1, TIM1_UP_TIM10_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF3_TIM10 }, // S10_OUT
//};
