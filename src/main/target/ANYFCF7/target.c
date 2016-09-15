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

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8), // motor #1
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8), // motor #2
    PWM16 | (MAP_TO_SERVO_OUTPUT << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT << 8), // servo #2
    PWM11 | (MAP_TO_SERVO_OUTPUT << 8), // servo #3
    PWM9  | (MAP_TO_SERVO_OUTPUT << 8), // servo #4
    PWM10 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),    // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),    // input #6
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8), // motor #1
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8), // motor #2
    PWM16 | (MAP_TO_SERVO_OUTPUT << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT << 8), // servo #2
    PWM11 | (MAP_TO_SERVO_OUTPUT << 8), // servo #3
    PWM9  | (MAP_TO_SERVO_OUTPUT << 8), // servo #4
    PWM10 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT << 8),
    0xFFFF
};


const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, TIM8_BRK_TIM12_IRQn, 0, IOCFG_IPD,      GPIO_AF9_TIM12 }, // S1_IN
    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, TIM8_BRK_TIM12_IRQn, 0, IOCFG_IPD,      GPIO_AF9_TIM12 }, // S2_IN
    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM8_CC_IRQn,        0, IOCFG_IPD,      GPIO_AF3_TIM8  }, // S3_IN
    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM8_CC_IRQn,        0, IOCFG_IPD,      GPIO_AF3_TIM8  }, // S4_IN
    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM8_CC_IRQn,        0, IOCFG_IPD,      GPIO_AF3_TIM8  }, // S5_IN
    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM8_CC_IRQn,        0, IOCFG_IPD,      GPIO_AF3_TIM8  }, // S6_IN

    { TIM2,  IO_TAG(PA3),  TIM_CHANNEL_4, TIM2_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF1_TIM2 }, // S1_OUT
    { TIM5,  IO_TAG(PA1),  TIM_CHANNEL_2, TIM5_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF2_TIM5 }, // S2_OUT
    { TIM9,  IO_TAG(PE6),  TIM_CHANNEL_2, TIM1_BRK_TIM9_IRQn,  1, IOCFG_AF_PP_PD, GPIO_AF3_TIM9 }, // S3_OUT
    { TIM3,  IO_TAG(PB5),  TIM_CHANNEL_2, TIM3_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF2_TIM3 }, // S4_OUT
    { TIM4,  IO_TAG(PB9),  TIM_CHANNEL_4, TIM4_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF2_TIM4 }, // S5_OUT
    { TIM2,  IO_TAG(PA2),  TIM_CHANNEL_3, TIM2_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF1_TIM2 }, // S6_OUT
    { TIM5,  IO_TAG(PA0),  TIM_CHANNEL_1, TIM5_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF2_TIM5 }, // S7_OUT
    { TIM2,  IO_TAG(PB3),  TIM_CHANNEL_2, TIM2_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF1_TIM2 }, // S8_OUT
    { TIM3,  IO_TAG(PB4),  TIM_CHANNEL_1, TIM3_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF2_TIM3 }, // S9_OUT
    { TIM4,  IO_TAG(PB8),  TIM_CHANNEL_3, TIM4_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF2_TIM4 }, // S10_OUT
};

// ALTERNATE LAYOUT
//const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
//    { TIM12, IO_TAG(PB14), TIM_CHANNEL_1, TIM8_BRK_TIM12_IRQn, 0, IOCFG_IPD,      GPIO_AF9_TIM12 }, // S1_IN
//    { TIM12, IO_TAG(PB15), TIM_CHANNEL_2, TIM8_BRK_TIM12_IRQn, 0, IOCFG_IPD,      GPIO_AF9_TIM12 }, // S2_IN
//    { TIM8,  IO_TAG(PC6),  TIM_CHANNEL_1, TIM8_CC_IRQn,        0, IOCFG_IPD,      GPIO_AF3_TIM8  }, // S3_IN
//    { TIM8,  IO_TAG(PC7),  TIM_CHANNEL_2, TIM8_CC_IRQn,        0, IOCFG_IPD,      GPIO_AF3_TIM8  }, // S4_IN
//    { TIM8,  IO_TAG(PC9),  TIM_CHANNEL_4, TIM8_CC_IRQn,        0, IOCFG_IPD,      GPIO_AF3_TIM8  }, // S5_IN
//    { TIM8,  IO_TAG(PC8),  TIM_CHANNEL_3, TIM8_CC_IRQn,        0, IOCFG_IPD,      GPIO_AF3_TIM8  }, // S6_IN
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
