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
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),  // Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

/*
 * This target does not allows PWM Input, so this configuration will never be used
 */
const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1 or servo #1 (swap to servo if needed)
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2 or servo #2 (swap to servo if needed)
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1 or #3
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #4 or #6
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

/*
 * This target does not allows PWM Input, so this configuration will never be used
 */
const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #3
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #4
    0xFFFF
};


const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM12, IO_TAG(PB14), TIM_Channel_1, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM12 }, // PPM / S.BUS input, above MOTOR1
    { TIM12, IO_TAG(PB15), TIM_Channel_2, TIM8_BRK_TIM12_IRQn, 0, IOCFG_AF_PP_PD, GPIO_AF_TIM12 }, // Connected: small CH2 pad, not used as PWM, definition inherited from REVO target - GPIO_PartialRemap_TIM3
    { TIM8,  IO_TAG(PC6),  TIM_Channel_1, TIM8_CC_IRQn,        0, IOCFG_AF_PP_PD, GPIO_AF_TIM8  }, // Connected: UART6 TX, not used as PWM, definition inherited from REVO target
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, TIM8_CC_IRQn,        0, IOCFG_AF_PP_PD, GPIO_AF_TIM8  }, // Connected: UART6 RX, not used as PWM, definition inherited from REVO target
    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, TIM8_CC_IRQn,        0, IOCFG_AF_PP_PD, GPIO_AF_TIM8  }, // Connected: small CH5 pad, not used as PWM, definition inherited from REVO target
    { TIM8,  IO_TAG(PC9),  TIM_Channel_4, TIM8_CC_IRQn,        0, IOCFG_AF_PP_PD, GPIO_AF_TIM8  }, // Connected: small CH6 pad, not used as PWM, definition inherited from REVO target
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF_TIM3 }, // MOTOR_1
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF_TIM3 }, // MOTOR_2
    { TIM9,  IO_TAG(PA3),  TIM_Channel_2, TIM1_BRK_TIM9_IRQn,  1, IOCFG_AF_PP_PD, GPIO_AF_TIM9 }, // MOTOR_3
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM2_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF_TIM2 }, // MOTOR_4
    { TIM5,  IO_TAG(PA1),  TIM_Channel_2, TIM5_IRQn,           1, IOCFG_AF_PP_PD, GPIO_AF_TIM5 }, // MOTOR_5 - GPIO_PartialRemap_TIM3
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_UP_TIM10_IRQn,  1, IOCFG_AF_PP_PD, GPIO_AF_TIM1 }, // MOTOR_6
};
