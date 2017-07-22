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
    // PWM1 PPM Pad
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, 0, IOCFG_AF_PP, GPIO_AF_2,  TIM_USE_PPM }, // Pin PPM - PB4
    // PB5 / TIM3 CH2 is connected to USBPresent

    // PWM2-PWM5
    { TIM8,  IO_TAG(PB8),  TIM_Channel_2, 1, IOCFG_AF_PP, GPIO_AF_10, TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_MOTOR },// Pin PWM1 - PB8
    { TIM4,  IO_TAG(PB9),  TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_2,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR }, // Pin PWM2 - PB9
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, 1, IOCFG_AF_PP, GPIO_AF_9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // Pin PWM3 - PA3
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // Pin PWM4 - PA2

    // For iNav, PWM6&7 (PWM pins 5&6) are shared with UART3
    { TIM2,  IO_TAG(PB10), TIM_Channel_3, 1, IOCFG_AF_PP, GPIO_AF_1,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // Pin PWM5 - PB10 - TIM2_CH3 / UART3_TX (AF7)
    { TIM2,  IO_TAG(PB11), TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_1,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO }, // Pin PWM6 - PB11 - TIM2_CH4 / UART3_RX (AF7)

    { TIM1, IO_TAG(PA8),   TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_6,  TIM_USE_LED },
};
