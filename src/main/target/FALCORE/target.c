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
    // MOTOR outputs
    { TIM8,  IO_TAG(PC6),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_4,    TIM_USE_MC_MOTOR }, // PWM1 - PC6 - TIM8_CH1
    { TIM8,  IO_TAG(PC7),  TIM_Channel_2, 1, IOCFG_AF_PP, GPIO_AF_4,    TIM_USE_MC_MOTOR }, // PWM2 - PC7 - TIM8_CH2
    { TIM8,  IO_TAG(PC8),  TIM_Channel_3, 1, IOCFG_AF_PP, GPIO_AF_4,    TIM_USE_MC_MOTOR }, // PWM3 - PC8 - TIM8_CH3
    { TIM8,  IO_TAG(PC9),  TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_4,    TIM_USE_MC_MOTOR }, // PWM4 - PC9 - TIM8_CH4

    // Additional servo outputs
    { TIM3,  IO_TAG(PA4),  TIM_Channel_2, 0, IOCFG_AF_PP, GPIO_AF_2,    TIM_USE_MC_SERVO }, // PWM5 - PA4  - TIM3_CH2
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, 0, IOCFG_AF_PP, GPIO_AF_2,    TIM_USE_MC_SERVO }, // PWM6 - PB1  - TIM3_CH4

    // PPM PORT - Also USART2 RX (AF5)
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, 0, IOCFG_AF_PP, GPIO_AF_1,    TIM_USE_PPM }, // PPM - PA3  - TIM2_CH4

    // LED_STRIP
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, 0, IOCFG_AF_PP, GPIO_AF_6,    TIM_USE_ANY },  // GPIO_TIMER / LED_STRIP

    // PWM_BUZZER
    { TIM16, IO_TAG(PB4),  TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_1,    TIM_USE_BEEPER },  // BUZZER - PB4 - TIM16_CH1N
};

