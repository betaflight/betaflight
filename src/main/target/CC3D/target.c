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
#ifdef CC3D_PPM1
    { TIM4, IO_TAG(PB6), TIM_Channel_1, TIM4_IRQn,    0, IOCFG_IPD,  TIM_USE_PPM | TIM_USE_PWM },   // S1_IN
#else
    { TIM4, IO_TAG(PB6), TIM_Channel_1, TIM4_IRQn,    0, IOCFG_IPD,  TIM_USE_PWM },   // S1_IN
#endif
    { TIM3, IO_TAG(PB5), TIM_Channel_2, TIM3_IRQn,    0, IOCFG_IPD,  TIM_USE_PWM },   // S2_IN - SoftSerial TX - GPIO_PartialRemap_TIM3 / Sonar trigger
    { TIM3, IO_TAG(PB0), TIM_Channel_3, TIM3_IRQn,    0, IOCFG_IPD,  TIM_USE_PWM },   // S3_IN - SoftSerial RX / Sonar echo / RSSI ADC
    { TIM3, IO_TAG(PB1), TIM_Channel_4, TIM3_IRQn,    0, IOCFG_IPD,  TIM_USE_PWM },   // S4_IN - Current
    { TIM2, IO_TAG(PA0), TIM_Channel_1, TIM2_IRQn,    0, IOCFG_IPD,  TIM_USE_PWM },   // S5_IN - Vbattery
#ifdef CC3D_PPM1
    { TIM2, IO_TAG(PA1), TIM_Channel_2, TIM2_IRQn,    0, IOCFG_IPD,  TIM_USE_PWM },   // S6_IN - PPM IN
#else
    { TIM2, IO_TAG(PA1), TIM_Channel_2, TIM2_IRQn,    0, IOCFG_IPD,  TIM_USE_PPM | TIM_USE_PWM },   // S6_IN - PPM IN
#endif

    { TIM4, IO_TAG(PB9), TIM_Channel_4, TIM4_IRQn,    1, IOCFG_AF_PP, TIM_USE_MC_MOTOR |                    TIM_USE_FW_MOTOR}, // S1_OUT
    { TIM4, IO_TAG(PB8), TIM_Channel_3, TIM4_IRQn,    1, IOCFG_AF_PP, TIM_USE_MC_MOTOR |                    TIM_USE_FW_MOTOR}, // S1_OUT
    { TIM4, IO_TAG(PB7), TIM_Channel_2, TIM4_IRQn,    1, IOCFG_AF_PP, TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO}, // S3_OUT
    { TIM1, IO_TAG(PA8), TIM_Channel_1, TIM1_CC_IRQn, 1, IOCFG_AF_PP, TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO}, // S3_OUT
    { TIM3, IO_TAG(PB4), TIM_Channel_1, TIM3_IRQn,    1, IOCFG_AF_PP, TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO}, // S5_OUT
    { TIM2, IO_TAG(PA2), TIM_Channel_3, TIM2_IRQn,    1, IOCFG_AF_PP, TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO}, // S6_OUT
};
