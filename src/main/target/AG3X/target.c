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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
                                                     // ADC1   *D(2,4) D(2,0)
                                                     // ADC2   D(2,2) D(2,3)
                                                     // ADC3   D(2,0) D(2,1)

    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_PPM,            0, 0), // PPM    D(1,0)

    // Motors
    DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_MOTOR,          0, 0), // MOTOR1 U(2,1) *D(2,2) D(2,4)
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,          0, 0), // MOTOR2 U(1,2) D(1,7)
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,          0, 0), // MOTOR3 U(1,2) D(1,2)
    DEF_TIM(TIM4,  CH2, PB7,  TIM_USE_MOTOR,          0, 0), // MOTOR4 U(1,6) D(1,3)

    // More outputs
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_MOTOR,          0, 0), // MOTOR5 U(1,6) D(1,7)
    DEF_TIM(TIM8,  CH4, PC9,  TIM_USE_MOTOR,          0, 0), // MOTOR6 U(2,1) D(2,7)
    DEF_TIM(TIM12, CH1, PB14, TIM_USE_MOTOR,          0, 0), // MOTOR7 ; SS2
    DEF_TIM(TIM12, CH2, PB15, TIM_USE_MOTOR,          0, 0), // MOTOR8

    // DEF_TIM(TIM4,  CH1, PB6,  TIM_USE_LED, 0, 0), // LED strip XXX Collision with M4

    // Backdoor timers
    // UART1 (Collision with PPM)
    DEF_TIM(TIM1,  CH2, PA9,  TIM_USE_NONE,           0, 1), // TX1    D(2,6) D(2,2) ; SS1
    DEF_TIM(TIM1,  CH3, PA10, TIM_USE_NONE,           0, 0), // RX1    D(2,6) D(2,6)

    // UART2
    DEF_TIM(TIM5,  CH3, PA2,  TIM_USE_LED,            0, 0), // TX2    D(1,0) ; SA port ---> LED
    DEF_TIM(TIM9,  CH2, PA3,  TIM_USE_CAMERA_CONTROL, 0, 0), // RX2           ; CAMC port

    // I2C2
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_NONE,   0, 0), // SCL2   D(1,1)
    DEF_TIM(TIM2,  CH4, PB11, TIM_USE_NONE,   0, 0), // SDA2   D(1,7) D(1,6)
};
