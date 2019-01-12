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
    // Main motors
    DEF_TIM(TIM3,  CH2,  PB5,  TIM_USE_MOTOR,               0, 0), // M1
    DEF_TIM(TIM3,  CH1,  PB4,  TIM_USE_MOTOR,               0, 0), // M2
    DEF_TIM(TIM3,  CH3,  PB0,  TIM_USE_MOTOR,               0, 0), // M3
    DEF_TIM(TIM3,  CH4,  PB1,  TIM_USE_MOTOR,               0, 0), // M4

    // Additional motors/servos
    DEF_TIM(TIM8,  CH4,  PC9,  TIM_USE_NONE,                0, 0), // MST5 Collision with TX/RX6 (useful for OCTO)
    DEF_TIM(TIM8,  CH3,  PC8,  TIM_USE_NONE,                0, 1), // MST6 Collision with TX/RX6 (useful for OCTO)
    DEF_TIM(TIM11, CH1,  PB9,  TIM_USE_NONE,                0, 0), // I2C1_SDA, MST7
    DEF_TIM(TIM10, CH1,  PB8,  TIM_USE_NONE,                0, 0), // I2C1_SCL, MST8

    // PPM
    DEF_TIM(TIM4,  CH2,  PB7,  TIM_USE_PPM,                 0, 0), // Overloaded with UART1_RX

    // LED strip
    DEF_TIM(TIM1,  CH2,  PA9,  TIM_USE_LED,                 0, 0),

    // CAMCTL
    DEF_TIM(TIM12, CH2,  PB15, TIM_USE_CAMERA_CONTROL,      0, 0),

    // Backdoor timers on UARTs
    DEF_TIM(TIM4,  CH1,  PB6,  TIM_USE_NONE,                0, 0), // UART1_TX Collision with PPM

    DEF_TIM(TIM9,  CH1,  PA2,  TIM_USE_NONE,                0, 0), // UART2_TX
    DEF_TIM(TIM9,  CH2,  PA3,  TIM_USE_NONE,                0, 0), // UART2_RX

    DEF_TIM(TIM2,  CH3,  PB10, TIM_USE_NONE,                0, 0), // UART3_TX
    DEF_TIM(TIM2,  CH4,  PB11, TIM_USE_NONE,                0, 0), // UART3_RX

    DEF_TIM(TIM5,  CH1,  PA0,  TIM_USE_NONE,                0, 0), // UART4_TX
    DEF_TIM(TIM5,  CH2,  PA1,  TIM_USE_NONE,                0, 0), // UART4_RX

    DEF_TIM(TIM8,  CH1,  PC6,  TIM_USE_NONE,                0, 0), // UART6_TX Collision with MS1&2 (useful for OCTO)
    DEF_TIM(TIM8,  CH2,  PC7,  TIM_USE_NONE,                0, 1), // UART6_RX Collision with MS1&2 (useful for OCTO)

    // Others
    DEF_TIM(TIM1,  CH3,  PA10, TIM_USE_NONE,                0, 0), // CS_ExtIMU, Collision with LED_STRIP
};
