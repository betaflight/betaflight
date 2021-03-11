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

// TIM1, TIM2, TIM15 reserved for OSD.
// TIM8 can be used for GYRO INT capture/FSYNC generation

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // On 4in1ESC_1
    DEF_TIM(TIM5,  CH1, PA0,  TIM_USE_MOTOR,               0,  0,  0 ), // M1
    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_MOTOR,               0,  1,  0 ), // ..
    DEF_TIM(TIM5,  CH3, PA2,  TIM_USE_MOTOR,               0,  2,  0 ), // ..
    DEF_TIM(TIM5,  CH4, PA3,  TIM_USE_MOTOR,               0,  3,  0 ), // M4

    // On 4in1ESC_2
    DEF_TIM(TIM3,  CH1, PA6,  TIM_USE_MOTOR,               0,  4,  0 ), // M5
    DEF_TIM(TIM3,  CH2, PA7,  TIM_USE_MOTOR,               0,  5,  0 ), // ..
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,               0,  6,  0 ), // ..
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,               0,  7,  0 ), // M8

    // On GPS_IO1
    DEF_TIM(TIM4,  CH1, PD12, TIM_USE_MOTOR,               0,  8,  1 ), // M9  / I2C4_SCL
    DEF_TIM(TIM4,  CH2, PD13, TIM_USE_MOTOR,               0,  9,  1 ), // ..  / I2C4_SDA
    DEF_TIM(TIM4,  CH3, PD14, TIM_USE_MOTOR,               0,  10, 1 ), // ..  / UART9_RX
    DEF_TIM(TIM4,  CH4, PD15, TIM_USE_MOTOR,               0,  0, 0 ), // M12 / UART9_TX

    // On B_L_IO2
    DEF_TIM(TIM16, CH1, PB8,  TIM_USE_LED,                 0,  12, 1 ), // LED Strip
    DEF_TIM(TIM17, CH1, PB9,  TIM_USE_CAMERA_CONTROL,      0,  13, 1 ), // Camera Control
};


