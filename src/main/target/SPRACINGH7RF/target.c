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

#include "build/debug_pin.h"

// TIM1, TIM2, TIM15 reserved for OSD.
// TIM4 can be used for GYRO INT capture/FSYNC generation
// TIM8 used for SX1280 timing

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // On 4in1ESC_1 (Top, CPU side)
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,               0,  0,  0 ), // M1
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,               0,  1,  0 ), // ..
    DEF_TIM(TIM3,  CH1, PA6,  TIM_USE_MOTOR,               0,  2,  0 ), // ..
    DEF_TIM(TIM3,  CH2, PA7,  TIM_USE_MOTOR,               0,  3,  0 ), // M4

    // On 4in1ESC_2 (Bottom)
    DEF_TIM(TIM5,  CH1, PA0,  TIM_USE_MOTOR,               0,  4,  0 ), // M5
    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_MOTOR,               0,  5,  0 ), // ..
    DEF_TIM(TIM5,  CH3, PA2,  TIM_USE_MOTOR,               0,  6,  0 ), // ..
    DEF_TIM(TIM5,  CH4, PA3,  TIM_USE_MOTOR,               0,  7,  0 ), // M8


    // On SX1280
    DEF_TIM(TIM8,  CH1, PC6,  TIM_USE_NONE,                0,  8,  1 ),  // SX1280 DIO1
    DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_NONE,                0,  9,  1 ),  // SX1280 BUSY

    // On GYRO
    DEF_TIM(TIM4,  CH3, PD14, TIM_USE_NONE,                0,  0,  0 ), // Gyro SYNC
    DEF_TIM(TIM4,  CH4, PD15, TIM_USE_NONE,                0,  0,  0 ), // Gyro INT

    // On
    DEF_TIM(TIM17, CH1N, PB7, TIM_USE_LED,                0,  12, 1 ), // LED Strip

    DEF_TIM(TIM15, CH1, PE5,  TIM_USE_VIDEO_PIXEL,         0,  15, 15 ), // Pixel DMA
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_VIDEO_SYNC,          0,  14, 14 ), // Sync

};


dbgPin_t dbgPins[DEBUG_PIN_COUNT] = {
    { .tag = IO_TAG(PD9) },
};
