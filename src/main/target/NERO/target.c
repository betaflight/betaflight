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

// QUAD + LED_STRIP can be handled without DMAR (dshot_burst).
// Anything beyond should use DMAR, as TIM5_CH1 and TIM3_CH4 have
// inevitable DMA collision on D(1,2).
//
// Additional DMA resource info
// ADC1    D(2,4)
// TIM5_UP U(1,0)
// TIM3_UP U(1,2)
// TIM8_UP U(2,1)

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM3, CH2, PC7, TIM_USE_PPM,                 0, 0 ),
    DEF_TIM(TIM5, CH1, PA0, TIM_USE_MOTOR,               0, 0 ), // D(1,2)
    DEF_TIM(TIM5, CH2, PA1, TIM_USE_MOTOR,               0, 0 ), // D(1,4)
    DEF_TIM(TIM5, CH3, PA2, TIM_USE_MOTOR,               0, 0 ), // D(1,0)
    DEF_TIM(TIM5, CH4, PA3, TIM_USE_MOTOR,               0, 0 ), // *D(1,1) D(1,3)
    DEF_TIM(TIM3, CH3, PB0, TIM_USE_MOTOR | TIM_USE_LED, 0, 0 ), // D(1,7)
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_MOTOR,               0, 0 ), // xD(1,2)
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_MOTOR,               0, 0 ), // *D(2,2) D(2,4)
    DEF_TIM(TIM8, CH4, PC9, TIM_USE_MOTOR,               0, 0 ), // D(2,7)
};
