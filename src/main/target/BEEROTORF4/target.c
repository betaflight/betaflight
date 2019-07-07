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
    DEF_TIM(TIM9, CH2,  PA3, TIM_USE_PPM,                       0, 0), // PPM IN

    DEF_TIM(TIM1, CH2N, PB0, TIM_USE_MOTOR,                     0, 0), // M1 - DMA2_ST6
    DEF_TIM(TIM8, CH3N, PB1, TIM_USE_MOTOR,                     0, 1), // M2 - DMA2_ST4
    DEF_TIM(TIM2, CH2,  PA1, TIM_USE_MOTOR,                     0, 0), // M3 - DMA1_ST6
    DEF_TIM(TIM5, CH1,  PA0, TIM_USE_MOTOR,                     0, 0), // M4 - DMA1_ST2
    DEF_TIM(TIM3, CH1,  PC6, TIM_USE_MOTOR,                     0, 0), // M5 - DMA1_ST4 (conflicts with SDCard, switch off SDCard DMA if used for DShot)
    DEF_TIM(TIM8, CH2,  PC7, TIM_USE_MOTOR,                     0, 0), // M6 - DMA2_ST2
    DEF_TIM(TIM3, CH2,  PB5, TIM_USE_MOTOR,                     0, 0), // M7 - DMA1_ST5 (doesn't work for DShot)
    DEF_TIM(TIM4, CH4,  PB9, TIM_USE_MOTOR,                     0, 0), // M8 - no DMA

    DEF_TIM(TIM4, CH3,  PB8, TIM_USE_LED | TIM_USE_TRANSPONDER, 0, 0), // LED_STRIP / TRANSPONDER - DMA1_ST7 (can be used for DShot, conflicts with OSD TX)
};
