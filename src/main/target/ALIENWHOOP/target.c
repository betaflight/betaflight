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


/*




         \   |   _ _| __|  \ |\ \      /|  |  _ \  _ \ _ \
        _ \  |     |  _|  .  | \ \ \  / __ | (   |(   |__/
      _/  _\____|___|___|_|\_|  \_/\_/ _| _|\___/\___/_|


              Take me to your leader-board...



*/

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "drivers/dma.h"

/* Currently only supporting brushed quad configuration e.g. Tiny Whoop. Care must be
 * taken to ensure functionality on both F4 and F7 (STM32F405RGT and STM32F722RET)
 */
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM8, CH4, PC9, TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM3, CH3, PC8, TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM3, CH2, PC7, TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM8, CH1, PC6, TIM_USE_MOTOR, 0, 0),

    DEF_TIM(TIM5, CH1, PA0, TIM_USE_LED,   0, 0),
};
