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

// HPMicro timer helpers: per-SoC clock lookup and PWM compare register access.

#pragma once

#include "platform.h"
#include "drivers/timer.h"

// timerRCC is implemented per-SoC (target/HPM6360/timer_hpm6360.c,
// target/HPM6750/timer_hpm6750.c) and maps a timer peripheral to its clock.
rccPeriphTag_t timerRCC(TIM_TypeDef *tim);

uint32_t getPWMFre(TIM_TypeDef *timHw);

// Returns a pointer to the PWM compare register for a timer channel.
// Implemented per-SoC (target/HPM6360/timer_hpm6360.c,
// target/HPM6750/timer_hpm6750.c). timCCR_t is uint32_t on HPMicro.
volatile uint32_t *timerChCCR(const timerHardware_t *timHw);
