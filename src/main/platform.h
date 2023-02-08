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

#pragma once

#define NOINLINE __attribute__((noinline))

#if !defined(UNIT_TEST) && !defined(SIMULATOR_BUILD) && !(USBD_DEBUG_LEVEL > 0)
#pragma GCC poison sprintf snprintf
#endif

// common to all
#define USE_TIMER_AF

#if defined(STM32)

#include "drivers/stm32/platform_stm32.h"

#elif defined(AT32)

#include "drivers/at32/platform_at32.h"

#elif defined(SIMULATOR_BUILD)

// Nop
#undef USE_TIMER_AF
#define DEFAULT_CPU_OVERCLOCK 1
#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW

#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO
#else
#error "Invalid chipset specified. Update platform.h"
#endif

#include "target/common_pre.h"

#ifdef __BOARD__
#include "board.h"
#endif

#ifdef USE_CUSTOM_TARGET
#include <custom_target.h>
#endif

#include "target.h"
#include "target/common_post.h"
#include "target/common_defaults_post.h"
