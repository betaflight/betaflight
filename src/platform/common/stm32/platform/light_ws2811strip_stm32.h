/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once

#include "platform.h"

#include "drivers/io_types.h"
#include "drivers/light_ws2811strip.h"

#define WS2811_LED_STRIP_LENGTH    LED_STRIP_MAX_LENGTH

#define WS2811_BITS_PER_LED_MAX    32
#define WS2811_TIMER_MHZ           48

#if defined(USE_WS2811_SINGLE_COLOUR)
#define WS2811_DMA_BUFFER_SIZE     (WS2811_DATA_BUFFER_SIZE * WS2811_BITS_PER_LED_MAX)
// Do 2 extra iterations of the DMA transfer with the output set to low to generate the > 50us delay.
#define WS2811_DELAY_ITERATIONS    2
#else
// for 50us delay
#define WS2811_DELAY_BUFFER_LENGTH 42
// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes)
#define WS2811_DMA_BUFFER_SIZE     (WS2811_DATA_BUFFER_SIZE * WS2811_BITS_PER_LED_MAX + WS2811_DELAY_BUFFER_LENGTH)
#endif

#ifdef USE_LED_STRIP_CACHE_MGMT
// WS2811_DMA_BUFFER_SIZE is multiples of uint32_t
// Number of bytes required for buffer
#define WS2811_DMA_BUF_BYTES              (WS2811_DMA_BUFFER_SIZE * sizeof(uint32_t))
// Number of bytes required to cache align buffer
#define WS2811_DMA_BUF_CACHE_ALIGN_BYTES  ((WS2811_DMA_BUF_BYTES + 0x20) & ~0x1f)
// Size of array to create a cache aligned buffer
#define WS2811_DMA_BUF_CACHE_ALIGN_LENGTH (WS2811_DMA_BUF_CACHE_ALIGN_BYTES / sizeof(uint32_t))
extern uint32_t ledStripDMABuffer[WS2811_DMA_BUF_CACHE_ALIGN_LENGTH];
#else
extern uint32_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
#endif

extern uint16_t BIT_COMPARE_1;
extern uint16_t BIT_COMPARE_0;
extern ioTag_t ledStripIoTag;
