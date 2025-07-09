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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include "common/maths.h"

#include "common/color.h"
#include "drivers/light_ws2811strip.h"
#include "platform/light_ws2811strip.h"

uint16_t BIT_COMPARE_1 = 0;
uint16_t BIT_COMPARE_0 = 0;

#ifdef USE_LED_STRIP_CACHE_MGMT
// WS2811_DMA_BUFFER_SIZE is multiples of uint32_t
// Number of bytes required for buffer
#define WS2811_DMA_BUF_BYTES              (WS2811_DMA_BUFFER_SIZE * sizeof(uint32_t))
// Number of bytes required to cache align buffer
#define WS2811_DMA_BUF_CACHE_ALIGN_BYTES  ((WS2811_DMA_BUF_BYTES + 0x20) & ~0x1f)
// Size of array to create a cache aligned buffer
#define WS2811_DMA_BUF_CACHE_ALIGN_LENGTH (WS2811_DMA_BUF_CACHE_ALIGN_BYTES / sizeof(uint32_t))
DMA_RW_AXI __attribute__((aligned(32))) uint32_t ledStripDMABuffer[WS2811_DMA_BUF_CACHE_ALIGN_LENGTH];
#else
DMA_DATA_ZERO_INIT uint32_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
#endif

ioTag_t ledStripIoTag;
static ledStripFormatRGB_e ws2811LedFormat;

void ws2811LedStripInit(ioTag_t ioTag, ledStripFormatRGB_e ledFormat)
{
    ws2811LedFormat = ledFormat;
    memset(ledStripDMABuffer, 0, sizeof(ledStripDMABuffer));
    ledStripIoTag = ioTag;
}

void ws2811LedStripUpdateTransferBuffer(rgbColor24bpp_t *color, unsigned ledIndex)
{
    uint32_t bits_per_led;
    uint32_t packed_colour;

    switch (ws2811LedFormat) {
        case LED_RGB: // WS2811 drivers use RGB format
            packed_colour = (color->rgb.r << 16) | (color->rgb.g << 8) | (color->rgb.b);
            bits_per_led = 24;
            break;

        case LED_GRBW: // SK6812 drivers use this
        {
            /* reconstruct white channel from RGB, making the intensity a bit nonlinear, but thats fine for this use case */
            uint8_t white = MIN(MIN(color->rgb.r, color->rgb.g), color->rgb.b);
            packed_colour = (color->rgb.g << 24) | (color->rgb.r << 16) | (color->rgb.b << 8) | (white);
            bits_per_led = 32;
            break;
        }

        case LED_GRB: // WS2812 drivers use GRB format
        default:
            packed_colour = (color->rgb.g << 16) | (color->rgb.r << 8) | (color->rgb.b);
            bits_per_led = 24;
        break;
    }

    unsigned dmaBufferOffset = 0;
    for (int index = bits_per_led - 1; index >= 0; index--) {
        ledStripDMABuffer[ledIndex * bits_per_led + dmaBufferOffset++] = (packed_colour & (1U << index)) ? BIT_COMPARE_1 : BIT_COMPARE_0;
    }
}
