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

/*
 * "Note that the timing on the WS2812/WS2812B LEDs has changed as of batches from WorldSemi
 * manufactured made in October 2013, and timing tolerance for approx 10-30% of parts is very small.
 * Recommendation from WorldSemi is now: 0 = 400ns high/850ns low, and 1 = 850ns high, 400ns low"
 *
 * Currently the timings are 0 = 350ns high/800ns and 1 = 700ns high/650ns low.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include "common/maths.h"

#ifdef USE_LED_STRIP

#include "build/build_config.h"

#include "common/color.h"
#include "common/colorconversion.h"

#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "io/ledstrip.h"

#include "light_ws2811strip.h"

#include "scheduler/scheduler.h"

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
#if defined(STM32F7)
FAST_DATA_ZERO_INIT uint32_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
#else
uint32_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
#endif
#endif

static ioTag_t ledStripIoTag;
static bool ws2811Initialised = false;
volatile bool ws2811LedDataTransferInProgress = false;
static unsigned usedLedCount = 0;
static bool needsFullRefresh = true;

uint16_t BIT_COMPARE_1 = 0;
uint16_t BIT_COMPARE_0 = 0;

static hsvColor_t ledColorBuffer[WS2811_DATA_BUFFER_SIZE];

#if !defined(USE_WS2811_SINGLE_COLOUR)
void setLedHsv(uint16_t index, const hsvColor_t *color)
{
    ledColorBuffer[index] = *color;
}

void getLedHsv(uint16_t index, hsvColor_t *color)
{
    *color = ledColorBuffer[index];
}

void setLedValue(uint16_t index, const uint8_t value)
{
    ledColorBuffer[index].v = value;
}

void scaleLedValue(uint16_t index, const uint8_t scalePercent)
{
    ledColorBuffer[index].v = ((uint16_t)ledColorBuffer[index].v * scalePercent / 100);
}
#endif

void setStripColor(const hsvColor_t *color)
{
    for (unsigned index = 0; index < usedLedCount; index++) {
        ledColorBuffer[index] = *color;
    }
}

void setStripColors(const hsvColor_t *colors)
{
    for (unsigned index = 0; index < usedLedCount; index++) {
        setLedHsv(index, colors++);
    }
}

void setUsedLedCount(unsigned ledCount)
{
    usedLedCount = (ledCount < WS2811_DATA_BUFFER_SIZE) ? ledCount : WS2811_DATA_BUFFER_SIZE;

     // Update all possible positions on the next update in case the count
     // decreased otherwise LEDs on the end could be left in their previous state
    needsFullRefresh = true;
}

void ws2811LedStripInit(ioTag_t ioTag)
{
    memset(ledStripDMABuffer, 0, sizeof(ledStripDMABuffer));

    ledStripIoTag = ioTag;
}

void ws2811LedStripEnable(void)
{
    if (!ws2811Initialised) {
        if (!ws2811LedStripHardwareInit(ledStripIoTag)) {
            return;
        }

        const hsvColor_t hsv_black = { 0, 0, 0 };
        setStripColor(&hsv_black);

        ws2811Initialised = true;

        // RGB or GRB ordering doesn't matter for black, use 4-channel LED configuraton to make sure all channels are zero
        // Multiple calls may be required as normally broken into multiple parts
        while (!ws2811UpdateStrip(LED_GRBW, 100));
    }
}

bool isWS2811LedStripReady(void)
{
    return ws2811Initialised && !ws2811LedDataTransferInProgress;
}

STATIC_UNIT_TESTED void updateLEDDMABuffer(ledStripFormatRGB_e ledFormat, rgbColor24bpp_t *color, unsigned ledIndex)
{
    uint32_t bits_per_led;
    uint32_t packed_colour;

    switch (ledFormat) {
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

/*
 * This method is non-blocking unless an existing LED update is in progress.
 * it does not wait until all the LEDs have been updated, that happens in the background.
 */
bool ws2811UpdateStrip(ledStripFormatRGB_e ledFormat, uint8_t brightness)
{
    static uint8_t ledIndex = 0;
    timeUs_t startTime = micros();
    // don't wait - risk of infinite block, just get an update next time round
    if (!ws2811Initialised || ws2811LedDataTransferInProgress) {
        schedulerIgnoreTaskStateTime();
        return false;
    }


    // fill transmit buffer with correct compare values to achieve
    // correct pulse widths according to color values
    const unsigned ledUpdateCount = needsFullRefresh ? WS2811_DATA_BUFFER_SIZE : usedLedCount;
    const hsvColor_t hsvBlack = { 0, 0, 0 };
    while (ledIndex < ledUpdateCount) {
        hsvColor_t scaledLed = ledIndex < usedLedCount ? ledColorBuffer[ledIndex] : hsvBlack;
        // Scale the LED brightness
        scaledLed.v = scaledLed.v * brightness / 100;

        rgbColor24bpp_t *rgb24 = hsvToRgb24(&scaledLed);

        updateLEDDMABuffer(ledFormat, rgb24, ledIndex++);

        if (cmpTimeUs(micros(), startTime) > LED_TARGET_UPDATE_US) {
            return false;
        }
    }
    ledIndex = 0;
    needsFullRefresh = false;

#ifdef USE_LED_STRIP_CACHE_MGMT
    SCB_CleanDCache_by_Addr(ledStripDMABuffer, WS2811_DMA_BUF_CACHE_ALIGN_BYTES);
#endif

    ws2811LedDataTransferInProgress = true;
    ws2811LedStripDMAEnable();

    return true;
}

#endif
