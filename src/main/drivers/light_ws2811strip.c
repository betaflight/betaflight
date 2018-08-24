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

#ifdef USE_LED_STRIP

#include "build/build_config.h"

#include "common/color.h"
#include "common/colorconversion.h"

#include "drivers/dma.h"
#include "drivers/io.h"

#include "light_ws2811strip.h"

#if defined(STM32F1) || defined(STM32F3)
uint8_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
#elif defined(STM32F7)
FAST_RAM_ZERO_INIT uint32_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
#else
uint32_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
#endif

volatile uint8_t ws2811LedDataTransferInProgress = 0;

uint16_t BIT_COMPARE_1 = 0;
uint16_t BIT_COMPARE_0 = 0;

static hsvColor_t ledColorBuffer[WS2811_LED_STRIP_LENGTH];

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

void setStripColor(const hsvColor_t *color)
{
    uint16_t index;
    for (index = 0; index < WS2811_LED_STRIP_LENGTH; index++) {
        setLedHsv(index, color);
    }
}

void setStripColors(const hsvColor_t *colors)
{
    uint16_t index;
    for (index = 0; index < WS2811_LED_STRIP_LENGTH; index++) {
        setLedHsv(index, colors++);
    }
}

void ws2811LedStripInit(ioTag_t ioTag)
{
    memset(ledStripDMABuffer, 0, sizeof(ledStripDMABuffer));
    ws2811LedStripHardwareInit(ioTag);

    const hsvColor_t hsv_white = { 0, 255, 255 };
    setStripColor(&hsv_white);
    // RGB or GRB ordering doesn't matter for white
    ws2811UpdateStrip(LED_RGB);
}

bool isWS2811LedStripReady(void)
{
    return !ws2811LedDataTransferInProgress;
}

STATIC_UNIT_TESTED uint16_t dmaBufferOffset;
static int16_t ledIndex;

#define USE_FAST_DMA_BUFFER_IMPL
#ifdef USE_FAST_DMA_BUFFER_IMPL

STATIC_UNIT_TESTED void fastUpdateLEDDMABuffer(ledStripFormatRGB_e ledFormat, rgbColor24bpp_t *color)
{
    uint32_t packed_colour;

    switch (ledFormat) {
        case LED_RGB: // WS2811 drivers use RGB format
            packed_colour = (color->rgb.r << 16) | (color->rgb.g << 8) | (color->rgb.b);
            break;

        case LED_GRB: // WS2812 drivers use GRB format
        default:
            packed_colour = (color->rgb.g << 16) | (color->rgb.r << 8) | (color->rgb.b);
        break;
    }

    for (int8_t index = 23; index >= 0; index--) {
        ledStripDMABuffer[dmaBufferOffset++] = (packed_colour & (1 << index)) ? BIT_COMPARE_1 : BIT_COMPARE_0;
    }
}
#else
STATIC_UNIT_TESTED void updateLEDDMABuffer(uint8_t componentValue)
{
    uint8_t bitIndex;

    for (bitIndex = 0; bitIndex < 8; bitIndex++)
    {
        if ((componentValue << bitIndex) & 0x80 )    // data sent MSB first, j = 0 is MSB j = 7 is LSB
        {
            ledStripDMABuffer[dmaBufferOffset] = BIT_COMPARE_1;
        }
        else
        {
            ledStripDMABuffer[dmaBufferOffset] = BIT_COMPARE_0;   // compare value for logical 0
        }
        dmaBufferOffset++;
    }
}
#endif

/*
 * This method is non-blocking unless an existing LED update is in progress.
 * it does not wait until all the LEDs have been updated, that happens in the background.
 */
void ws2811UpdateStrip(ledStripFormatRGB_e ledFormat)
{
    static rgbColor24bpp_t *rgb24;

    // don't wait - risk of infinite block, just get an update next time round
    if (ws2811LedDataTransferInProgress) {
        return;
    }

    dmaBufferOffset = 0;                // reset buffer memory index
    ledIndex = 0;                       // reset led index

    // fill transmit buffer with correct compare values to achieve
    // correct pulse widths according to color values
    while (ledIndex < WS2811_LED_STRIP_LENGTH)
    {
        rgb24 = hsvToRgb24(&ledColorBuffer[ledIndex]);

#ifdef USE_FAST_DMA_BUFFER_IMPL
        fastUpdateLEDDMABuffer(ledFormat, rgb24);
#else
        switch (ledFormat) {
            case LED_RGB: // WS2811 drivers use RGB format
            updateLEDDMABuffer(rgb24->rgb.r);
            updateLEDDMABuffer(rgb24->rgb.g);
            break;

        case LED_GRB: // WS2812 drivers use GRB format
        default:
            updateLEDDMABuffer(rgb24->rgb.g);
            updateLEDDMABuffer(rgb24->rgb.r);
            break;
        }

        updateLEDDMABuffer(rgb24->rgb.b);
#endif

        ledIndex++;
    }

    ws2811LedDataTransferInProgress = 1;
    ws2811LedStripDMAEnable();
}

#endif
