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
 * "Note that the timing on the WS2812/WS2812B LEDs has changed as of batches from WorldSemi
 * manufactured made in October 2013, and timing tolerance for approx 10-30% of parts is very small.
 * Recommendation from WorldSemi is now: 0 = 400ns high/850ns low, and 1 = 850ns high, 400ns low"
 *
 * Currently the timings are 0 = 350ns high/800ns and 1 = 700ns high/650ns low.
 */
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/light_ws2811strip.h"

uint8_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
volatile uint8_t ws2811LedDataTransferInProgress = 0;

static rgbColor24bpp_t ledColorBuffer[WS2811_LED_STRIP_LENGTH];

void setLedColor(uint16_t index, const rgbColor24bpp_t *color)
{
    ledColorBuffer[index].rgb = color->rgb;
}

/**
 * use this after you set the color
 */
void setLedBrightness(uint16_t index, const uint8_t scalePercent)
{
    ledColorBuffer[index].rgb.r = ((uint16_t)ledColorBuffer[index].rgb.r * scalePercent / 100);
    ledColorBuffer[index].rgb.g = ((uint16_t)ledColorBuffer[index].rgb.g * scalePercent / 100);
    ledColorBuffer[index].rgb.b = ((uint16_t)ledColorBuffer[index].rgb.b * scalePercent / 100);
}

void setStripColor(const rgbColor24bpp_t *color)
{
    uint16_t index;
    for (index = 0; index < WS2811_LED_STRIP_LENGTH; index++) {
        setLedColor(index, color);
    }
}

void setStripColors(const rgbColor24bpp_t *colors)
{
    uint16_t index;
    for (index = 0; index < WS2811_LED_STRIP_LENGTH; index++) {
        setLedColor(index, colors++);
    }
}

void ws2811LedStripInit(void)
{
    ws2811LedStripHardwareInit();

    setStripColor(&white);
    ws2811UpdateStrip();
}

bool isWS2811LedStripReady(void)
{
    return !ws2811LedDataTransferInProgress;
}

static uint16_t dmaBufferOffset;
static int16_t ledIndex;

void updateLEDDMABuffer(uint8_t componentValue)
{
    uint8_t bitIndex;
    
    for (bitIndex = 0; bitIndex < 8; bitIndex++)
    {
        if ((componentValue << bitIndex) & 0x80 )    // data sent MSB first, j = 0 is MSB j = 7 is LSB
        {
            ledStripDMABuffer[dmaBufferOffset] = 17;  // compare value for logical 1
        }
        else
        {
            ledStripDMABuffer[dmaBufferOffset] = 9;   // compare value for logical 0
        }
        dmaBufferOffset++;
    }
}

/*
 * This method is non-blocking unless an existing LED update is in progress.
 * it does not wait until all the LEDs have been updated, that happens in the background.
 */
void ws2811UpdateStrip(void)
{
    static uint32_t waitCounter = 0;
    // wait until previous transfer completes
    while(ws2811LedDataTransferInProgress) {
        waitCounter++;
    }

    dmaBufferOffset = 0;                // reset buffer memory index
    ledIndex = 0;                    // reset led index

    // fill transmit buffer with correct compare values to achieve
    // correct pulse widths according to color values
    while (ledIndex < WS2811_LED_STRIP_LENGTH)
    {
        updateLEDDMABuffer(ledColorBuffer[ledIndex].rgb.g);
        updateLEDDMABuffer(ledColorBuffer[ledIndex].rgb.r);
        updateLEDDMABuffer(ledColorBuffer[ledIndex].rgb.b);

        ledIndex++;
    }

    // add needed delay at end of byte cycle, pulsewidth = 0
    while(dmaBufferOffset < WS2811_DMA_BUFFER_SIZE)
    {
        ledStripDMABuffer[dmaBufferOffset] = 0;
        dmaBufferOffset++;
    }

    ws2811LedDataTransferInProgress = 1;
    ws2811LedStripDMAEnable();
}

