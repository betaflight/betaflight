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
#include <string.h>

#include <platform.h>

#include "build/build_config.h"

#include "common/color.h"
#include "common/colorconversion.h"
#include "drivers/dma.h"
#include "drivers/nvic.h"
#include "drivers/light_ws2811strip.h"

uint8_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
volatile uint8_t ws2811LedDataTransferInProgress = 0;

static hsvColor_t ledColorBuffer[WS2811_LED_STRIP_LENGTH];
static dmaCallbackHandler_t ws2811DMAHandlerRec;

void setLedHsv(int index, const hsvColor_t *color)
{
    ledColorBuffer[index] = *color;
}

void getLedHsv(int index, hsvColor_t *color)
{
    *color = ledColorBuffer[index];
}

void setLedValue(int index, const uint8_t value)
{
    ledColorBuffer[index].v = value;
}

void scaleLedValue(int index, const uint8_t scalePercent)
{
    ledColorBuffer[index].v = (ledColorBuffer[index].v * scalePercent / 100);
}

void setStripColor(const hsvColor_t *color)
{
    for (int index = 0; index < WS2811_LED_STRIP_LENGTH; index++)
        setLedHsv(index, color);
}

void setStripColors(const hsvColor_t *colors)
{
    uint16_t index;
    for (index = 0; index < WS2811_LED_STRIP_LENGTH; index++) {
        setLedHsv(index, colors++);
    }
}

void ws2811DMAHandler(dmaChannel_t* descriptor, dmaCallbackHandler_t* handler)
{
    UNUSED(handler);

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(descriptor->channel, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

void ws2811LedStripInit(void)
{
    memset(&ledStripDMABuffer, 0, WS2811_DMA_BUFFER_SIZE);
    dmaHandlerInit(&ws2811DMAHandlerRec, ws2811DMAHandler);
    dmaSetHandler(WS2811_DMA_HANDLER_IDENTIFER, &ws2811DMAHandlerRec, NVIC_PRIO_WS2811_DMA);
    ws2811LedStripHardwareInit();
    ws2811UpdateStrip();
}

bool isWS2811LedStripReady(void)
{
    return !ws2811LedDataTransferInProgress;
}

STATIC_UNIT_TESTED void fastUpdateLEDDMABuffer(uint8_t **buffer, rgbColor24bpp_t color)
{
    uint32_t grb = (color.rgb.g << 16) | (color.rgb.r << 8) | (color.rgb.b);
//    uint32_t grb = (color.rgb.r << 16) | (color.rgb.g << 8) | (color.rgb.b);
    for (int bit = 0; bit < 24; bit++) {
        *(*buffer)++ = (grb & (1 << 23)) ? BIT_COMPARE_1 : BIT_COMPARE_0;
        grb <<= 1;
    }
}

/*
 * This method is non-blocking unless an existing LED update is in progress.
 * it does not wait until all the LEDs have been updated, that happens in the background.
 */
void ws2811UpdateStrip(void)
{
    // wait until previous transfer completes
    while(ws2811LedDataTransferInProgress);

    uint8_t *dst = ledStripDMABuffer;               // reset buffer memory index

    // fill transmit buffer with correct compare values to achieve
    // correct pulse widths according to color values
    for (int ledIndex = 0; ledIndex < WS2811_LED_STRIP_LENGTH; ledIndex++) {
        rgbColor24bpp_t rgb24 = hsvToRgb24(&ledColorBuffer[ledIndex]);

        fastUpdateLEDDMABuffer(&dst, rgb24);
    }

    ws2811LedDataTransferInProgress = 1;
    ws2811LedStripDMAEnable();
}

