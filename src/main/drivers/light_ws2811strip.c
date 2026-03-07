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

static bool ws2811Initialised = false;
volatile bool ws2811LedDataTransferInProgress = false;
static unsigned usedLedCount = 0;
static bool needsFullRefresh = true;

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

void ws2811LedStripEnable(void)
{
    if (!ws2811Initialised) {
        if (!ws2811LedStripHardwareInit()) {
            return;
        }

        const hsvColor_t hsv_black = { 0, 0, 0 };
        setStripColor(&hsv_black);

        ws2811Initialised = true;

        // RGB or GRB ordering doesn't matter for black, use 4-channel LED configuraton to make sure all channels are zero
        // Multiple calls may be required as normally broken into multiple parts
        while (!ws2811UpdateStrip(100));
    }
}

bool isWS2811LedStripReady(void)
{
    return ws2811Initialised && !ws2811LedDataTransferInProgress;
}

/*
 * This method is non-blocking unless an existing LED update is in progress.
 * it does not wait until all the LEDs have been updated, that happens in the background.
 */
bool ws2811UpdateStrip(uint8_t brightness)
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

        ws2811LedStripUpdateTransferBuffer(rgb24, ledIndex++);

        if (cmpTimeUs(micros(), startTime) > LED_TARGET_UPDATE_US) {
            return false;
        }
    }
    ledIndex = 0;
    needsFullRefresh = false;

    ws2811LedDataTransferInProgress = true;
    ws2811LedStripStartTransfer();

    return true;
}

#endif
