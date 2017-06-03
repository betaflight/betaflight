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
 Created by Dominic Clifton
  */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_OSD_SLAVE

#include "build/debug.h"
#include "build/version.h"

#include "common/printf.h"
#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/max7456_symbols.h"
#include "drivers/time.h"

#include "io/osd_slave.h"

//#define OSD_SLAVE_DEBUG

// when locked the system ignores requests to enter cli or bootloader mode via serial connection.
bool osdSlaveIsLocked = false;

static displayPort_t *osdDisplayPort;

static void osdDrawLogo(int x, int y)
{
    char fontOffset = 160;
    for (int row = 0; row < 4; row++) {
        for (int column = 0; column < 24; column++) {
            if (fontOffset != 255) // FIXME magic number
                displayWriteChar(osdDisplayPort, x + column, y + row, fontOffset++);
        }
    }
}

bool displayDrawScreenQueued = false;
bool receivingScreen = false;
bool stalled = false;

void osdSlaveDrawScreen(void)
{
    displayDrawScreenQueued = true;
}

static uint32_t timeoutAt = 0;

void osdSlaveClearScreen(void)
{
    displayClearScreen(osdDisplayPort);
    receivingScreen = true;
}

void osdSlaveWriteChar(const uint8_t x, const uint8_t y, const uint8_t c)
{
    displayWriteChar(osdDisplayPort, x, y, c);
}

void osdSlaveWrite(const uint8_t x, const uint8_t y, const char *s)
{
    displayWrite(osdDisplayPort, x, y, s);
}

void osdSlaveHeartbeat(void)
{
    timeoutAt = micros() + (1000 * 1000);
    stalled = false;
}

void osdSlaveInit(displayPort_t *osdDisplayPortToUse)
{
    if (!osdDisplayPortToUse)
        return;

    osdDisplayPort = osdDisplayPortToUse;

    displayClearScreen(osdDisplayPort);

    osdDrawLogo(3, 1);

    char string_buffer[30];
    tfp_sprintf(string_buffer, "V%s", FC_VERSION_STRING);
    displayWrite(osdDisplayPort, 20, 6, string_buffer);
    displayWrite(osdDisplayPort, 13, 6, "OSD");

    displayResync(osdDisplayPort);

    displayDrawScreenQueued = true;
}

bool osdSlaveCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);
    UNUSED(currentDeltaTimeUs);

    if (!stalled && (cmp32(currentTimeUs, timeoutAt) > 0)) {
        stalled = true;

        displayWrite(osdDisplayPort, 8, 12, "WAITING FOR FC");
        displayResync(osdDisplayPort);
    }

    return receivingScreen || displayDrawScreenQueued || stalled;
}

/*
 * Called periodically by the scheduler
 */
void osdSlaveUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

#ifdef MAX7456_DMA_CHANNEL_TX
    // don't touch buffers if DMA transaction is in progress
    if (displayIsTransferInProgress(osdDisplayPort)) {
        return;
    }
#endif // MAX7456_DMA_CHANNEL_TX

#ifdef OSD_SLAVE_DEBUG
    char buff[32];
    for (int i = 0; i < 4; i ++) {
        tfp_sprintf(buff, "%5d", debug[i]);
        displayWrite(osdDisplayPort, i * 8, 0, buff);
    }
#endif

    if (displayDrawScreenQueued || stalled) {
        displayDrawScreen(osdDisplayPort);
        displayDrawScreenQueued = false;
        receivingScreen = false;
    }
}
#endif // OSD_SLAVE
