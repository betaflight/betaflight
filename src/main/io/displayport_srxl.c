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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#if defined (USE_SPEKTRUM_CMS_TELEMETRY) && defined (USE_CMS)

#include "common/utils.h"

#include "drivers/display.h"
#include "cms/cms.h"

#include "telemetry/srxl.h"

displayPort_t srxlDisplayPort;

static int srxlDrawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int srxlScreenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

static int srxlWriteChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t c)
{
    return (spektrumTmTextGenPutChar(col, row, c));
    UNUSED(displayPort);
}


static int srxlWriteString(displayPort_t *displayPort, uint8_t col, uint8_t row, const char *s)
{
    while (*s) {
        srxlWriteChar(displayPort,  col++, row, *(s++));
    }
    return 0;
}

static int srxlClearScreen(displayPort_t *displayPort)
{
    for (int row = 0;  row < SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS; row++) {
        for (int col= 0; col < SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS; col++) {
            srxlWriteChar(displayPort, col, row, ' ');
        }
    }
    srxlWriteString(displayPort, 1, 0,  "BETAFLIGHT");

    if ( displayPort->grabCount == 0 ) {
        srxlWriteString(displayPort, 0, 2,  CMS_STARTUP_HELP_TEXT1);
        srxlWriteString(displayPort, 2, 3,  CMS_STARTUP_HELP_TEXT2);
        srxlWriteString(displayPort, 2, 4,  CMS_STARTUP_HELP_TEXT3);
    }
    return 0;
}

static bool srxlIsTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static int srxlHeartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static void srxlResync(displayPort_t *displayPort)
{
    UNUSED(displayPort);
}

static uint32_t srxlTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static int srxlGrab(displayPort_t *displayPort)
{
    return displayPort->grabCount = 1;
}

static int srxlRelease(displayPort_t *displayPort)
{
    int cnt = displayPort->grabCount = 0;
    srxlClearScreen(displayPort);
    return cnt;
}

static const displayPortVTable_t srxlVTable = {
    .grab = srxlGrab,
    .release = srxlRelease,
    .clearScreen = srxlClearScreen,
    .drawScreen = srxlDrawScreen,
    .screenSize = srxlScreenSize,
    .writeString = srxlWriteString,
    .writeChar = srxlWriteChar,
    .isTransferInProgress = srxlIsTransferInProgress,
    .heartbeat = srxlHeartbeat,
    .resync = srxlResync,
    .txBytesFree = srxlTxBytesFree
};

displayPort_t *displayPortSrxlInit()
{
    srxlDisplayPort.device = NULL;
    displayInit(&srxlDisplayPort, &srxlVTable);
    srxlDisplayPort.rows = SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS;
    srxlDisplayPort.cols = SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS;
    return &srxlDisplayPort;
}

#endif
