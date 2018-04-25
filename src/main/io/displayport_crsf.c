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

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#if defined(USE_CRSF_CMS_TELEMETRY)

#include "cms/cms.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/time.h"
#include "drivers/display.h"
#include "drivers/time.h"
#include "io/displayport_crsf.h"
#include "pg/pg_ids.h"

#define CRSF_DISPLAY_PORT_OPEN_DELAY_MS     400
#define CRSF_DISPLAY_PORT_CLEAR_DELAY_MS    38

static crsfDisplayPortScreen_t crsfScreen;
static timeMs_t delayTransportUntilMs = 0;

PG_REGISTER(displayPortProfile_t, displayPortProfileCrsf, PG_DISPLAY_PORT_CRSF_CONFIG, 0);

displayPort_t crsfDisplayPort;

static int crsfGrab(displayPort_t *displayPort)
{
    return displayPort->grabCount = 1;
}

static int crsfClearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    crsfDisplayPortRow_t *screenRow;
    for (int row=0; row<CRSF_DISPLAY_PORT_ROWS_MAX; row++) {
        screenRow = &crsfScreen.rows[row];
        screenRow->pendingTransport = false;
        for (int col=0; col<CRSF_DISPLAY_PORT_COLS_MAX; col++) {
            screenRow->data[col]=' ';
        }
    }
    crsfScreen.reset = true;
    delayTransportUntilMs = millis() + CRSF_DISPLAY_PORT_CLEAR_DELAY_MS;
    return 0;
}

static int crsfRelease(displayPort_t *displayPort)
{
    displayPort->grabCount = 0;
    return crsfClearScreen(displayPort);
}

static int crsfDrawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int crsfScreenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}


static int crsfWriteString(displayPort_t *displayPort, uint8_t col, uint8_t row, const char *s)
{
    UNUSED(displayPort);
    if (row >= CRSF_DISPLAY_PORT_ROWS_MAX || col >= CRSF_DISPLAY_PORT_COLS_MAX) {
        return 0;
    }
    const size_t truncLen = MIN((int)strlen(s), CRSF_DISPLAY_PORT_COLS_MAX-col);  // truncate at CRSF_DISPLAY_PORT_COLS_MAX
    crsfDisplayPortRow_t *screenRow = &crsfScreen.rows[row];
    screenRow->pendingTransport = memcmp(&screenRow->data[col], s, truncLen);
    if (screenRow->pendingTransport) {
        memcpy(&screenRow->data[col], s, truncLen);
    }
    return 0;
}

static int crsfWriteChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t c)
{
    char s[1];
    tfp_sprintf(s, "%c", c);
    return crsfWriteString(displayPort, col, row, s);
}

static bool crsfIsTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static bool crsfIsSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true;
}

static int crsfHeartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static void crsfResync(displayPort_t *displayPort)
{
    displayPort->rows = CRSF_DISPLAY_PORT_ROWS_MAX + displayPortProfileCrsf()->rowAdjust;
    displayPort->cols = CRSF_DISPLAY_PORT_COLS_MAX + displayPortProfileCrsf()->colAdjust;
    crsfClearScreen(displayPort);
}

static uint32_t crsfTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static const displayPortVTable_t crsfDisplayPortVTable = {
    .grab = crsfGrab,
    .release = crsfRelease,
    .clearScreen = crsfClearScreen,
    .drawScreen = crsfDrawScreen,
    .screenSize = crsfScreenSize,
    .writeString = crsfWriteString,
    .writeChar = crsfWriteChar,
    .isTransferInProgress = crsfIsTransferInProgress,
    .heartbeat = crsfHeartbeat,
    .resync = crsfResync,
    .isSynced = crsfIsSynced,
    .txBytesFree = crsfTxBytesFree
};

displayPort_t *displayPortCrsfInit()
{
    displayInit(&crsfDisplayPort, &crsfDisplayPortVTable);
    crsfResync(&crsfDisplayPort);
    return &crsfDisplayPort;
}

crsfDisplayPortScreen_t *crsfDisplayPortScreen(void)
{
    return &crsfScreen;
}

void crsfDisplayPortMenuOpen(void)
{
    if (cmsInMenu) {
        return;
    }
    if (cmsDisplayPortSelect(&crsfDisplayPort)) {
        cmsMenuOpen();
        delayTransportUntilMs = millis() + CRSF_DISPLAY_PORT_OPEN_DELAY_MS;
    }
}

void crsfDisplayPortMenuExit(void)
{
    if (!cmsInMenu) {
        return;
    }
    uint8_t exitMenu = CMS_EXIT;
    cmsMenuExit(&crsfDisplayPort, &exitMenu);
}


void crsfDisplayPortRefresh(void)
{
    if (!cmsInMenu) {
        crsfDisplayPortMenuOpen();
        return;
    }
    crsfDisplayPortRow_t *screenRow;
    for (int row=0; row<CRSF_DISPLAY_PORT_ROWS_MAX; row++) {
        screenRow = &crsfScreen.rows[row];
        screenRow->pendingTransport = true;
    }
    crsfScreen.reset = true;
}

int crsfDisplayPortNextRow(void)
{
    const timeMs_t currentTimeMs = millis();
    if (currentTimeMs < delayTransportUntilMs) {
        return -1;
    }
    crsfDisplayPortRow_t *screenRow;
    for(int i=0; i<CRSF_DISPLAY_PORT_ROWS_MAX; i++) {
        screenRow = &crsfScreen.rows[i];
        if (screenRow->pendingTransport) {
            return i;
        }
    }
    return -1;
}

#endif
