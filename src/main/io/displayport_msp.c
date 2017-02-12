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
#include <ctype.h>

#include "platform.h"

#ifdef USE_MSP_DISPLAYPORT

#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/system.h"

#include "fc/fc_msp.h"

#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

static displayPort_t mspDisplayPort;

static displayPortProfile_t *mspDisplayPortProfile;

static int output(displayPort_t *displayPort, uint8_t cmd, const uint8_t *buf, int len)
{
    UNUSED(displayPort);
    return mspSerialPush(cmd, buf, len);
}

static int heartbeat(displayPort_t *displayPort)
{
    const uint8_t subcmd[] = { 0 };

    // ensure display is not released by MW OSD software
    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static int grab(displayPort_t *displayPort)
{
    return heartbeat(displayPort);
}

static int release(displayPort_t *displayPort)
{
    const uint8_t subcmd[] = { 1 };

    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static int clearScreen(displayPort_t *displayPort)
{
    const uint8_t subcmd[] = { 2 };

    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static int drawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int screenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

static int write(displayPort_t *displayPort, uint8_t col, uint8_t row, const char *string)
{
#define MSP_OSD_MAX_STRING_LENGTH 30
    uint8_t buf[MSP_OSD_MAX_STRING_LENGTH + 4];

    int len = strlen(string);
    if (len >= MSP_OSD_MAX_STRING_LENGTH) {
        len = MSP_OSD_MAX_STRING_LENGTH;
    }

    buf[0] = 3;
    buf[1] = row;
    buf[2] = col;
    buf[3] = 0;
    memcpy(&buf[4], string, len);

    return output(displayPort, MSP_DISPLAYPORT, buf, len + 4);
}

static int writeChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t c)
{
    char buf[2];

    buf[0] = c;
    buf[1] = 0;
    return write(displayPort, col, row, buf); //!!TODO - check if there is a direct MSP command to do this
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static void resync(displayPort_t *displayPort)
{
    displayPort->rows = 13 + mspDisplayPortProfile->rowAdjust; // XXX Will reflect NTSC/PAL in the future
    displayPort->cols = 30 + mspDisplayPortProfile->colAdjust;
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return mspSerialTxBytesFree();
}

static const displayPortVTable_t mspDisplayPortVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .write = write,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .resync = resync,
    .txBytesFree = txBytesFree
};

displayPort_t *displayPortMspInit(displayPortProfile_t *displayPortProfileToUse)
{
    mspDisplayPortProfile = displayPortProfileToUse;
    displayInit(&mspDisplayPort, &mspDisplayPortVTable);
    resync(&mspDisplayPort);
    return &mspDisplayPort;
}
#endif // USE_MSP_DISPLAYPORT
