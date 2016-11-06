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

#ifdef CANVAS

#include "build/version.h"

#include "common/utils.h"

#include "drivers/system.h"

#include "io/cms.h"

#include "fc/fc_msp.h"

#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

static displayPort_t canvasDisplayPort;

static int canvasOutput(displayPort_t *displayPort, uint8_t cmd, const uint8_t *buf, int len)
{
    UNUSED(displayPort);
    return mspSerialPush(cmd, buf, len);
}

static int canvasOpen(displayPort_t *displayPort)
{
    const uint8_t subcmd[] = { 0 };

    return canvasOutput(displayPort, MSP_CANVAS, subcmd, sizeof(subcmd));
}

static int canvasHeartBeat(displayPort_t *displayPort)
{
    return canvasOpen(displayPort);
}

static int canvasClose(displayPort_t *displayPort)
{
    const uint8_t subcmd[] = { 1 };

    return canvasOutput(displayPort, MSP_CANVAS, subcmd, sizeof(subcmd));
}

static int canvasClear(displayPort_t *displayPort)
{
    const uint8_t subcmd[] = { 2 };

    return canvasOutput(displayPort, MSP_CANVAS, subcmd, sizeof(subcmd));
}

static int canvasWrite(displayPort_t *displayPort, uint8_t col, uint8_t row, const char *string)
{
#define MSP_CANVAS_MAX_STRING_LENGTH 30
    uint8_t buf[MSP_CANVAS_MAX_STRING_LENGTH + 4];

    int len = strlen(string);
    if (len >= MSP_CANVAS_MAX_STRING_LENGTH) {
        len = MSP_CANVAS_MAX_STRING_LENGTH;
    }

    buf[0] = 3;
    buf[1] = row;
    buf[2] = col;
    buf[3] = 0;
    memcpy(&buf[4], string, len);

    return canvasOutput(displayPort, MSP_CANVAS, buf, len + 4);
}

static void canvasResync(displayPort_t *displayPort)
{
    displayPort->rows = 13; // XXX Will reflect NTSC/PAL in the future
    displayPort->cols = 30;
}

static uint32_t canvasTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return mspSerialTxBytesFree();
}

static const displayPortVTable_t canvasVTable = {
    .open = canvasOpen,
    .close = canvasClose,
    .clear = canvasClear,
    .write = canvasWrite,
    .heartbeat = canvasHeartBeat,
    .resync = canvasResync,
    .txBytesFree = canvasTxBytesFree
};

displayPort_t *canvasInit(void)
{
    canvasDisplayPort.vTable = &canvasVTable;
    canvasDisplayPort.isOpen = false;
    canvasResync(&canvasDisplayPort);
    return &canvasDisplayPort;
}
#endif
