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

static int canvasOutput(displayPort_t *displayPort, uint8_t cmd, uint8_t *buf, int len)
{
    UNUSED(displayPort);
    mspSerialPush(cmd, buf, len);

    return 6 + len;
}

static int canvasBegin(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { 0 };

    return canvasOutput(displayPort, MSP_CANVAS, subcmd, sizeof(subcmd));
}

static int canvasHeartBeat(displayPort_t *displayPort)
{
    return canvasBegin(displayPort);
}

static int canvasEnd(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { 1 };

    return canvasOutput(displayPort, MSP_CANVAS, subcmd, sizeof(subcmd));
}

static int canvasClear(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { 2 };

    return canvasOutput(displayPort, MSP_CANVAS, subcmd, sizeof(subcmd));
}

static int canvasWrite(displayPort_t *displayPort, uint8_t col, uint8_t row, char *string)
{
    int len;
    uint8_t buf[30 + 4];

    if ((len = strlen(string)) >= 30)
        len = 30;

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

static uint32_t canvasTxBytesFree(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return mspSerialTxBytesFree();
}

static const displayPortVTable_t canvasVTable = {
    canvasBegin,
    canvasEnd,
    canvasClear,
    canvasWrite,
    canvasHeartBeat,
    canvasResync,
    canvasTxBytesFree,
};

void canvasInit()
{
    canvasDisplayPort.vTable = &canvasVTable;
    canvasDisplayPort.inCMS = false;
    canvasResync(&canvasDisplayPort);
    cmsDisplayPortRegister(&canvasDisplayPort);
}
#endif
