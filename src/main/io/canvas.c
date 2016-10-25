#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#ifdef CANVAS

#include "drivers/system.h"

#include "io/cms_types.h"

#include "fc/fc_msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

void canvasGetDevParam(uint8_t *pRows, uint8_t *pCols, uint16_t *pBuftime, uint16_t *pBufsize)
{
    *pRows = 13;
    *pCols = 30;
    *pBuftime = 23;     // = 256/(115200/10)
    *pBufsize = 192;    // 256 * 3/4 (Be conservative)
}

int canvasOutput(uint8_t cmd, uint8_t *buf, int len)
{
    mspSerialPush(cmd, buf, len);

    return 6 + len;
}

int canvasBegin(void)
{
    uint8_t subcmd[] = { 0 };

    return canvasOutput(MSP_CANVAS, subcmd, sizeof(subcmd));
}

int canvasHeartBeat(void)
{
    return canvasBegin();
}

int canvasEnd(void)
{
    uint8_t subcmd[] = { 1 };

    return canvasOutput(MSP_CANVAS, subcmd, sizeof(subcmd));
}

int canvasClear(void)
{
    uint8_t subcmd[] = { 2 };

    return canvasOutput(MSP_CANVAS, subcmd, sizeof(subcmd));
}

int canvasWrite(uint8_t col, uint8_t row, char *string)
{
    int len;
    char buf[30 + 4];

    if ((len = strlen(string)) >= 30)
        len = 30;

    buf[0] = 3;
    buf[1] = row;
    buf[2] = col;
    buf[3] = 0;
    memcpy((char *)&buf[4], string, len);

    return canvasOutput(MSP_CANVAS, (uint8_t *)buf, len + 4);
}

screenFnVTable_t canvasVTable = {
    canvasGetDevParam,
    canvasBegin,
    canvasEnd,
    canvasClear,
    canvasWrite,
    canvasHeartBeat,
    NULL,
};

screenFnVTable_t *canvasInit(void)
{
    mspSerialPushInit(mspFcPushInit()); // Called once at startup to initialize push function in msp

    return &canvasVTable;
}
#endif
