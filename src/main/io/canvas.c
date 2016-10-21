#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#ifdef CANVAS

#include "io/cms_types.h"

#include "fc/fc_msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

void canvasGetSize(uint8_t *pRows, uint8_t *pCols)
{
    *pRows = 13;
    *pCols = 30;
}

void canvasBegin(void)
{
    uint8_t subcmd[] = { 0 };

    mspSerialPush(MSP_CANVAS, subcmd, sizeof(subcmd));
}

void canvasHeartBeat(void)
{
    canvasBegin();
}

void canvasEnd(void)
{
    uint8_t subcmd[] = { 1 };

    mspSerialPush(MSP_CANVAS, subcmd, sizeof(subcmd));
}

void canvasClear(void)
{
    uint8_t subcmd[] = { 2 };

    mspSerialPush(MSP_CANVAS, subcmd, sizeof(subcmd));
}

void canvasWrite(uint8_t col, uint8_t row, char *string)
{

//debug[0]++; // Let's capture excess canvas writes

    int len;
    char buf[30 + 4];

    if ((len = strlen(string)) >= 30)
        len = 30;

    buf[0] = 3;
    buf[1] = row;
    buf[2] = col;
    buf[3] = 0;
    memcpy((char *)&buf[4], string, len);

    mspSerialPush(MSP_CANVAS, (uint8_t *)buf, len + 4);
}

screenFnVTable_t canvasVTable = {
    canvasGetSize,
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
