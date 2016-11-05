#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef OSD

#include "common/utils.h"

#include "config/config_master.h"

#include "drivers/display.h"
#include "drivers/max7456.h"

displayPort_t osd7456DisplayPort;

extern uint16_t refreshTimeout;
void osdResetAlarms(void);

static int osdMenuBegin(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    osdResetAlarms();
    displayPort->inCMS = true;
    refreshTimeout = 0;

    return 0;
}

static int osdMenuEnd(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    displayPort->inCMS = false;

    return 0;
}

static int osdClearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    max7456ClearScreen();

    return 0;
}

static int osdWrite(displayPort_t *displayPort, uint8_t x, uint8_t y, char *s)
{
    UNUSED(displayPort);
    max7456Write(x, y, s);

    return 0;
}

static void osdResync(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    max7456RefreshAll();
    displayPort->rows = max7456GetRowsCount();
    displayPort->cols = 30;
}

static int osdHeartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static uint32_t osdTxBytesFree(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

displayPortVTable_t osdVTable = {
    osdMenuBegin,
    osdMenuEnd,
    osdClearScreen,
    osdWrite,
    osdHeartbeat,
    osdResync,
    osdTxBytesFree,
};

void osd7456DisplayPortInit(void)
{
    osd7456DisplayPort.vTable = &osdVTable;
    osd7456DisplayPort.inCMS = false;
    osdResync(&osd7456DisplayPort);
    cmsDisplayPortRegister(&osd7456DisplayPort);
}
#endif // OSD
