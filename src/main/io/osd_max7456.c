#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef OSD

#include "common/utils.h"

#include "config/config_master.h"

#include "drivers/display.h"
#include "drivers/max7456.h"

extern bool osdInMenu;
extern uint16_t refreshTimeout;
void osdResetAlarms(void);

uint8_t shiftdown;

static int osdMenuBegin(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    osdResetAlarms();
    osdInMenu = true;
    refreshTimeout = 0;

    return 0;
}

static int osdMenuEnd(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    osdInMenu = false;

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
    max7456Write(x, y + shiftdown, s);

    return 0;
}

static void osdResync(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    max7456RefreshAll();
    displayPort->rows = max7456GetRowsCount() - masterConfig.osdProfile.row_shiftdown;
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

void osdMax7456Init(displayPort_t *displayPort)
{
    displayPort->vTable = &osdVTable;
    osdResync(displayPort);
}
#endif // OSD
