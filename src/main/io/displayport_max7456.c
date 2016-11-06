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

#include "platform.h"

#ifdef OSD

#include "common/utils.h"

#include "config/config_master.h"

#include "drivers/display.h"
#include "drivers/max7456.h"

displayPort_t max7456DisplayPort; // Referenced from osd.c

extern uint16_t refreshTimeout;

static int grab(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    osdResetAlarms();
    displayPort->isGrabbed = true;
    refreshTimeout = 0;

    return 0;
}

static int release(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    displayPort->isGrabbed = false;

    return 0;
}

static int clearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    max7456ClearScreen();

    return 0;
}

static int write(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *s)
{
    UNUSED(displayPort);
    max7456Write(x, y, s);

    return 0;
}

static void resync(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    max7456RefreshAll();
    displayPort->rows = max7456GetRowsCount();
    displayPort->cols = 30;
}

static int heartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static displayPortVTable_t max7456VTable = {
    .grab = grab,
    .release = release,
    .clear = clearScreen,
    .write = write,
    .heartbeat = heartbeat,
    .resync = resync,
    .txBytesFree = txBytesFree,
};

displayPort_t *max7456DisplayPortInit(void)
{
    max7456DisplayPort.vTable = &max7456VTable;
    max7456DisplayPort.isGrabbed = false;
    resync(&max7456DisplayPort);
    return &max7456DisplayPort;
}
#endif // OSD
