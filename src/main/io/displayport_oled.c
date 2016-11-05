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

#ifdef OLEDCMS

#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/display_ug2864hsweg01.h"

#include "io/displayport_oled.h"

extern bool dashboardInCMS; // temporary

static int oledOpen(displayPort_t *displayPort)
{
dashboardInCMS = true;
    displayPort->inCMS = true;
    return 0;
}

static int oledClose(displayPort_t *displayPort)
{
dashboardInCMS = false;
    displayPort->inCMS = false;
    return 0;
}

static int oledClear(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    i2c_OLED_clear_display_quick();
    return 0;
}

static int oledWrite(displayPort_t *displayPort, uint8_t x, uint8_t y, char *s)
{
    UNUSED(displayPort);
    i2c_OLED_set_xy(x, y);
    i2c_OLED_send_string(s);
    return 0;
}

static int oledHeartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static void oledResync(displayPort_t *displayPort)
{
    UNUSED(displayPort);
}

static uint32_t oledTxBytesFree(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static const displayPortVTable_t oledVTable = {
    .open = oledOpen,
    .close = oledClose,
    .clear = oledClear,
    .write = oledWrite,
    .heartbeat = oledHeartbeat,
    .resync = oledResync,
    .txBytesFree = oledTxBytesFree
};

void displayPortOledInit(displayPort_t *displayPort)
{
    displayPort->vTable = &oledVTable;
    displayPort->rows = SCREEN_CHARACTER_ROW_COUNT;
    displayPort->cols = SCREEN_CHARACTER_COLUMN_COUNT;
}
#endif // OLEDCMS
