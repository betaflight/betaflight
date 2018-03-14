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

#ifdef USE_RCDEVICE

#include "drivers/display.h"

#include "io/rcdevice.h"
#include "io/rcdevice_osd.h"

#include "pg/vcd.h"

#include "displayport_rcdevice.h"

displayPort_t rcdeviceOSDDisplayPort;

static const displayPortVTable_t rcdeviceOSDVTable = {
    .grab = rcdeviceOSDGrab,
    .release = rcdeviceOSDRelease,
    .clearScreen = rcdeviceOSDClearScreen,
    .drawScreen = rcdeviceOSDDrawScreen,
    .writeString = rcdeviceOSDWriteString,
    .writeChar = rcdeviceOSDWriteChar,
    .isTransferInProgress = rcdeviceOSDIsTransferInProgress,
    .heartbeat = rcdeviceOSDHeartbeat,
    .resync = rcdeviceOSDResync,
    .isSynced = rcdeviceOSDIsSynced,
    .txBytesFree = rcdeviceOSDTxBytesFree,
    .screenSize = rcdeviceScreenSize,
};

displayPort_t *rcdeviceDisplayPortInit(const vcdProfile_t *vcdProfile)
{
    if (rcdeviceOSDInit(vcdProfile)) {
        displayInit(&rcdeviceOSDDisplayPort, &rcdeviceOSDVTable);
        rcdeviceOSDResync(&rcdeviceOSDDisplayPort);
        return &rcdeviceOSDDisplayPort;
    } else {
        return NULL;
    }
}

#endif // defined(USE_RCDEVICE)
