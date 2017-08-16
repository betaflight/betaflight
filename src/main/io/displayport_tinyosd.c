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


#ifndef USE_MAX7456  // MAX7456 and tinyOSD video buffers will not fit into memory

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/display.h"
#include "drivers/tinyosd.h"
#include "drivers/vcd.h"

#include "io/displayport_tinyosd.h"
#include "io/osd.h"
#include "io/osd_slave.h"

#include "fc/config.h"

displayPort_t tinyOSDDisplayPort;

PG_REGISTER_WITH_RESET_FN(displayPortProfile_t, displayPortProfileTinyOSD, PG_DISPLAY_PORT_TINYOSD_CONFIG, 0);

void pgResetFn_displayPortProfileTinyOSD(displayPortProfile_t *displayPortProfile)
{
    displayPortProfile->colAdjust = 0;
    displayPortProfile->rowAdjust = 0;
    displayPortProfile->invert = false;
    displayPortProfile->blackBrightness = 0;
    displayPortProfile->whiteBrightness = 0;
}

static const displayPortVTable_t tinyOSDVTable = {
    .grab = tinyOSDGrab,
    .release = tinyOSDRelease,
    .clearScreen = tinyOSDClearScreen,
    .drawScreen = tinyOSDDrawScreen,
    .writeString = tinyOSDWriteString,
    .writeChar = tinyOSDWriteChar,
    .isTransferInProgress = tinyOSDIsTransferInProgress,
    .heartbeat = tinyOSDHeartbeat,
    .resync = tinyOSDResync,
    .txBytesFree = tinyOSDTxBytesFree,
};

displayPort_t *tinyOSDDisplayPortInit(const vcdProfile_t *vcdProfile)
{
    displayInit(&tinyOSDDisplayPort, &tinyOSDVTable);
    if (tinyOSDInit(vcdProfile)) {
        // init succeeded, prepare osd
        tinyOSDResync(&tinyOSDDisplayPort);
        // return port on sucessfull init
        return &tinyOSDDisplayPort;
    } else {
        // init failed, do not use this
        featureClear(FEATURE_OSD);
        return NULL;
    }
}

#endif
