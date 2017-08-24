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

#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/display.h"
#include "drivers/opentco_osd.h"
#include "drivers/vcd.h"

#include "io/displayport_opentco.h"
#include "io/osd.h"
#include "io/osd_slave.h"

#include "fc/config.h"

#if defined(USE_OPENTCO)

displayPort_t opentcoOSDDisplayPort;

static const displayPortVTable_t opentcoOSDVTable = {
    .grab          = opentcoOSDGrab,
    .release       = opentcoOSDRelease,
    .clearScreen   = opentcoOSDClearScreen,
    .drawScreen    = opentcoOSDDrawScreen,
    .fillRegion    = opentcoOSDFillRegion,
    .writeString   = opentcoOSDWriteString,
    .writeChar     = opentcoOSDWriteChar,
    .reloadProfile = opentcoOSDReloadProfile,
    .isTransferInProgress = opentcoOSDIsTransferInProgress,
    .heartbeat     = opentcoOSDHeartbeat,
    .resync        = opentcoOSDResync,
    .txBytesFree   = opentcoOSDTxBytesFree,
};

displayPort_t *opentcoDisplayPortInit(const vcdProfile_t *vcdProfile)
{
    if (opentcoOSDInit(vcdProfile)) {
        // init succeeded, prepare osd
        displayInit(&opentcoOSDDisplayPort, &opentcoOSDVTable);
        opentcoOSDResync(&opentcoOSDDisplayPort);
        // return port on sucessfull init
        return &opentcoOSDDisplayPort;
    } else {
        // init failed, do not use this
        featureClear(FEATURE_OSD);
        return NULL;
    }
}

#endif  // defined(USE_OPENTCO)
