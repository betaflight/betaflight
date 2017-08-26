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

//
// CMS things for blackbox and flashfs.
//

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef CMS

#include "blackbox/blackbox.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_blackbox.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/time.h"

#include "fc/config.h"

#include "io/flashfs.h"

#ifdef USE_FLASHFS
static long cmsx_EraseFlash(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);

    displayClearScreen(pDisplay);
    displayWrite(pDisplay, 5, 3, "ERASING FLASH...");
    displayResync(pDisplay); // Was max7456RefreshAll(); Why at this timing?

    flashfsEraseCompletely();
    while (!flashfsIsReady()) {
        delay(100);
    }

    displayClearScreen(pDisplay);
    displayResync(pDisplay); // Was max7456RefreshAll(); wedges during heavy SPI?

    return 0;
}
#endif // USE_FLASHFS

static bool featureRead = false;
static uint8_t cmsx_FeatureBlackbox;
static uint8_t blackboxConfig_rate_denom;

static long cmsx_menuBlackboxOnEnter(void)
{
    if (!featureRead) {
        cmsx_FeatureBlackbox = feature(FEATURE_BLACKBOX) ? 1 : 0;
        blackboxConfig_rate_denom = blackboxConfig()->rate_denom;
        featureRead = true;
    }
    return 0;
}

static long cmsx_Blackbox_FeatureWriteback(void)
{
    // If we did read the data into CMS cache - write it back
    if (featureRead) {
        if (cmsx_FeatureBlackbox)
            featureSet(FEATURE_BLACKBOX);
        else
            featureClear(FEATURE_BLACKBOX);

        blackboxConfigMutable()->rate_denom = blackboxConfig_rate_denom;
    }

    return 0;
}

static OSD_Entry cmsx_menuBlackboxEntries[] =
{
    { "-- BLACKBOX --", OME_Label, NULL, NULL, 0},
    { "ENABLED",     OME_Bool,    NULL,            &cmsx_FeatureBlackbox,                                      0 },
    { "RATE DENOM",  OME_UINT8,   NULL,            &(OSD_UINT8_t){ &blackboxConfig_rate_denom,1,32,1 }, 0 },

#ifdef USE_FLASHFS
    { "ERASE FLASH", OME_Funcall, cmsx_EraseFlash, NULL,                                                       0 },
#endif // USE_FLASHFS

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuBlackbox = {
    .GUARD_text = "MENUBB",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_menuBlackboxOnEnter,
    .onExit = NULL,
    .onGlobalExit = cmsx_Blackbox_FeatureWriteback,
    .entries = cmsx_menuBlackboxEntries
};
#endif
