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
#include <string.h>
#include <ctype.h>

#include "platform.h"

#if defined(USE_CMS) && defined(USE_BLACKBOX)

#include "build/version.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_blackbox.h"

#include "common/printf.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/flash.h"
#include "drivers/time.h"
#include "drivers/sdcard.h"

#include "fc/config.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/flashfs.h"
#include "io/beeper.h"

#include "pg/pg.h"

static const char * const cmsx_BlackboxDeviceNames[] = {
    "NONE",
    "FLASH ",
    "SDCARD",
    "SERIAL"
};

static uint16_t blackboxConfig_p_denom;

static uint8_t cmsx_BlackboxDevice;
static OSD_TAB_t cmsx_BlackboxDeviceTable = { &cmsx_BlackboxDevice, 2, cmsx_BlackboxDeviceNames };

#define CMS_BLACKBOX_STRING_LENGTH 8
static char cmsx_BlackboxStatus[CMS_BLACKBOX_STRING_LENGTH];
static char cmsx_BlackboxDeviceStorageUsed[CMS_BLACKBOX_STRING_LENGTH];
static char cmsx_BlackboxDeviceStorageFree[CMS_BLACKBOX_STRING_LENGTH];

static void cmsx_Blackbox_GetDeviceStatus(void)
{
    char * unit = "B";
#if defined(USE_SDCARD) || defined(USE_FLASHFS)
    bool storageDeviceIsWorking = false;
#endif
    uint32_t storageUsed = 0;
    uint32_t storageFree = 0;

    switch (blackboxConfig()->device)
    {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        unit = "MB";

        if (!sdcard_isInserted()) {
            tfp_sprintf(cmsx_BlackboxStatus, "NO CARD");
        } else if (!sdcard_isFunctional()) {
            tfp_sprintf(cmsx_BlackboxStatus, "FAULT");
        } else {
            switch (afatfs_getFilesystemState()) {
            case AFATFS_FILESYSTEM_STATE_READY:
                tfp_sprintf(cmsx_BlackboxStatus, "READY");
                storageDeviceIsWorking = true;
                break;
            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
                tfp_sprintf(cmsx_BlackboxStatus, "INIT");
                break;
            case AFATFS_FILESYSTEM_STATE_FATAL:
            case AFATFS_FILESYSTEM_STATE_UNKNOWN:
            default:
                tfp_sprintf(cmsx_BlackboxStatus, "FAULT");
                break;
            }
        }

        if (storageDeviceIsWorking) {
            storageFree = afatfs_getContiguousFreeSpace() / 1024000;
            storageUsed = (sdcard_getMetadata()->numBlocks / 2000) - storageFree;
        }

        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        unit = "KB";

        storageDeviceIsWorking = flashfsIsReady();
        if (storageDeviceIsWorking) {
            tfp_sprintf(cmsx_BlackboxStatus, "READY");

            const flashGeometry_t *geometry = flashfsGetGeometry();
            storageUsed = flashfsGetOffset() / 1024;
            storageFree = (geometry->totalSize / 1024) - storageUsed;
        } else {
            tfp_sprintf(cmsx_BlackboxStatus, "FAULT");
        }

        break;
#endif

    default:
        tfp_sprintf(cmsx_BlackboxStatus, "---");
    }

    /* Storage counters */
    tfp_sprintf(cmsx_BlackboxDeviceStorageUsed, "%ld%s", storageUsed, unit);
    tfp_sprintf(cmsx_BlackboxDeviceStorageFree, "%ld%s", storageFree, unit);
}

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

    beeper(BEEPER_BLACKBOX_ERASE);
    displayClearScreen(pDisplay);
    displayResync(pDisplay); // Was max7456RefreshAll(); wedges during heavy SPI?

    // Update storage device status to show new used space amount
    cmsx_Blackbox_GetDeviceStatus();

    return 0;
}
#endif // USE_FLASHFS

static long cmsx_Blackbox_onEnter(void)
{
    cmsx_Blackbox_GetDeviceStatus();
    cmsx_BlackboxDevice = blackboxConfig()->device;

    blackboxConfig_p_denom = blackboxConfig()->p_denom;
    return 0;
}

static long cmsx_Blackbox_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    if (blackboxMayEditConfig()) {
        blackboxConfigMutable()->device = cmsx_BlackboxDevice;
        blackboxValidateConfig();
    }
    blackboxConfigMutable()->p_denom = blackboxConfig_p_denom;
    return 0;
}

static OSD_Entry cmsx_menuBlackboxEntries[] =
{
    { "-- BLACKBOX --", OME_Label, NULL, NULL, 0},
    { "DEVICE",      OME_TAB,     NULL,            &cmsx_BlackboxDeviceTable,                                 0 },
    { "(STATUS)",    OME_String,  NULL,            &cmsx_BlackboxStatus,                                      0 },
    { "(USED)",      OME_String,  NULL,            &cmsx_BlackboxDeviceStorageUsed,                           0 },
    { "(FREE)",      OME_String,  NULL,            &cmsx_BlackboxDeviceStorageFree,                           0 },
    { "P DENOM",     OME_UINT16,  NULL,            &(OSD_UINT16_t){ &blackboxConfig_p_denom, 1, INT16_MAX, 1 },0 },

#ifdef USE_FLASHFS
    { "ERASE FLASH", OME_Funcall, cmsx_EraseFlash, NULL,                                                      0 },
#endif // USE_FLASHFS

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuBlackbox = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUBB",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Blackbox_onEnter,
    .onExit = cmsx_Blackbox_onExit,
    .entries = cmsx_menuBlackboxEntries
};

#endif
