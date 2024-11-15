/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

//
// Main menu structure and support functions
//

#include <stdbool.h>

#include "platform.h"

#ifdef USE_CMS

#include "cms/cms.h"
#include "cms/cms_types.h"

// Sub menus

#include "cms/cms_menu_imu.h"
#include "cms/cms_menu_blackbox.h"
#include "cms/cms_menu_failsafe.h"
#include "cms/cms_menu_firmware.h"
#include "cms/cms_menu_ledstrip.h"
#include "cms/cms_menu_misc.h"
#include "cms/cms_menu_osd.h"
#include "cms/cms_menu_power.h"
#include "cms/cms_menu_saveexit.h"

#ifdef USE_PERSISTENT_STATS
#include "cms/cms_menu_persistent_stats.h"
#endif

// VTX supplied menus

#include "cms/cms_menu_vtx_common.h"

#include "common/printf.h"

#include "config/config.h"

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "sensors/acceleration.h"

#include "cms_menu_main.h"

#ifdef USE_BATTERY_CONTINUE
#include "sensors/battery.h"
#include "pg/stats.h"
#endif

#define CALIBRATION_STATUS_MAX_LENGTH 9

#define CALIBRATION_STATUS_REQUIRED "REQUIRED"
#define CALIBRATION_STATUS_ACTIVE   "  ACTIVE"
#define CALIBRATION_STATUS_COMPLETE "COMPLETE"

#if defined(USE_ACC)
static char accCalibrationStatus[CALIBRATION_STATUS_MAX_LENGTH];
#endif

#ifdef USE_BATTERY_CONTINUE
static char batteryContinueAmount[18];
#endif

// Features

static const OSD_Entry menuFeaturesEntries[] =
{
    {"--- FEATURES ---", OME_Label, NULL, NULL},

#if defined(USE_BLACKBOX)
    {"BLACKBOX", OME_Submenu, cmsMenuChange, &cmsx_menuBlackbox},
#endif
#if defined(USE_VTX_CONTROL)
#if defined(USE_VTX_RTC6705) || defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP) || defined(USE_VTX_MSP)
    {"VTX", OME_Funcall, cmsSelectVtx, NULL},
#endif
#endif // VTX_CONTROL
#ifdef USE_LED_STRIP
    {"LED STRIP", OME_Submenu, cmsMenuChange, &cmsx_menuLedstrip},
#endif // LED_STRIP
    {"POWER", OME_Submenu, cmsMenuChange, &cmsx_menuPower},
#ifdef USE_CMS_FAILSAFE_MENU
    {"FAILSAFE", OME_Submenu, cmsMenuChange, &cmsx_menuFailsafe},
#endif
#ifdef USE_PERSISTENT_STATS
    {"PERSISTENT STATS", OME_Submenu, cmsMenuChange, &cmsx_menuPersistentStats},
#endif
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuFeatures = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUFEATURES",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = menuFeaturesEntries,
};

static const void *cmsx_SaveExitMenu(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);

    cmsMenuChange(pDisplay, getSaveExitMenu());

    return NULL;
}

#ifdef USE_BATTERY_CONTINUE
#define SETUP_POPUP_MAX_ENTRIES 2   // Increase as new entries are added
#else
#define SETUP_POPUP_MAX_ENTRIES 1   // Increase as new entries are added
#endif

static OSD_Entry setupPopupMenuEntries[SETUP_POPUP_MAX_ENTRIES + 3];

#ifdef USE_BATTERY_CONTINUE
static const void *cmsRestoreMah(displayPort_t *pDisp, const void *self)
{
    UNUSED(self);

    setMAhDrawn(statsConfig()->stats_mah_used);
    statsConfigMutable()->stats_mah_used = 0;

    cmsMenuExit(pDisp, (void *)CMS_EXIT);

    return NULL;
}
#endif

static bool setupPopupMenuBuild(void)
{
    uint8_t menuIndex = 0;
    updateArmingStatus();

    cmsAddMenuEntry(&setupPopupMenuEntries[menuIndex], "-- SETUP MENU --", OME_Label, NULL, NULL);

    // Add menu entries for uncompleted setup tasks
#if defined(USE_ACC)
    if (sensors(SENSOR_ACC) && (getArmingDisableFlags() & ARMING_DISABLED_ACC_CALIBRATION)) {
        cmsAddMenuEntry(&setupPopupMenuEntries[++menuIndex], "CALIBRATE ACC", OME_Funcall | DYNAMIC, cmsCalibrateAccMenu, accCalibrationStatus);
    }
#endif

#ifdef USE_BATTERY_CONTINUE
    if (hasUsedMAh()) {
        cmsAddMenuEntry(&setupPopupMenuEntries[++menuIndex], batteryContinueAmount, OME_Funcall | DYNAMIC, cmsRestoreMah, NULL);
    }
#endif

    cmsAddMenuEntry(&setupPopupMenuEntries[++menuIndex], "EXIT", OME_Back | DYNAMIC, NULL, NULL);
    cmsAddMenuEntry(&setupPopupMenuEntries[++menuIndex], "NULL", OME_END, NULL, NULL);

    return (menuIndex > 2);  // return true if any setup items were added
}

static const void *setupPopupMenuOnDisplayUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);

#if defined(USE_ACC)
    // Update the ACC calibration status message.
    tfp_sprintf(accCalibrationStatus, accIsCalibrationComplete() ? accHasBeenCalibrated() ? CALIBRATION_STATUS_COMPLETE : CALIBRATION_STATUS_REQUIRED : CALIBRATION_STATUS_ACTIVE);
#endif
#ifdef USE_BATTERY_CONTINUE
    if (hasUsedMAh()) {
        tfp_sprintf(batteryContinueAmount, "RESTORE %5d MAH", statsConfig()->stats_mah_used);
    }
#endif

    return NULL;
}

CMS_Menu cmsx_menuSetupPopup = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "SETUPPOPUP",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = setupPopupMenuOnDisplayUpdate,
    .entries = setupPopupMenuEntries,
};

// Main
static const void *mainMenuOnEnter(displayPort_t *pDisp)
{
    if (setupPopupMenuBuild()) {
        // If setup issues were found then switch to the dynamically constructed menu
        cmsMenuChange(pDisp, &cmsx_menuSetupPopup);
    }
    return NULL;
}

static const OSD_Entry menuMainEntries[] =
{
    {"-- MAIN --",  OME_Label, NULL, NULL},

    {"PROFILE",     OME_Submenu,  cmsMenuChange, &cmsx_menuImu},
    {"FEATURES",    OME_Submenu,  cmsMenuChange, &cmsx_menuFeatures},
#ifdef USE_OSD
    {"OSD",         OME_Submenu,  cmsMenuChange, &cmsx_menuOsd},
#endif
    {"FC&FIRMWARE", OME_Submenu,  cmsMenuChange, &cmsx_menuFirmware},
    {"MISC",        OME_Submenu,  cmsMenuChange, &cmsx_menuMisc},
    {"SAVE/EXIT",   OME_Funcall,  cmsx_SaveExitMenu, NULL},
    {NULL, OME_END, NULL, NULL},
};

CMS_Menu cmsx_menuMain = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUMAIN",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = mainMenuOnEnter,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = menuMainEntries,
};

#endif
