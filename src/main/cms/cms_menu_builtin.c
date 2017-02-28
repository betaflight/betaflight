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
// Built-in menu contents and support functions
//

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef CMS

#include "build/version.h"

#include "drivers/time.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_builtin.h"

// Sub menus

#include "cms/cms_menu_imu.h"
#include "cms/cms_menu_blackbox.h"
#include "cms/cms_menu_vtx.h"
#include "cms/cms_menu_osd.h"
#include "cms/cms_menu_ledstrip.h"
#include "cms/cms_menu_misc.h"


// Info

static char infoGitRev[GIT_SHORT_REVISION_LENGTH];
static char infoTargetName[] = __TARGET__;

#include "msp/msp_protocol.h" // XXX for FC identification... not available elsewhere

static long cmsx_InfoInit(void)
{
    for (int i = 0 ; i < GIT_SHORT_REVISION_LENGTH ; i++) {
        if (shortGitRevision[i] >= 'a' && shortGitRevision[i] <= 'f')
            infoGitRev[i] = shortGitRevision[i] - 'a' + 'A';
        else
            infoGitRev[i] = shortGitRevision[i];
    }

    return 0;
}

static OSD_Entry menuInfoEntries[] = {
    { "--- INFO ---", OME_Label, NULL, NULL, 0 },
    { "FWID", OME_String, NULL, INAV_IDENTIFIER, 0 },
    { "FWVER", OME_String, NULL, FC_VERSION_STRING, 0 },
    { "GITREV", OME_String, NULL, infoGitRev, 0 },
    { "TARGET", OME_String, NULL, infoTargetName, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu menuInfo = {
    .GUARD_text = "MENUINFO",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_InfoInit,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuInfoEntries
};

// Features

static OSD_Entry menuFeaturesEntries[] =
{
    {"--- FEATURES ---", OME_Label, NULL, NULL, 0},
    {"BLACKBOX", OME_Submenu, cmsMenuChange, &cmsx_menuBlackbox, 0},
#if defined(VTX) || defined(USE_RTC6705)
    {"VTX", OME_Submenu, cmsMenuChange, &cmsx_menuVtx, 0},
#endif // VTX || USE_RTC6705
#ifdef LED_STRIP
    {"LED STRIP", OME_Submenu, cmsMenuChange, &cmsx_menuLedstrip, 0},
#endif // LED_STRIP
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

static CMS_Menu menuFeatures = {
    .GUARD_text = "MENUFEATURES",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuFeaturesEntries,
};

// Main

static OSD_Entry menuMainEntries[] =
{
    {"-- MAIN --",  OME_Label, NULL, NULL, 0},

    {"PID TUNING",  OME_Submenu,  cmsMenuChange, &cmsx_menuImu, 0},
    {"FEATURES",    OME_Submenu,  cmsMenuChange, &menuFeatures, 0},
#ifdef OSD
    {"SCR LAYOUT",  OME_Submenu,  cmsMenuChange, &cmsx_menuOsdLayout, 0},
    {"ALARMS",      OME_Submenu,  cmsMenuChange, &cmsx_menuAlarms, 0},
#endif
    {"FC&FW INFO",  OME_Submenu,  cmsMenuChange, &menuInfo, 0},
    {"MISC",        OME_Submenu,  cmsMenuChange, &cmsx_menuMisc, 0},
    {"SAVE&REBOOT", OME_OSD_Exit, cmsMenuExit,   (void*)1, 0},
    {"EXIT",        OME_OSD_Exit, cmsMenuExit,   (void*)0, 0},
#ifdef CMS_MENU_DEBUG
    {"ERR SAMPLE",  OME_Submenu,  cmsMenuChange, &menuInfoEntries[0], 0},
#endif

    {NULL,OME_END, NULL, NULL, 0}
};

CMS_Menu menuMain = {
    .GUARD_text = "MENUMAIN",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuMainEntries,
};
#endif
