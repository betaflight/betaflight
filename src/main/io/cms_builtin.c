//
// Built-in menu contents and support functions
//

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#ifdef CMS

#include "drivers/system.h"

#include "io/cms.h"
#include "io/cms_types.h"
#include "io/cms_imu.h"

// Sub menus

#include "io/cms_imu.h"
#include "io/cms_blackbox.h"
#include "io/cms_vtx.h"
#ifdef OSD
#include "io/cms_osd.h"
#endif
#include "io/cms_ledstrip.h"


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
    { "FWID", OME_String, NULL, BETAFLIGHT_IDENTIFIER, 0 },
    { "FWVER", OME_String, NULL, FC_VERSION_STRING, 0 },
    { "GITREV", OME_String, NULL, infoGitRev, 0 },
    { "TARGET", OME_String, NULL, infoTargetName, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu menuInfo = {
    "MENUINFO",
    OME_MENU,
    cmsx_InfoInit,
    NULL,
    NULL,
    menuInfoEntries,
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
    "MENUFEATURES",
    OME_MENU,
    NULL,
    NULL,
    NULL,
    menuFeaturesEntries,
};

// Main

static OSD_Entry menuMainEntries[] =
{
    {"--- MAIN MENU ---", OME_Label, NULL, NULL, 0},
    {"CFG&IMU", OME_Submenu, cmsMenuChange, &cmsx_menuImu, 0},
    {"FEATURES", OME_Submenu, cmsMenuChange, &menuFeatures, 0},
#ifdef OSD
    {"SCR LAYOUT", OME_Submenu, cmsMenuChange, &cmsx_menuOsdLayout, 0},
    {"ALARMS", OME_Submenu, cmsMenuChange, &cmsx_menuAlarms, 0},
#endif
    {"FC&FW INFO", OME_Submenu, cmsMenuChange, &menuInfo, 0},
    {"SAVE&REBOOT", OME_OSD_Exit, cmsMenuExit, (void*)1, 0},
    {"EXIT", OME_OSD_Exit, cmsMenuExit, (void*)0, 0},
#ifdef CMS_MENU_DEBUG
    {"ERR SAMPLE", OME_Submenu, cmsMenuChange, &menuInfoEntries[0], 0},
#endif
    {NULL,OME_END, NULL, NULL, 0}
};

CMS_Menu menuMain = {
    "MENUMAIN",
    OME_MENU,
    NULL,
    NULL,
    NULL,
    menuMainEntries,
};

#endif
