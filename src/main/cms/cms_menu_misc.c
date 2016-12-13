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
#include <string.h>
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#ifdef CMS

#include "common/axis.h"
#include "io/gimbal.h"
#include "flight/pid.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_ledstrip.h"

//
// Misc
//

static long cmsx_menuRcConfirmBack(const OSD_Entry *self)
{
    if (self && self->type == OME_Back)
        return 0;
    else
        return -1;
}

//
// RC preview
//
static OSD_Entry cmsx_menuRcEntries[] =
{
    { "-- RC PREV --", OME_Label, NULL, NULL, 0},

    { "ROLL",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[ROLL],     1, 2500, 0 }, DYNAMIC },
    { "PITCH", OME_INT16, NULL, &(OSD_INT16_t){ &rcData[PITCH],    1, 2500, 0 }, DYNAMIC },
    { "THR",   OME_INT16, NULL, &(OSD_INT16_t){ &rcData[THROTTLE], 1, 2500, 0 }, DYNAMIC },
    { "YAW",   OME_INT16, NULL, &(OSD_INT16_t){ &rcData[YAW],      1, 2500, 0 }, DYNAMIC },

    { "AUX1",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX1],     1, 2500, 0 }, DYNAMIC },
    { "AUX2",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX2],     1, 2500, 0 }, DYNAMIC },
    { "AUX3",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX3],     1, 2500, 0 }, DYNAMIC },
    { "AUX4",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX4],     1, 2500, 0 }, DYNAMIC },

    { "BACK",  OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuRcPreview = {
    .GUARD_text = "XRCPREV",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = cmsx_menuRcConfirmBack,
    .onGlobalExit = NULL,
    .entries = cmsx_menuRcEntries
};


static OSD_Entry menuMiscEntries[]=
{
    { "-- MISC --", OME_Label, NULL, NULL, 0 },

    { "MIN THR",    OME_UINT16,  NULL,          &(OSD_UINT16_t){ &motorConfig()->minthrottle,         1000, 2000, 1 }, 0 },
    { "VBAT SCALE", OME_UINT8,   NULL,          &(OSD_UINT8_t) { &batteryConfig()->vbatscale,             1, 250, 1 }, 0 },
    { "VBAT CLMAX", OME_UINT8,   NULL,          &(OSD_UINT8_t) { &batteryConfig()->vbatmaxcellvoltage,   10,  50, 1 }, 0 },
    { "RC PREV",    OME_Submenu, cmsMenuChange, &cmsx_menuRcPreview, 0},

    { "BACK", OME_Back, NULL, NULL, 0},
    { NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuMisc = {
    .GUARD_text = "XMISC",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuMiscEntries
};

#endif // CMS
