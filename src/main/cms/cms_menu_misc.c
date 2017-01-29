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

#ifdef CMS

#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_misc.h"

#include "flight/mixer.h"

#include "fc/config.h"
#include "fc/rc_controls.h"

#include "rx/rx.h"

#include "sensors/battery.h"

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

static uint16_t motorConfig_minthrottle;
static uint8_t batteryConfig_vbatscale;
static uint8_t batteryConfig_vbatmaxcellvoltage;

static long cmsx_menuMiscOnEnter(void)
{
    motorConfig_minthrottle = motorConfig()->minthrottle;
    batteryConfig_vbatscale = batteryConfig()->vbatscale;
    batteryConfig_vbatmaxcellvoltage = batteryConfig()->vbatmaxcellvoltage;
    return 0;
}

static long cmsx_menuMiscOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    motorConfigMutable()->minthrottle = motorConfig_minthrottle;
    batteryConfigMutable()->vbatscale = batteryConfig_vbatscale;
    batteryConfigMutable()->vbatmaxcellvoltage = batteryConfig_vbatmaxcellvoltage;
    return 0;
}

static OSD_Entry menuMiscEntries[]=
{
    { "-- MISC --", OME_Label, NULL, NULL, 0 },

    { "MIN THR",    OME_UINT16,  NULL,          &(OSD_UINT16_t){ &motorConfig_minthrottle,         1000, 2000, 1 }, 0 },
    { "VBAT SCALE", OME_UINT8,   NULL,          &(OSD_UINT8_t) { &batteryConfig_vbatscale,             1, 250, 1 }, 0 },
    { "VBAT CLMAX", OME_UINT8,   NULL,          &(OSD_UINT8_t) { &batteryConfig_vbatmaxcellvoltage,   10,  50, 1 }, 0 },
    { "RC PREV",    OME_Submenu, cmsMenuChange, &cmsx_menuRcPreview, 0},

    { "BACK", OME_Back, NULL, NULL, 0},
    { NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuMisc = {
    .GUARD_text = "XMISC",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_menuMiscOnEnter,
    .onExit = cmsx_menuMiscOnExit,
    .onGlobalExit = NULL,
    .entries = menuMiscEntries
};

#endif // CMS
