/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS
#ifdef USE_RPM_LIMIT

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "config/config.h"
#include "pg/stats.h"
#include "flight/mixer.h"

#include "common/spec.h"

#include "cms/cms_menu_spec.h"

static const void *cmsx_spec_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    return NULL;
}

static const void *cmsx_spec_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

//    mixerConfigMutable()->rpm_limit_value = rpm_limit_value;
//    motorConfigMutable()->kv = kv;
//    mixerConfigMutable()->rpm_limit = rpm_limit;

    return NULL;
}

static const void *cmsx_SetFreedom(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);
    setSpec(SPEC_FREEDOM);
    saveConfigAndNotify();
    cmsMenuExit(pDisplay, (const void *)(intptr_t)CMS_POPUP_SAVEREBOOT);
    return NULL;
}

static const void *cmsx_Set7(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);
    setSpec(SPEC_7IN);
    saveConfigAndNotify();
    cmsMenuExit(pDisplay, (const void *)(intptr_t)CMS_POPUP_SAVEREBOOT);
    return NULL;
}

static const void *cmsx_SetMayhem(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);
    setSpec(SPEC_MAYHEM);
    saveConfigAndNotify();
    cmsMenuExit(pDisplay, (const void *)(intptr_t)CMS_POPUP_SAVEREBOOT);
    return NULL;
}

static const void *cmsx_SetTt(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);
    setSpec(SPEC_TT);
    saveConfigAndNotify();
    cmsMenuExit(pDisplay, (const void *)(intptr_t)CMS_POPUP_SAVEREBOOT);
    return NULL;
}


static const void *cmsx_SetLl(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);
    setSpec(SPEC_LLIGUETA);
    saveConfigAndNotify();
    cmsMenuExit(pDisplay, (const void *)(intptr_t)CMS_POPUP_SAVEREBOOT);
    return NULL;
}


static const OSD_Entry cmsx_menuSpecEntries[] =
{
    { "-- SELECT SPEC --", OME_Label, NULL, NULL },
 
    {specArray[SPEC_FREEDOM].name, OME_Funcall, cmsx_SetFreedom, NULL},
    {specArray[SPEC_7IN].name, OME_Funcall, cmsx_Set7, NULL},
    {specArray[SPEC_MAYHEM].name, OME_Funcall, cmsx_SetMayhem, NULL},
    {specArray[SPEC_TT].name, OME_Funcall, cmsx_SetTt, NULL},
    {specArray[SPEC_LLIGUETA].name, OME_Funcall, cmsx_SetLl, NULL},

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuSpec = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "SPECS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_spec_onEnter,
    .onExit = cmsx_spec_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuSpecEntries
};

#endif
#endif // USE_RPM_LIMIT
