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

#include <stdbool.h>

#include "platform.h"

#ifdef USE_CMS

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_hover_cal.h"

#include "common/maths.h"
#include "common/printf.h"
#include "common/utils.h"

#include "flight/hover_calibration.h"
#ifndef USE_WING
#include "flight/autopilot_multirotor.h"
#endif

#include "pg/autopilot.h"

// CURRENT (effective hover throttle in use right now) and NEW (last captured), refreshed each CMS poll
static char hoverCalCurrentBuf[6];
static char hoverCalNewBuf[6];

static const void *cmsHoverCalOnDisplayUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);

#ifndef USE_WING
    tfp_sprintf(hoverCalCurrentBuf, "%4u", autopilotGetEffectiveHoverThrottlePwm());
#else
    tfp_sprintf(hoverCalCurrentBuf, "%4u", autopilotConfig()->hoverThrottle);
#endif
    tfp_sprintf(hoverCalNewBuf, "%4u", hoverCalibrationGetNewThrottle());

    return NULL;
}

static const void *cmsHoverCalSave(displayPort_t *pDisp, const void *ptr)
{
    UNUSED(ptr);

    const uint16_t capturedThrottle = hoverCalibrationGetNewThrottle();
    if (capturedThrottle != 0) {
        autopilotConfigMutable()->hoverThrottle = constrain(capturedThrottle, 0, 1700);
    }

    return cmsMenuExit(pDisp, (void *)CMS_POPUP_SAVEREBOOT);
}

static const OSD_Entry cmsx_menuHoverCalEntries[] =
{
    { "HOVER CALIBRATION", OME_Label,           NULL,            NULL },
    { "CURRENT",           OME_Label | DYNAMIC, NULL,            hoverCalCurrentBuf },
    { "NEW",               OME_Label | DYNAMIC, NULL,            hoverCalNewBuf },
    { "SAVE AND REBOOT",   OME_Funcall,         cmsHoverCalSave, NULL },
    { "EXIT",              OME_Back,            NULL,            NULL },
    { NULL, OME_END, NULL, NULL }
};

CMS_Menu cmsx_menuHoverCal = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUHOVERCAL",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = cmsHoverCalOnDisplayUpdate,
    .entries = cmsx_menuHoverCalEntries,
};

#endif // USE_CMS
