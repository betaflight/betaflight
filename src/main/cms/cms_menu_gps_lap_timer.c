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
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS_GPS_LAP_TIMER_MENU

#include "cli/settings.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_gps_lap_timer.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/gps_lap_timer.h"

static uint16_t gpsLapTimerConfig_minimumLapTimeSeconds;

static const void *cms_menuGpsLapTimerOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    gpsLapTimerConfig_minimumLapTimeSeconds = gpsLapTimerConfig()->minimumLapTimeSeconds;

    return NULL;
}

static const void *cms_menuGpsLapTimerOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gpsLapTimerConfigMutable()->minimumLapTimeSeconds = gpsLapTimerConfig_minimumLapTimeSeconds;

    return NULL;
}

static const void *cmsSetLeftGate(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gpsLapTimerSetLeftGate();

    return NULL;
}

static const void *cmsSetRightGate(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gpsLapTimerSetRightGate();

    return NULL;
}

const OSD_Entry cms_menuGpsLapTimerEntries[] =
{
    {"--- GPS LAP TIMER ---", OME_Label, NULL, NULL, 0},

    { "MIN LAP", OME_UINT16,  NULL,            &(OSD_UINT16_t){ &gpsLapTimerConfig_minimumLapTimeSeconds, 0, 3000, 1 }, 0 },
    { "LEFT",    OME_Funcall, cmsSetLeftGate,  NULL, 0 },
    { "RIGHT",   OME_Funcall, cmsSetRightGate, NULL, 0 },

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cms_menuGpsLapTimer = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUGPSLAPTIMER",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cms_menuGpsLapTimerOnEnter,
    .onExit = cms_menuGpsLapTimerOnExit,
    .onDisplayUpdate = NULL,
    .entries = cms_menuGpsLapTimerEntries,
};

#endif // CMS_GPS_LAP_TIMER_MENU
