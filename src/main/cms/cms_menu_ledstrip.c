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

#ifdef USE_CMS

#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_ledstrip.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "fc/config.h"

#include "io/ledstrip.h"

#include "interface/settings.h"

#ifdef USE_LED_STRIP

static uint8_t cmsx_FeatureLedstrip;
static uint8_t cmsx_LedProfile;
static uint8_t cmsx_RaceColor;
const char * const ledProfileNames[LED_PROFILE_COUNT] = {
    "RACE",
    "BEACON",
#ifdef USE_LED_STRIP_STATUS_MODE
    "STATUS"
#endif
};

static long cmsx_Ledstrip_OnEnter(void)
{
    cmsx_FeatureLedstrip = featureIsEnabled(FEATURE_LED_STRIP) ? 1 : 0;
    cmsx_LedProfile = getLedProfile();
    cmsx_RaceColor = getLedRaceColor();

    return 0;
}

static long cmsx_Ledstrip_OnExit(const OSD_Entry *self)
{
    UNUSED(self);

    if (cmsx_FeatureLedstrip) {
        featureEnable(FEATURE_LED_STRIP);
    } else {
        ledStripDisable();
        featureDisable(FEATURE_LED_STRIP);
    }

    setLedProfile(cmsx_LedProfile);
    setLedRaceColor(cmsx_RaceColor);

    return 0;
}

static OSD_Entry cmsx_menuLedstripEntries[] =
{
    { "-- LED STRIP --", OME_Label, NULL, NULL, 0 },
    { "ENABLED",         OME_Bool,  NULL, &cmsx_FeatureLedstrip, 0 },
    { "PROFILE",         OME_TAB,   NULL, &(OSD_TAB_t){ &cmsx_LedProfile, LED_PROFILE_COUNT-1, ledProfileNames }, 0 },
    { "RACE COLOR",      OME_TAB,   NULL, &(OSD_TAB_t){ &cmsx_RaceColor, COLOR_COUNT-1, lookupTableLEDRaceColors }, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuLedstrip = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENULED",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Ledstrip_OnEnter,
    .onExit = cmsx_Ledstrip_OnExit,
    .entries = cmsx_menuLedstripEntries
};
#endif // LED_STRIP
#endif // CMS
