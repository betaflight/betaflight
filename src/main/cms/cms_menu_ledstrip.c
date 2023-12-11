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

#include "cli/settings.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "config/feature.h"

#include "config/config.h"

#include "io/ledstrip.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "cms_menu_ledstrip.h"

#ifdef USE_LED_STRIP

static uint8_t cmsx_FeatureLedstrip;
static uint8_t cmsx_ledProfile;
static uint8_t cmsx_ledRaceColor;
static uint8_t cmsx_ledBeaconColor;
static uint16_t cmsx_ledBeaconPeriod;
static uint8_t cmsx_ledBeaconOnPercent;
static uint8_t cmsx_ledBeaconArmedOnly;
static uint8_t cmsx_ledVisualBeeper;
static uint8_t cmsx_ledVisualBeeperColor;

const char * const ledProfileNames[LED_PROFILE_COUNT] = {
    "RACE",
    "BEACON",
#ifdef USE_LED_STRIP_STATUS_MODE
    "DEFAULT"
#endif
};

static const void *cmsx_Ledstrip_OnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_FeatureLedstrip = featureIsEnabled(FEATURE_LED_STRIP) ? 1 : 0;
    cmsx_ledProfile = getLedProfile();
    cmsx_ledRaceColor = ledStripConfig()->ledstrip_race_color;
    cmsx_ledBeaconColor = ledStripConfig()->ledstrip_beacon_color;
    cmsx_ledBeaconPeriod = ledStripConfig()->ledstrip_beacon_period_ms;
    cmsx_ledBeaconOnPercent = ledStripConfig()->ledstrip_beacon_percent;
    cmsx_ledBeaconArmedOnly = ledStripConfig()->ledstrip_beacon_armed_only;
    cmsx_ledVisualBeeper = ledStripConfig()->ledstrip_visual_beeper;
    cmsx_ledVisualBeeperColor = ledStripConfig()->ledstrip_visual_beeper_color;

    return NULL;
}

static const void *cmsx_Ledstrip_OnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (cmsx_FeatureLedstrip) {
        featureEnableImmediate(FEATURE_LED_STRIP);
        ledStripEnable();
    } else {
        ledStripDisable();
        featureDisableImmediate(FEATURE_LED_STRIP);
    }

    setLedProfile(cmsx_ledProfile);
    ledStripConfigMutable()->ledstrip_race_color = cmsx_ledRaceColor;
    ledStripConfigMutable()->ledstrip_beacon_color = cmsx_ledBeaconColor;
    ledStripConfigMutable()->ledstrip_beacon_period_ms = cmsx_ledBeaconPeriod;
    ledStripConfigMutable()->ledstrip_beacon_percent = cmsx_ledBeaconOnPercent;
    ledStripConfigMutable()->ledstrip_beacon_armed_only = cmsx_ledBeaconArmedOnly;
    ledStripConfigMutable()->ledstrip_visual_beeper = cmsx_ledVisualBeeper;
    ledStripConfigMutable()->ledstrip_visual_beeper_color = cmsx_ledVisualBeeperColor;

    return NULL;
}

static const OSD_Entry cmsx_menuLedstripEntries[] =
{
    { "-- LED STRIP --",  OME_Label, NULL, NULL },
    { "ENABLED",          OME_Bool,  NULL, &cmsx_FeatureLedstrip },
    { "PROFILE",          OME_TAB,   NULL, &(OSD_TAB_t){ &cmsx_ledProfile, LED_PROFILE_COUNT - 1, ledProfileNames } },
    { "RACE COLOR",       OME_TAB,   NULL, &(OSD_TAB_t){ &cmsx_ledRaceColor, COLOR_COUNT - 1, lookupTableLedstripColors } },
    { "BEACON COLOR",     OME_TAB,   NULL, &(OSD_TAB_t){ &cmsx_ledBeaconColor, COLOR_COUNT -1, lookupTableLedstripColors } },
    { "BEACON PERIOD",    OME_UINT16,NULL, &(OSD_UINT16_t){ &cmsx_ledBeaconPeriod, 50, 10000, 10 } },
    { "BEACON ON %",      OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_ledBeaconOnPercent, 0, 100, 1 } },
    { "BEACON ARMED ONLY",OME_Bool,  NULL, &cmsx_ledBeaconArmedOnly },
    { "VISUAL BEEPER",    OME_Bool,  NULL, &cmsx_ledVisualBeeper },
    { "VISUAL COLOR",     OME_TAB,   NULL, &(OSD_TAB_t){ &cmsx_ledVisualBeeperColor, COLOR_COUNT - 1, lookupTableLedstripColors } },
    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuLedstrip = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENULED",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Ledstrip_OnEnter,
    .onExit = cmsx_Ledstrip_OnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuLedstripEntries
};
#endif // LED_STRIP
#endif // CMS
