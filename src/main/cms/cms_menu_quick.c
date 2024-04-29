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
#include <string.h>

#include "platform.h"

#ifdef USE_CMS
#ifdef USE_OSD_QUICK_MENU

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_main.h"
#include "cms/cms_menu_vtx_common.h"
#include "cms/cms_menu_rpm_limit.h"

#include "io/ledstrip.h"
#include "common/printf.h"
#include "config/config.h"

#include "drivers/pwm_output.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/runtime_config.h"
#include "flight/pid.h"
#include "flight/pid_init.h"

#include "sensors/battery.h"

#include "cli/settings.h"

#include "cms_menu_quick.h"

static controlRateConfig_t rateProfile;
static uint8_t rateProfileIndex;
static batteryConfig_t batteryProfile;
static uint8_t  cmsx_motorOutputLimit;
static uint8_t pidProfileIndex;
static uint8_t cmsx_extraLedstripColor;
static uint8_t cmsx_extraLedstripColor2;
static uint8_t cmsx_extraLedstripColor2_brightness;
static pidProfile_t *pidProfile;

static const void *quickMenuOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);
    pidProfileIndex = getCurrentPidProfileIndex();
    pidProfile = pidProfilesMutable(pidProfileIndex);

    rateProfileIndex = getCurrentControlRateProfileIndex();
    memcpy(&rateProfile, controlRateProfiles(rateProfileIndex), sizeof(controlRateConfig_t));
    memcpy(&batteryProfile, batteryConfigMutable(), sizeof(batteryConfig_t));

    cmsx_motorOutputLimit = pidProfile->motor_output_limit;
    cmsx_extraLedstripColor = ledStripConfig()->extra_ledstrip_color;
    cmsx_extraLedstripColor2 = ledStripConfig()->extra_ledstrip_color2;
    cmsx_extraLedstripColor2_brightness = ledStripConfig()->extra_ledstrip_color2_brightness;

    return NULL;
}

static const void *cmsx_RateProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    memcpy(controlRateProfilesMutable(rateProfileIndex), &rateProfile, sizeof(controlRateConfig_t));
        memcpy(batteryConfigMutable(), &batteryProfile, sizeof(batteryConfig_t));

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);
    pidProfile->motor_output_limit = cmsx_motorOutputLimit;

    ledStripConfigMutable()->extra_ledstrip_color = cmsx_extraLedstripColor;
    ledStripConfigMutable()->extra_ledstrip_color2 = cmsx_extraLedstripColor2;
    ledStripConfigMutable()->extra_ledstrip_color2_brightness = cmsx_extraLedstripColor2_brightness;

    pidInitConfig(currentPidProfile);

    return NULL;
}

static const void *writeLedColor(displayPort_t *pDisp, const OSD_Entry *self) {
    UNUSED(pDisp);
    UNUSED(self);
    ledStripConfigMutable()->extra_ledstrip_color = cmsx_extraLedstripColor;
    ledStripConfigMutable()->extra_ledstrip_color2 = cmsx_extraLedstripColor2;
    ledStripConfigMutable()->extra_ledstrip_color2_brightness = cmsx_extraLedstripColor2_brightness;
    return NULL;
}


static const char * const osdTableThrottleLimitType[] = {
    "OFF", "SCALE", "CLIP"
};


static const OSD_Entry menuMainEntries[] =
{
    { "-- QUICK --",  OME_Label, NULL, NULL },

#if defined(USE_RPM_LIMIT)
    { "RPM LIM", OME_Submenu, cmsMenuChange, &cmsx_menuRpmLimit },
#endif
    { "THR LIM TYPE",OME_TAB,    NULL, &(OSD_TAB_t)   { &rateProfile.throttle_limit_type, THROTTLE_LIMIT_TYPE_COUNT - 1, osdTableThrottleLimitType} },
    { "THR LIM %",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.throttle_limit_percent, 25,  100,  1} },
    { "MTR OUT LIM %",OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_motorOutputLimit, MOTOR_OUTPUT_LIMIT_PERCENT_MIN,  MOTOR_OUTPUT_LIMIT_PERCENT_MAX,  1} },
    { "FORCE CELLS",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &batteryProfile.forceBatteryCellCount, 0, 24, 1} },
#if defined(USE_VTX_CONTROL)
#if defined(USE_VTX_RTC6705) || defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
    {"VTX", OME_Funcall, cmsSelectVtx, NULL},
#endif
#endif // VTX_CONTROL
    { "FORCE LED",     OME_TAB, NULL, &(OSD_TAB_t) { &cmsx_extraLedstripColor, COLOR_COUNT - 1, lookupTableLedstripColors} },
    { "FORCE LED2",    OME_TAB, NULL, &(OSD_TAB_t) { &cmsx_extraLedstripColor2, COLOR_COUNT - 1, lookupTableLedstripColors} },
    { "LED2 BRIGHT",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_extraLedstripColor2_brightness, 0, 255, 1} },
    { "MAIN",     OME_Submenu,  NULL, &cmsx_menuMain},
    { "EXIT",            OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT},
    { "SAVE&REBOOT",     OME_OSD_Exit, cmsMenuExit,   (void *)CMS_POPUP_SAVEREBOOT},
    {NULL, OME_END, NULL, NULL},
};

CMS_Menu cmsx_menuQuick = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUQUICK",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = quickMenuOnEnter,
    .onExit = cmsx_RateProfileWriteback,
    .onDisplayUpdate = writeLedColor,
    .entries = menuMainEntries,
};

#endif // USE_OSD_QUICK_MENU
#endif // USE_CMS
