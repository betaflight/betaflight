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

#include "common/utils.h"

#include "drivers/system.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_ledstrip.h"

#ifdef LED_STRIP

//local variable to keep color value
static uint8_t ledColor;

// Find first led with color flag and restore color index.
// After saving all leds with flags color will have color set in OSD
static void getLedColor(void)
{
    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];

        int fn = ledGetFunction(ledConfig);

        if (fn == LED_FUNCTION_COLOR) {
            ledColor = ledGetColor(ledConfig);
            break;
        }
    }
}

//udate all leds with flag color
static long applyLedColor(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];
        if (ledGetFunction(ledConfig) == LED_FUNCTION_COLOR)
            *ledConfig = DEFINE_LED(ledGetX(ledConfig), ledGetY(ledConfig), ledColor, ledGetDirection(ledConfig), ledGetFunction(ledConfig), ledGetOverlay(ledConfig), 0);
    }

    ledStripUpdate(micros());

    return 0;
}

static bool featureRead = false;
static uint8_t cmsx_FeatureLedstrip;

static long cmsx_Ledstrip_Read(void)
{
    if (!featureRead) {
        cmsx_FeatureLedstrip = feature(FEATURE_LED_STRIP) ? 1 : 0;
        featureRead = true;
    }

    getLedColor();

    return 0;
}

static long cmsx_Ledstrip_Writeback(void)
{
    if (featureRead) {
        if (cmsx_FeatureLedstrip)
            featureSet(FEATURE_LED_STRIP);
        else
            featureClear(FEATURE_LED_STRIP);
    }

    return 0;
}

static const char * const LED_COLOR_NAMES[] = {
    "BLACK   ",
    "WHITE   ",
    "RED     ",
    "ORANGE  ",
    "YELLOW  ",
    "L. GREEN",
    "GREEN   ",
    "M. GREEN",
    "CYAN    ",
    "LT BLUE ",
    "BLUE    ",
    "D VIOLET",
    "MAGENTA ",
    "DP PINK "
};

static OSD_TAB_t entryLed = {&ledColor, 13, LED_COLOR_NAMES};

static OSD_Entry cmsx_menuLedstripEntries[] =
{
    { "-- LED STRIP --", OME_Label, NULL,          NULL, 0 },
    { "ENABLED",         OME_Bool,  NULL,          &cmsx_FeatureLedstrip, 0 },
    { "LED COLOR",       OME_TAB,   applyLedColor, &entryLed, 0 },
    { "BACK",            OME_Back,  NULL,          NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuLedstrip = {
    .GUARD_text = "MENULED",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_Ledstrip_Read,
    .onExit = NULL,
    .onGlobalExit = cmsx_Ledstrip_Writeback,
    .entries = cmsx_menuLedstripEntries
};
#endif // LED_STRIP
#endif // CMS
