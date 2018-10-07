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
#include "io/ledstrip.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "fc/config.h"


#ifdef USE_LED_STRIP

static bool featureRead = false;
static uint8_t cmsx_FeatureLedstrip;
static int configuredRaceColor = 0;
static uint8_t cmsx_Ledstrip_RaceColor = 0;

static long cmsx_Ledstrip_FeatureRead(void)
{
    if (!featureRead) {
        cmsx_FeatureLedstrip = featureIsEnabled(FEATURE_LED_STRIP) ? 1 : 0;
        featureRead = true;
    }

    // Race color mode is enabled only if all "Color" function LEDs have the same non-black color. 
    if (configuredRaceColor == 0) {
        for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
            const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[ledIndex];
            if (ledGetFunction(ledConfig) == LED_FUNCTION_COLOR) {
                if (ledGetColor(ledConfig) == 0) {
                    continue;
                }
                if (configuredRaceColor == 0) {
                    // First non-black color
                    configuredRaceColor = ledGetColor(ledConfig);
                } else {
                    if (configuredRaceColor != ledGetColor(ledConfig)) {
                        // Different color found. Disable race color.
                        configuredRaceColor = 0;
                        break;
                    }
                }
            }
        }
    }

    cmsx_Ledstrip_RaceColor = configuredRaceColor;

    return 0;
}

static long cmsx_Ledstrip_FeatureWriteback(const OSD_Entry *self)
{
    UNUSED(self);
    if (featureRead) {
        if (cmsx_FeatureLedstrip)
            featureEnable(FEATURE_LED_STRIP);
        else
            featureDisable(FEATURE_LED_STRIP);
    }

    return 0;
}

// Names for colorId_e. Zero (black position) is used for disabled.
static const char * const cmsx_Ledstrip_ColorNames[] = {
    "   DISABLED",
    " 1 WHITE   ",
    " 2 RED     ",
    " 3 ORANGE  ",
    " 4 YELLOW  ",
    " 5 LIME GRN",
    " 6 GREEN   ",
    " 7 MINT GRN",
    " 8 CYAN    ",
    " 9 LT BLUE ",
    "10 BLUE    ",
    "11 DK VIOLT",
    "12 MAGENTA ",
    "13 DEEP PNK"
};

static OSD_TAB_t cmsx_Ledstrip_RaceColorTab = { &cmsx_Ledstrip_RaceColor, ARRAYLEN(cmsx_Ledstrip_ColorNames) - 1, cmsx_Ledstrip_ColorNames};

static long cmsx_Ledstrip_ApplyColor(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    // If race color is disabled, bounce back.
    if (configuredRaceColor == 0) {
        cmsx_Ledstrip_RaceColor = 0;
        return 0;
    }

    // If race color is enabled, don't go back to disabled
    if (configuredRaceColor) {
        if (cmsx_Ledstrip_RaceColor == 0) {
            cmsx_Ledstrip_RaceColor = configuredRaceColor;
            return 0;
        }
    }

    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        ledConfig_t *ledConfig = &ledStripConfigMutable()->ledConfigs[ledIndex];
        if (ledGetFunction(ledConfig) == LED_FUNCTION_COLOR) {
            *ledConfig = DEFINE_LED(ledGetX(ledConfig), ledGetY(ledConfig), cmsx_Ledstrip_RaceColor, ledGetDirection(ledConfig), ledGetFunction(ledConfig), ledGetOverlay(ledConfig), ledGetParams(ledConfig));
        }
    }

    reevaluateLedConfig();

    configuredRaceColor = cmsx_Ledstrip_RaceColor;

    return 0;
}

static long cmsx_Ledstrip_AllSolid(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (configuredRaceColor == 0) {
        // No color is currently configured; default to 1 (white)
        configuredRaceColor = 1;
        cmsx_Ledstrip_RaceColor = configuredRaceColor;
    }

    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        ledConfig_t *ledConfig = &ledStripConfigMutable()->ledConfigs[ledIndex];
        *ledConfig = DEFINE_LED(ledGetX(ledConfig), ledGetY(ledConfig), cmsx_Ledstrip_RaceColor, 0, LED_FUNCTION_COLOR, 0, 0);
    }

    return 0;
}

static OSD_Entry cmsx_menuLedstripEntries[] =
{
    { "-- LED STRIP --", OME_Label,   NULL, NULL, 0 },
    { "ENABLED",         OME_Bool,    NULL, &cmsx_FeatureLedstrip, 0 },
    { "RACE COLOR",      OME_TAB,     cmsx_Ledstrip_ApplyColor, &cmsx_Ledstrip_RaceColorTab, DYNAMIC },
    { "ALL SOLID",       OME_Funcall, cmsx_Ledstrip_AllSolid, NULL, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuLedstrip = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENULED",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Ledstrip_FeatureRead,
    .onExit = cmsx_Ledstrip_FeatureWriteback,
    .entries = cmsx_menuLedstripEntries
};
#endif // LED_STRIP
#endif // CMS
