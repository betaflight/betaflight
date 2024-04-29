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

#include "io/ledstrip.h"
#include "cms/cms.h"
#include "cms/cms_types.h"
#include "config/config.h"
#include "pg/stats.h"
#include "flight/mixer.h"

#include "cli/settings.h"

static uint8_t cmsx_extraLedstripColor;
static uint8_t cmsx_extraLedstripColor2;
static uint8_t cmsx_extraLedstripColor2_brightness;


static const void *cmsx_ExtraLed_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_extraLedstripColor = ledStripConfig()->extra_ledstrip_color;
    cmsx_extraLedstripColor2 = ledStripConfig()->extra_ledstrip_color2;
    cmsx_extraLedstripColor2_brightness = ledStripConfig()->extra_ledstrip_color2_brightness;

    return NULL;
}

static const void *cmsx_ExtraLed_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    ledStripConfigMutable()->extra_ledstrip_color = cmsx_extraLedstripColor;
    ledStripConfigMutable()->extra_ledstrip_color2 = cmsx_extraLedstripColor2;
    ledStripConfigMutable()->extra_ledstrip_color2_brightness = cmsx_extraLedstripColor2_brightness;

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

static const OSD_Entry cmsx_menuExtraLedEntries[] =
{
    { "-- LEDS EXTRA --", OME_Label, NULL, NULL },

    { "FORCE LED",     OME_TAB, NULL, &(OSD_TAB_t) { &cmsx_extraLedstripColor, COLOR_COUNT - 1, lookupTableLedstripColors} },
    { "FORCE LED2",    OME_TAB, NULL, &(OSD_TAB_t) { &cmsx_extraLedstripColor2, COLOR_COUNT - 1, lookupTableLedstripColors} },
    { "LED2 BRIGHT",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_extraLedstripColor2_brightness, 0, 255, 1} },

    { "SAVE&REBOOT",     OME_OSD_Exit, cmsMenuExit,   (void *)CMS_POPUP_SAVEREBOOT },
    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuExtraLed = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "RPMLIMIT",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_ExtraLed_onEnter,
    .onExit = cmsx_ExtraLed_onExit,
    .onDisplayUpdate = writeLedColor,
    .entries = cmsx_menuExtraLedEntries
};

#endif
