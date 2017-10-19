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
#include <ctype.h>

#include "platform.h"

#ifdef CMS
#include "common/printf.h"
#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "io/vtx_string.h"
#include "io/vtx_furious.h"


char furiousCmsStatusString[31] = "- -- ---- ---- --------- -.-";
//                                 m bc ffff tppp HHHHHHHHH FFF
//                                 0123456789012345678901234567

void furiousCmsUpdateStatusString(void)
{
    furiousCmsStatusString[0] = '*';
    furiousCmsStatusString[1] = ' ';
    furiousCmsStatusString[2] = vtx58BandLetter[furiousBand];
    furiousCmsStatusString[3] = vtx58ChannelNames[furiousChannel][0];

    if (furiousCurFreq)
        tfp_sprintf(&furiousCmsStatusString[4], " %4d", furiousCurFreq);
    else
        tfp_sprintf(&furiousCmsStatusString[4], " ----");

    if (furiousPower) {
        tfp_sprintf(&furiousCmsStatusString[9], " %c%3d", (furiousPower == furiousConfiguredPower) ? ' ' : '*', furiousPower);
    }
    else
        tfp_sprintf(&furiousCmsStatusString[9], " ----");

    if (furiousHardwareVersion) {
        if(furiousHardwareVersion == 1) {
            tfp_sprintf(&furiousCmsStatusString[15], " Freestype");
        }
        if(furiousHardwareVersion == 2) {
            tfp_sprintf(&furiousCmsStatusString[15], " Race     ");
        }

    }
    else
        tfp_sprintf(&furiousCmsStatusString[15], " ---------");

    if (furiousFirmwareVersion) {
        tfp_sprintf(&furiousCmsStatusString[25], " %1d.%1d", (uint8_t)(furiousFirmwareVersion >> 4), (uint8_t)(furiousFirmwareVersion & 0xf));
    }
    else
        tfp_sprintf(&furiousCmsStatusString[25], " ---");
}

uint8_t furiousCmsPitMode = 0;
uint8_t furiousCmsBand = 1;
uint8_t furiousCmsChan = 1;
uint16_t furiousCmsFreqRef;

static OSD_TAB_t furiousCmsEntBand = { &furiousCmsBand, VTX58_BAND_COUNT, vtx58BandNames };

static OSD_TAB_t furiousCmsEntChan = { &furiousCmsChan, VTX58_CHANNEL_COUNT, vtx58ChannelNames };

static OSD_UINT16_t furiousCmsEntFreqRef = { &furiousCmsFreqRef, 5600, 5900, 0 };

static uint8_t furiousCmsPower = 1;

static OSD_TAB_t furiousCmsEntPower = { &furiousCmsPower, VTX_FURIOUS_POWER_COUNT, furiousPowerNames };

static void furiousCmsUpdateFreqRef(void)
{
    if (furiousCmsBand > 0 && furiousCmsChan > 0)
        furiousCmsFreqRef = vtx58frequencyTable[furiousCmsBand - 1][furiousCmsChan - 1];
}

static long furiousCmsConfigBand(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (furiousCmsBand == 0)
        // Bounce back
        furiousCmsBand = 1;
    else
        furiousCmsUpdateFreqRef();

    return 0;
}

static long furiousCmsConfigChan(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (furiousCmsChan == 0)
        // Bounce back
        furiousCmsChan = 1;
    else
        furiousCmsUpdateFreqRef();

    return 0;
}

static long furiousCmsConfigPower(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (furiousCmsPower == 0)
        // Bounce back
        furiousCmsPower = 1;

    return 0;
}

static OSD_INT16_t furiousCmsEntTemp = { &furiousTemperature, -100, 300, 0 };

static const char * const furiousCmsPitModeNames[] = {
    "---", "OFF", "ON "
};

static OSD_TAB_t furiousCmsEntPitMode = { &furiousCmsPitMode, 2, furiousCmsPitModeNames };

static long furiousCmsSetPitMode(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (furiousCmsPitMode == 0) {
        // Bouce back
        furiousCmsPitMode = 1;
    } else {
        furiousSetPitMode(furiousCmsPitMode - 1);
    }

    return 0;
}

static long furiousCmsCommence(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    furiousSetBandAndChannel(furiousCmsBand, furiousCmsChan);
    furiousSetRFPower(furiousPowerTable[furiousCmsPower-1]);

    // If it fails, the user should retry later
    furiousCommitChanges();


    return MENU_CHAIN_BACK;
}

static void furiousCmsInitSettings()
{
    if (furiousBand > 0) furiousCmsBand = furiousBand;
    if (furiousChannel > 0) furiousCmsChan = furiousChannel;

    furiousCmsUpdateFreqRef();
    furiousCmsPitMode = furiousPitMode + 1;

    if (furiousConfiguredPower > 0) {
        for (uint8_t i = 0; i < sizeof(furiousPowerTable); i++) {
            if (furiousConfiguredPower <= furiousPowerTable[i]) {
                furiousCmsPower = i + 1;
                break;
            }
        }
    }
}

static long furiousCmsOnEnter()
{
    furiousCmsInitSettings();
    return 0;
}

static OSD_Entry furiousCmsMenuCommenceEntries[] = {
    { "CONFIRM", OME_Label,   NULL,          NULL, 0 },
    { "YES",     OME_Funcall, furiousCmsCommence, NULL, 0 },
    { "BACK",    OME_Back, NULL, NULL, 0 },
    { NULL,      OME_END, NULL, NULL, 0 }
};

static CMS_Menu furiousCmsMenuCommence = {
    .GUARD_text = "XVTXTRC",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = furiousCmsMenuCommenceEntries,
};

static OSD_Entry furiousMenuEntries[] =
{
    { "- FURIOUS STEALTH -", OME_Label, NULL, NULL, 0 },

    { "",       OME_Label,   NULL,                   furiousCmsStatusString,  DYNAMIC },
    { "PIT",    OME_TAB,     furiousCmsSetPitMode,   &furiousCmsEntPitMode,   0 },
    { "BAND",   OME_TAB,     furiousCmsConfigBand,   &furiousCmsEntBand,      0 },
    { "CHAN",   OME_TAB,     furiousCmsConfigChan,   &furiousCmsEntChan,      0 },
    { "(FREQ)", OME_UINT16,  NULL,                   &furiousCmsEntFreqRef,   DYNAMIC },
    { "POWER",  OME_TAB,     furiousCmsConfigPower,  &furiousCmsEntPower,     0 },
    { "T(C)",   OME_INT16,   NULL,                   &furiousCmsEntTemp,      DYNAMIC },
    { "SET",    OME_Submenu, cmsMenuChange,          &furiousCmsMenuCommence, 0 },

    { "BACK",   OME_Back, NULL, NULL, 0 },
    { NULL,     OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuVtxFurious = {
    .GUARD_text = "XVTXTR",
    .GUARD_type = OME_MENU,
    .onEnter = furiousCmsOnEnter,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = furiousMenuEntries,
};
#endif
