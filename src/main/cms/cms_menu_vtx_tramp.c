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

#if defined(USE_CMS) && defined(USE_VTX_TRAMP)

#include "common/printf.h"
#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "drivers/vtx_common.h"

#include "fc/config.h"

#include "io/vtx_string.h"
#include "io/vtx_tramp.h"
#include "io/vtx.h"

char trampCmsStatusString[31] = "- -- ---- ----";
//                               m bc ffff tppp
//                               01234567890123

void trampCmsUpdateStatusString(void)
{
    trampCmsStatusString[0] = '*';
    trampCmsStatusString[1] = ' ';
    trampCmsStatusString[2] = vtx58BandLetter[trampBand];
    trampCmsStatusString[3] = vtx58ChannelNames[trampChannel][0];
    trampCmsStatusString[4] = ' ';

    if (trampCurFreq)
        tfp_sprintf(&trampCmsStatusString[5], "%4d", trampCurFreq);
    else
        tfp_sprintf(&trampCmsStatusString[5], "----");

    if (trampPower) {
        tfp_sprintf(&trampCmsStatusString[9], " %c%3d", (trampPower == trampConfiguredPower) ? ' ' : '*', trampPower);
    }
    else
        tfp_sprintf(&trampCmsStatusString[9], " ----");
}

uint8_t trampCmsPitMode = 0;
uint8_t trampCmsBand = 1;
uint8_t trampCmsChan = 1;
uint16_t trampCmsFreqRef;

static OSD_TAB_t trampCmsEntBand = { &trampCmsBand, VTX_TRAMP_BAND_COUNT, vtx58BandNames };

static OSD_TAB_t trampCmsEntChan = { &trampCmsChan, VTX_TRAMP_CHANNEL_COUNT, vtx58ChannelNames };

static OSD_UINT16_t trampCmsEntFreqRef = { &trampCmsFreqRef, 5600, 5900, 0 };

static uint8_t trampCmsPower = 1;

static OSD_TAB_t trampCmsEntPower = { &trampCmsPower, sizeof(trampPowerTable), trampPowerNames };

static void trampCmsUpdateFreqRef(void)
{
    if (trampCmsBand > 0 && trampCmsChan > 0)
        trampCmsFreqRef = vtx58frequencyTable[trampCmsBand - 1][trampCmsChan - 1];
}

static long trampCmsConfigBand(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (trampCmsBand == 0)
        // Bounce back
        trampCmsBand = 1;
    else
        trampCmsUpdateFreqRef();

    return 0;
}

static long trampCmsConfigChan(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (trampCmsChan == 0)
        // Bounce back
        trampCmsChan = 1;
    else
        trampCmsUpdateFreqRef();

    return 0;
}

static long trampCmsConfigPower(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (trampCmsPower == 0)
        // Bounce back
        trampCmsPower = 1;

    return 0;
}

static OSD_INT16_t trampCmsEntTemp = { &trampTemperature, -100, 300, 0 };

static const char * const trampCmsPitModeNames[] = {
    "---", "OFF", "ON "
};

static OSD_TAB_t trampCmsEntPitMode = { &trampCmsPitMode, 2, trampCmsPitModeNames };

static long trampCmsSetPitMode(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (trampCmsPitMode == 0) {
        // Bouce back
        trampCmsPitMode = 1;
    } else {
        trampSetPitMode(trampCmsPitMode - 1);
    }

    return 0;
}

static long trampCmsCommence(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    trampSetBandAndChannel(trampCmsBand, trampCmsChan);
    trampSetRFPower(trampPowerTable[trampCmsPower-1]);

    // If it fails, the user should retry later
    trampCommitChanges();

    // update'vtx_' settings
    vtxSettingsConfigMutable()->band = trampCmsBand;
    vtxSettingsConfigMutable()->channel = trampCmsChan;
    vtxSettingsConfigMutable()->power = trampCmsPower;
    vtxSettingsConfigMutable()->freq = vtx58_Bandchan2Freq(trampCmsBand, trampCmsChan);

    saveConfigAndNotify();

    return MENU_CHAIN_BACK;
}

static void trampCmsInitSettings(void)
{
    if (trampBand > 0) trampCmsBand = trampBand;
    if (trampChannel > 0) trampCmsChan = trampChannel;

    trampCmsUpdateFreqRef();
    trampCmsPitMode = trampPitMode + 1;

    if (trampConfiguredPower > 0) {
        for (uint8_t i = 0; i < VTX_TRAMP_POWER_COUNT; i++) {
            if (trampConfiguredPower <= trampPowerTable[i]) {
                trampCmsPower = i + 1;
                break;
            }
        }
    }
}

static long trampCmsOnEnter(void)
{
    trampCmsInitSettings();
    return 0;
}

static OSD_Entry trampCmsMenuCommenceEntries[] = {
    { "CONFIRM", OME_Label,   NULL,          NULL, 0 },
    { "YES",     OME_Funcall, trampCmsCommence, NULL, 0 },
    { "BACK",    OME_Back, NULL, NULL, 0 },
    { NULL,      OME_END, NULL, NULL, 0 }
};

static CMS_Menu trampCmsMenuCommence = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XVTXTRC",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .entries = trampCmsMenuCommenceEntries,
};

static OSD_Entry trampMenuEntries[] =
{
    { "- TRAMP -", OME_Label, NULL, NULL, 0 },

    { "",       OME_Label,   NULL,                   trampCmsStatusString,  DYNAMIC },
    { "PIT",    OME_TAB,     trampCmsSetPitMode,     &trampCmsEntPitMode,   0 },
    { "BAND",   OME_TAB,     trampCmsConfigBand,     &trampCmsEntBand,      0 },
    { "CHAN",   OME_TAB,     trampCmsConfigChan,     &trampCmsEntChan,      0 },
    { "(FREQ)", OME_UINT16,  NULL,                   &trampCmsEntFreqRef,   DYNAMIC },
    { "POWER",  OME_TAB,     trampCmsConfigPower,    &trampCmsEntPower,     0 },
    { "T(C)",   OME_INT16,   NULL,                   &trampCmsEntTemp,      DYNAMIC },
    { "SET",    OME_Submenu, cmsMenuChange,          &trampCmsMenuCommence, 0 },

    { "BACK",   OME_Back, NULL, NULL, 0 },
    { NULL,     OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuVtxTramp = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XVTXTR",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = trampCmsOnEnter,
    .onExit = NULL,
    .entries = trampMenuEntries,
};
#endif
