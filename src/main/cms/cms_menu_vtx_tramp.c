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
#include <ctype.h>
#include <string.h>
#include <drivers/vtx_table.h>

#include "platform.h"

#if defined(USE_CMS) && defined(USE_VTX_TRAMP)

#include "common/printf.h"
#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "drivers/vtx_common.h"

#include "config/config.h"

#include "io/vtx_tramp.h"
#include "io/vtx.h"

char trampCmsStatusString[31] = "- -- ---- ----";
//                               m bc ffff tppp
//                               01234567890123

static int16_t trampCmsTemp;
static OSD_INT16_t trampCmsEntTemp = { &trampCmsTemp, -100, 300, 0 };

void trampCmsUpdateStatusString(void)
{
    vtxDevice_t *vtxDevice = vtxCommonDevice();

    if (vtxTableBandCount == 0 || vtxTablePowerLevels == 0) {
        strncpy(trampCmsStatusString, "PLEASE CONFIGURE VTXTABLE", sizeof(trampCmsStatusString));
        return;
    }

    trampCmsStatusString[0] = '*';
    trampCmsStatusString[1] = ' ';
    uint8_t band;
    uint8_t chan;
    if (!vtxCommonGetBandAndChannel(vtxDevice, &band, &chan) || (band == 0 && chan == 0)) {
        trampCmsStatusString[2] = 'U';//user freq
        trampCmsStatusString[3] = 'F';
    } else {
        trampCmsStatusString[2] = vtxCommonLookupBandLetter(vtxDevice, band);
        trampCmsStatusString[3] = vtxCommonLookupChannelName(vtxDevice, chan)[0];
    }
    trampCmsStatusString[4] = ' ';

    uint16_t freq;
    if (!vtxCommonGetFrequency(vtxDevice, &freq) || (freq == 0)) {
        tfp_sprintf(&trampCmsStatusString[5], "----");
    } else {
        tfp_sprintf(&trampCmsStatusString[5], "%4d", freq);
    }

    uint16_t actualPower = vtxTrampGetCurrentActualPower();
    uint8_t powerIndex;
    uint16_t powerValue;
    if (actualPower > 0 && vtxCommonGetPowerIndex(vtxDevice, &powerIndex) && vtxCommonLookupPowerValue(vtxDevice, powerIndex, &powerValue)) {
        tfp_sprintf(&trampCmsStatusString[9], " %c%3d", (actualPower == powerValue) ? ' ' : '*', actualPower);
    } else {
        tfp_sprintf(&trampCmsStatusString[9], " ----");
    }

    trampCmsTemp = vtxTrampGetCurrentTemp();
}

uint8_t trampCmsPitMode = 0;
uint8_t trampCmsBand = 1;
uint8_t trampCmsChan = 1;
uint16_t trampCmsFreqRef;

static OSD_TAB_t trampCmsEntBand;
static OSD_TAB_t trampCmsEntChan;

static OSD_UINT16_t trampCmsEntFreqRef = { &trampCmsFreqRef, 5600, 5900, 0 };

static uint8_t trampCmsPower = 1;

static OSD_TAB_t trampCmsEntPower;

static void trampCmsUpdateFreqRef(void)
{
    if (trampCmsBand > 0 && trampCmsChan > 0) {
        trampCmsFreqRef = vtxCommonLookupFrequency(vtxCommonDevice(), trampCmsBand, trampCmsChan);
    }
}

static const void *trampCmsConfigBand(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (trampCmsBand == 0) {
        // Bounce back
        trampCmsBand = 1;
    } else {
        trampCmsUpdateFreqRef();
    }

    return NULL;
}

static const void *trampCmsConfigChan(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (trampCmsChan == 0) {
        // Bounce back
        trampCmsChan = 1;
    } else {
        trampCmsUpdateFreqRef();
    }

    return NULL;
}

static const void *trampCmsConfigPower(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (trampCmsPower == 0) {
        // Bounce back
        trampCmsPower = 1;
    }

    return NULL;
}

#define TRAMP_PIT_STATUS_NA (0)
#define TRAMP_PIT_STATUS_OFF (1)
#define TRAMP_PIT_STATUS_ON (2)

static const char * const trampCmsPitModeNames[] = {
    "---", "OFF", "ON "
};

static OSD_TAB_t trampCmsEntPitMode = { &trampCmsPitMode, 2, trampCmsPitModeNames };

static const void *trampCmsSetPitMode(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (trampCmsPitMode == TRAMP_PIT_STATUS_NA) {
        // Bouce back
        trampCmsPitMode = TRAMP_PIT_STATUS_OFF;
    } else {
        vtxCommonSetPitMode(vtxCommonDevice(),
                            (trampCmsPitMode == TRAMP_PIT_STATUS_OFF) ? 0 : 1);
    }

    return NULL;
}

static const void *trampCmsCommence(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    vtxDevice_t *device = vtxCommonDevice();
    vtxCommonSetBandAndChannel(device, trampCmsBand, trampCmsChan);
    vtxCommonSetPowerByIndex(device, trampCmsPower);

    // update'vtx_' settings
    vtxSettingsConfigMutable()->band = trampCmsBand;
    vtxSettingsConfigMutable()->channel = trampCmsChan;
    vtxSettingsConfigMutable()->power = trampCmsPower;
    vtxSettingsConfigMutable()->freq = vtxCommonLookupFrequency(vtxCommonDevice(), trampCmsBand, trampCmsChan);

    saveConfigAndNotify();

    return MENU_CHAIN_BACK;
}

static bool trampCmsInitSettings(void)
{
    vtxDevice_t *device = vtxCommonDevice();
    unsigned vtxStatus;

    if (!device) {
        return false;
    }

    vtxCommonGetBandAndChannel(device, &trampCmsBand, &trampCmsChan);

    trampCmsUpdateFreqRef();
    if (vtxCommonGetStatus(device, &vtxStatus)) {
        trampCmsPitMode = (vtxStatus & VTX_STATUS_PIT_MODE) ? TRAMP_PIT_STATUS_ON : TRAMP_PIT_STATUS_OFF;
    } else {
        trampCmsPitMode = TRAMP_PIT_STATUS_NA;
    }

    if (!vtxCommonGetPowerIndex(vtxCommonDevice(), &trampCmsPower)) {
        trampCmsPower = 1;
    }

    trampCmsEntBand.val = &trampCmsBand;
    trampCmsEntBand.max = vtxTableBandCount;
    trampCmsEntBand.names = vtxTableBandNames;

    trampCmsEntChan.val = &trampCmsChan;
    trampCmsEntChan.max = vtxTableChannelCount;
    trampCmsEntChan.names = vtxTableChannelNames;

    trampCmsEntPower.val = &trampCmsPower;
    trampCmsEntPower.max = vtxTablePowerLevels;
    trampCmsEntPower.names = vtxTablePowerLabels;

    return true;
}

static const void *trampCmsOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    if (!trampCmsInitSettings()) {
        return MENU_CHAIN_BACK;
    }

    return NULL;
}

static const OSD_Entry trampCmsMenuCommenceEntries[] = {
    { "CONFIRM", OME_Label,   NULL,          NULL, 0 },
    { "YES",     OME_Funcall, trampCmsCommence, NULL, 0 },
    { "NO",    OME_Back, NULL, NULL, 0 },
    { NULL,      OME_END, NULL, NULL, 0 }
};

static CMS_Menu trampCmsMenuCommence = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XVTXTRC",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = trampCmsMenuCommenceEntries,
};

static const OSD_Entry trampMenuEntries[] =
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
    .onDisplayUpdate = NULL,
    .entries = trampMenuEntries,
};
#endif
