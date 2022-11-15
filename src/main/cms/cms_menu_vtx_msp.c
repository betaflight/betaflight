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

#include "platform.h"

#if defined(USE_CMS) && defined(USE_VTX_MSP)

#include "common/printf.h"
#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "config/config.h"

#include "io/vtx_msp.h"
#include "io/vtx.h"

char mspCmsStatusString[31] = "- -- ---- ----";
//                               m bc ffff tppp
//                               01234567890123

void mspCmsUpdateStatusString(void)
{
    vtxDevice_t *vtxDevice = vtxCommonDevice();

    if (vtxTableBandCount == 0 || vtxTablePowerLevels == 0) {
        strncpy(mspCmsStatusString, "PLEASE CONFIGURE VTXTABLE", sizeof(mspCmsStatusString));
        return;
    }

    mspCmsStatusString[0] = '*';
    mspCmsStatusString[1] = ' ';
    uint8_t band;
    uint8_t chan;
    if (!vtxCommonGetBandAndChannel(vtxDevice, &band, &chan) || (band == 0 && chan == 0)) {
        mspCmsStatusString[2] = 'U';//user freq
        mspCmsStatusString[3] = 'F';
    } else {
        mspCmsStatusString[2] = vtxCommonLookupBandLetter(vtxDevice, band);
        mspCmsStatusString[3] = vtxCommonLookupChannelName(vtxDevice, chan)[0];
    }
    mspCmsStatusString[4] = ' ';

    uint16_t freq;
    if (!vtxCommonGetFrequency(vtxDevice, &freq) || (freq == 0)) {
        tfp_sprintf(&mspCmsStatusString[5], "----");
    } else {
        tfp_sprintf(&mspCmsStatusString[5], "%4d", freq);
    }

    uint8_t powerIndex;
    uint16_t powerValue;
    if (vtxCommonGetPowerIndex(vtxDevice, &powerIndex) && vtxCommonLookupPowerValue(vtxDevice, powerIndex, &powerValue)) {
        tfp_sprintf(&mspCmsStatusString[9], " *%3d", powerValue);
    } else {
        tfp_sprintf(&mspCmsStatusString[9], " ----");
    }
}

uint8_t mspCmsPitMode = 0;
uint8_t mspCmsBand = 1;
uint8_t mspCmsChan = 1;
uint16_t mspCmsFreqRef;

static OSD_TAB_t mspCmsEntBand;
static OSD_TAB_t mspCmsEntChan;

static OSD_UINT16_t mspCmsEntFreqRef = { &mspCmsFreqRef, 5600, 5900, 0 };

static uint8_t mspCmsPower = 1;

static OSD_TAB_t mspCmsEntPower;

static void mspCmsUpdateFreqRef(void)
{
    if (mspCmsBand > 0 && mspCmsChan > 0) {
        mspCmsFreqRef = vtxCommonLookupFrequency(vtxCommonDevice(), mspCmsBand, mspCmsChan);
    }
}

static const void *mspCmsConfigBand(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (mspCmsBand == 0) {
        // Bounce back
        mspCmsBand = 1;
    } else {
        mspCmsUpdateFreqRef();
    }

    return NULL;
}

static const void *mspCmsConfigChan(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (mspCmsChan == 0) {
        // Bounce back
        mspCmsChan = 1;
    } else {
        mspCmsUpdateFreqRef();
    }

    return NULL;
}

static const void *mspCmsConfigPower(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (mspCmsPower == 0) {
        // Bounce back
        mspCmsPower = 1;
    }

    return NULL;
}

#define MSP_PIT_STATUS_NA (0)
#define MSP_PIT_STATUS_OFF (1)
#define MSP_PIT_STATUS_ON (2)

static const char * const mspCmsPitModeNames[] = {
    "---", "OFF", "ON "
};

static OSD_TAB_t mspCmsEntPitMode = { &mspCmsPitMode, 2, mspCmsPitModeNames };

static const void *mspCmsSetPitMode(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (mspCmsPitMode == MSP_PIT_STATUS_NA) {
        // Bouce back
        mspCmsPitMode = MSP_PIT_STATUS_OFF;
    } else {
        vtxCommonSetPitMode(vtxCommonDevice(), (mspCmsPitMode == MSP_PIT_STATUS_OFF) ? 0 : 1);
    }

    return NULL;
}

static const void *mspCmsCommence(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    vtxDevice_t *device = vtxCommonDevice();
    vtxCommonSetBandAndChannel(device, mspCmsBand, mspCmsChan);
    vtxCommonSetPowerByIndex(device, mspCmsPower);

    // update'vtx_' settings
    vtxSettingsConfigMutable()->band = mspCmsBand;
    vtxSettingsConfigMutable()->channel = mspCmsChan;
    vtxSettingsConfigMutable()->power = mspCmsPower;
    vtxSettingsConfigMutable()->freq = vtxCommonLookupFrequency(vtxCommonDevice(), mspCmsBand, mspCmsChan);

    saveConfigAndNotify();

    return MENU_CHAIN_BACK;
}

static bool mspCmsInitSettings(void)
{
    vtxDevice_t *device = vtxCommonDevice();
    unsigned vtxStatus;

    if (!device) {
        return false;
    }

    vtxCommonGetBandAndChannel(device, &mspCmsBand, &mspCmsChan);

    mspCmsUpdateFreqRef();
    if (vtxCommonGetStatus(device, &vtxStatus)) {
        mspCmsPitMode = (vtxStatus & VTX_STATUS_PIT_MODE) ? MSP_PIT_STATUS_ON : MSP_PIT_STATUS_OFF;
    } else {
        mspCmsPitMode = MSP_PIT_STATUS_NA;
    }

    if (!vtxCommonGetPowerIndex(vtxCommonDevice(), &mspCmsPower)) {
        mspCmsPower = 1;
    }

    mspCmsEntBand.val = &mspCmsBand;
    mspCmsEntBand.max = vtxTableBandCount;
    mspCmsEntBand.names = vtxTableBandNames;

    mspCmsEntChan.val = &mspCmsChan;
    mspCmsEntChan.max = vtxTableChannelCount;
    mspCmsEntChan.names = vtxTableChannelNames;

    mspCmsEntPower.val = &mspCmsPower;
    mspCmsEntPower.max = vtxTablePowerLevels;
    mspCmsEntPower.names = vtxTablePowerLabels;

    return true;
}

static const void *mspCmsOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    if (!mspCmsInitSettings()) {
        return MENU_CHAIN_BACK;
    }

    return NULL;
}

static const OSD_Entry mspCmsMenuCommenceEntries[] = {
    { "CONFIRM", OME_Label,   NULL,           NULL },
    { "YES",     OME_Funcall, mspCmsCommence, NULL },
    { "NO",      OME_Back,    NULL,           NULL },
    { NULL,      OME_END,     NULL,           NULL }
};

static CMS_Menu mspCmsMenuCommence = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XVTXMSPC",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = mspCmsMenuCommenceEntries,
};

static const OSD_Entry mspMenuEntries[] =
{
    { "- MSP -", OME_Label,             NULL,              NULL },
    { "",        OME_Label | DYNAMIC,   NULL,              mspCmsStatusString },
    { "PIT",     OME_TAB,               mspCmsSetPitMode,  &mspCmsEntPitMode },
    { "BAND",    OME_TAB,               mspCmsConfigBand,  &mspCmsEntBand },
    { "CHAN",    OME_TAB,               mspCmsConfigChan,  &mspCmsEntChan },
    { "(FREQ)",  OME_UINT16 | DYNAMIC,  NULL,              &mspCmsEntFreqRef },
    { "POWER",   OME_TAB,               mspCmsConfigPower, &mspCmsEntPower },
    { "SAVE",    OME_Submenu,           cmsMenuChange,     &mspCmsMenuCommence },
    { "BACK",    OME_Back,              NULL,              NULL },
    { NULL,      OME_END,               NULL,              NULL }
};

CMS_Menu cmsx_menuVtxMsp = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XVTXMSP",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = mspCmsOnEnter,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = mspMenuEntries,
};
#endif
