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
#include <drivers/vtx_table.h>

#include "platform.h"

#if defined(USE_CMS) && defined(USE_VTX_RTC6705)

#include "common/printf.h"
#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "drivers/vtx_common.h"

#include "config/config.h"

#include "io/vtx_rtc6705.h"
#include "io/vtx.h"

static uint8_t cmsx_vtxBand;
static uint8_t cmsx_vtxChannel;
static uint8_t cmsx_vtxPower;
static uint8_t cmsx_vtxPit;

#ifdef CMS_SKIP_EMPTY_VTX_TABLE_ENTRIES
static uint8_t lastVtxBand;
static uint8_t lastVtxChannel;
#endif

static OSD_TAB_t entryVtxBand;
static OSD_TAB_t entryVtxChannel;
static OSD_TAB_t entryVtxPower;
static const char * const cmsxCmsPitNames[] = {
        "---",
        "OFF",
        "ON ",
};
static OSD_TAB_t entryVtxPit = {&cmsx_vtxPit, 2, cmsxCmsPitNames};

static void cmsx_Vtx_ConfigRead(void)
{
    vtxCommonGetBandAndChannel(vtxCommonDevice(), &cmsx_vtxBand, &cmsx_vtxChannel);
    vtxCommonGetPowerIndex(vtxCommonDevice(), &cmsx_vtxPower);

#ifdef CMS_SKIP_EMPTY_VTX_TABLE_ENTRIES
    lastVtxBand = cmsx_vtxBand;
    lastVtxChannel = cmsx_vtxChannel;
#endif

    unsigned status;
    if (vtxCommonGetStatus(vtxCommonDevice(), &status)) {
        cmsx_vtxPit = status & VTX_STATUS_PIT_MODE ? 2 : 1;
    } else {
        cmsx_vtxPit = 0;
    }
}

static void cmsx_Vtx_ConfigWriteback(void)
{
    // update vtx_ settings
    vtxSettingsConfigMutable()->band = cmsx_vtxBand;
    vtxSettingsConfigMutable()->channel = cmsx_vtxChannel;
    vtxSettingsConfigMutable()->power = cmsx_vtxPower;
    vtxSettingsConfigMutable()->freq = vtxCommonLookupFrequency(vtxCommonDevice(), cmsx_vtxBand, cmsx_vtxChannel);

    saveConfigAndNotify();
}

static const void *cmsx_Vtx_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_Vtx_ConfigRead();

    entryVtxBand.val = &cmsx_vtxBand;
    entryVtxBand.max = vtxTableBandCount;
    entryVtxBand.names = vtxTableBandNames;

    entryVtxChannel.val = &cmsx_vtxChannel;
    entryVtxChannel.max = vtxTableChannelCount;
    entryVtxChannel.names = vtxTableChannelNames;

    entryVtxPower.val = &cmsx_vtxPower;
    entryVtxPower.max = vtxTablePowerLevels;
    entryVtxPower.names = vtxTablePowerLabels;

    return NULL;
}

static const void *cmsx_Vtx_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    vtxCommonSetPitMode(vtxCommonDevice(), cmsx_vtxPit);
    cmsx_Vtx_ConfigWriteback();

    return NULL;
}

static const void *cmsx_Vtx_onBandChange(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);
    if (cmsx_vtxBand == 0) {
        cmsx_vtxBand = 1;
    }
#ifdef CMS_SKIP_EMPTY_VTX_TABLE_ENTRIES
    for (uint8_t band = 0; band < VTX_TABLE_MAX_BANDS; band++) {
        if (cmsx_vtxBand < (VTX_TABLE_MAX_BANDS + 1)) {
            if (vtxCommonLookupFrequency(vtxCommonDevice(), cmsx_vtxBand, cmsx_vtxChannel) == 0) {
                for (uint8_t channel = 1; channel < (VTX_TABLE_MAX_CHANNELS + 1); channel++) {
                    if (vtxCommonLookupFrequency(vtxCommonDevice(), cmsx_vtxBand, channel) != 0) {
                        lastVtxChannel = cmsx_vtxChannel = channel;
                        lastVtxBand = cmsx_vtxBand;
                        return NULL;
                    }
                }
                if ((lastVtxBand - cmsx_vtxBand) > 0) {
                    cmsx_vtxBand--;
                } else {
                    cmsx_vtxBand++;
                }
            } else {
                lastVtxBand = cmsx_vtxBand;
                break;
            }
        } else {
            cmsx_vtxBand = lastVtxBand;
            break;
        }
    }
#endif
    return NULL;
}

static const void *cmsx_Vtx_onChanChange(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);
    if (cmsx_vtxChannel == 0) {
        cmsx_vtxChannel = 1;
    }
#ifdef CMS_SKIP_EMPTY_VTX_TABLE_ENTRIES
    for (uint8_t channel = 0; channel < VTX_TABLE_MAX_CHANNELS; channel++) {
        if (cmsx_vtxChannel < (VTX_TABLE_MAX_CHANNELS + 1)) {
            if (vtxCommonLookupFrequency(vtxCommonDevice(), cmsx_vtxBand, cmsx_vtxChannel) == 0) {
                if ((lastVtxChannel - cmsx_vtxChannel) > 0) {
                    cmsx_vtxChannel--;
                } else {
                    cmsx_vtxChannel++;
                }
            } else {
                lastVtxChannel = cmsx_vtxChannel;
                break;
            }
        } else {
            cmsx_vtxChannel = lastVtxChannel;
            break;
        }
    }
#endif
    return NULL;
}

static const void *cmsx_Vtx_onPowerChange(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);
    if (cmsx_vtxPower == 0) {
        cmsx_vtxPower = 1;
    }
    return NULL;
}

static const void *cmsx_Vtx_onPitChange(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);
    if (cmsx_vtxPit == 0) {
        cmsx_vtxPit = 1;
    }
    return NULL;
}

static const OSD_Entry cmsx_menuVtxEntries[] = {
    {"--- VTX ---", OME_Label, NULL, NULL},
    {"BAND", OME_TAB, cmsx_Vtx_onBandChange, &entryVtxBand},
    {"CHANNEL", OME_TAB, cmsx_Vtx_onChanChange, &entryVtxChannel},
    {"POWER", OME_TAB, cmsx_Vtx_onPowerChange, &entryVtxPower},
    {"PIT", OME_TAB, cmsx_Vtx_onPitChange, &entryVtxPit},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuVtxRTC6705 = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUVTX",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Vtx_onEnter,
    .onExit = cmsx_Vtx_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuVtxEntries
};

#endif // CMS
