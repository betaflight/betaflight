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
#include "io/vtx_opentco.h"
#include "drivers/vtx_common.h"


static uint8_t cmsx_vtxOpenTcoBand;
static uint8_t cmsx_vtxOpenTcoChannel;
static uint8_t cmsx_vtxOpenTcoPower;

static OSD_TAB_t entryVtxBand    = {&cmsx_vtxOpenTcoBand, 1, &vtx58BandNames[0]};
static OSD_TAB_t entryVtxChannel = {&cmsx_vtxOpenTcoChannel, 1, &vtx58ChannelNames[0]};
static OSD_TAB_t entryVtxPower   = {&cmsx_vtxOpenTcoPower, 1, &vtxOpentcoSupportedPowerNames[0]};

static void cmsx_Vtx_ConfigRead(void)
{
    vtxCommonGetBandAndChannel(&cmsx_vtxOpenTcoBand, &cmsx_vtxOpenTcoChannel);
    vtxCommonGetPowerIndex(&cmsx_vtxOpenTcoPower);
}

static void cmsx_Vtx_ConfigWriteback(void)
{
    vtxCommonSetBandAndChannel(cmsx_vtxOpenTcoBand,  cmsx_vtxOpenTcoChannel);
    vtxCommonSetPowerByIndex(cmsx_vtxOpenTcoPower);
}

static long cmsx_Vtx_onEnter(void)
{
    // query capabilities:
    vtxDeviceCapability_t vtxCapabilities;
    if (vtxCommonGetDeviceCapability(&vtxCapabilities)) {
        // fill max entries:
        entryVtxBand.max = vtxCapabilities.bandCount;
        entryVtxChannel.max = vtxCapabilities.channelCount;
        entryVtxPower.max = vtxCapabilities.powerCount;
    }

    cmsx_Vtx_ConfigRead();

    return 0;
}

static long cmsx_Vtx_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    cmsx_Vtx_ConfigWriteback();

    return 0;
}


static OSD_Entry cmsx_menuVtxEntries[] =
{
    {"--- VTX openTCO ---", OME_Label, NULL, NULL, 0},
    {"BAND", OME_TAB, NULL, &entryVtxBand, 0},
    {"CHANNEL", OME_UINT8, NULL, &entryVtxChannel, 0},
    {"POWER", OME_TAB, NULL, &entryVtxPower, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuVtxOpenTCO = {
    .GUARD_text = "MENUVTX",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_Vtx_onEnter,
    .onExit= cmsx_Vtx_onExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuVtxEntries
};

#endif // CMS

