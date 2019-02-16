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

#include "platform.h"

#if defined(USE_CMS) && defined(USE_VTX_RTC6705)

#include "common/printf.h"
#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "drivers/vtx_common.h"

#include "fc/config.h"

#include "io/vtx_string.h"
#include "io/vtx_rtc6705.h"
#include "io/vtx.h"

static uint8_t cmsx_vtxBand;
static uint8_t cmsx_vtxChannel;
static uint8_t cmsx_vtxPower;

static OSD_TAB_t entryVtxBand;
static OSD_UINT8_t entryVtxChannel;
static OSD_TAB_t entryVtxPower;

static void cmsx_Vtx_ConfigRead(void)
{
    cmsx_vtxBand = vtxSettingsConfig()->band - 1;
    cmsx_vtxChannel = vtxSettingsConfig()->channel;
    cmsx_vtxPower = vtxSettingsConfig()->power - VTX_RTC6705_MIN_POWER;
}

static void cmsx_Vtx_ConfigWriteback(void)
{
    // update vtx_ settings
    vtxSettingsConfigMutable()->band = cmsx_vtxBand + 1;
    vtxSettingsConfigMutable()->channel = cmsx_vtxChannel;
    vtxSettingsConfigMutable()->power = cmsx_vtxPower + VTX_RTC6705_MIN_POWER;
    vtxSettingsConfigMutable()->freq = vtxCommonLookupFrequency(vtxCommonDevice(), cmsx_vtxBand + 1, cmsx_vtxChannel);

    saveConfigAndNotify();
}

static long cmsx_Vtx_onEnter(void)
{
    cmsx_Vtx_ConfigRead();

    vtxDevice_t *device = vtxCommonDevice();

    entryVtxBand.val = &cmsx_vtxBand;
    entryVtxBand.max = device->capability.bandCount - 1;
    entryVtxBand.names = &device->bandNames[1];

    entryVtxChannel.val = &cmsx_vtxChannel;
    entryVtxChannel.min = 1;
    entryVtxChannel.max = device->capability.channelCount;
    entryVtxChannel.step = 1;

    entryVtxPower.val = &cmsx_vtxPower;
    entryVtxPower.max = device->capability.powerCount - 1;
    entryVtxPower.names = device->powerNames;

    return 0;
}

static long cmsx_Vtx_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    cmsx_Vtx_ConfigWriteback();

    return 0;
}


static const OSD_Entry cmsx_menuVtxEntries[] =
{
    {"--- VTX ---", OME_Label, NULL, NULL, 0},
    {"BAND", OME_TAB, NULL, &entryVtxBand, 0},
    {"CHANNEL", OME_UINT8, NULL, &entryVtxChannel, 0},
    {"POWER", OME_TAB, NULL, &entryVtxPower, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuVtxRTC6705 = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUVTX",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Vtx_onEnter,
    .onExit= cmsx_Vtx_onExit,
    .entries = cmsx_menuVtxEntries
};

#endif // CMS
