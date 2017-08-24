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
#include "io/vtx_rtc6705.h"


static uint8_t cmsx_vtxBand;
static uint8_t cmsx_vtxChannel;
static uint8_t cmsx_vtxPower;

static const char * const rtc6705BandNames[] = {
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
};

static OSD_TAB_t entryVtxBand =         {&cmsx_vtxBand, ARRAYLEN(rtc6705BandNames) - 1, &rtc6705BandNames[0]};
static OSD_UINT8_t entryVtxChannel =    {&cmsx_vtxChannel, 1, 8, 1};
static OSD_TAB_t entryVtxPower =        {&cmsx_vtxPower, RTC6705_POWER_COUNT - 1, &rtc6705PowerNames[0]};

static void cmsx_Vtx_ConfigRead(void)
{
    cmsx_vtxBand = vtxRTC6705Config()->band - 1;
    cmsx_vtxChannel = vtxRTC6705Config()->channel;
    cmsx_vtxPower = vtxRTC6705Config()->power;
}

static void cmsx_Vtx_ConfigWriteback(void)
{
    vtxRTC6705ConfigMutable()->band = cmsx_vtxBand + 1;
    vtxRTC6705ConfigMutable()->channel = cmsx_vtxChannel;
    vtxRTC6705ConfigMutable()->power = cmsx_vtxPower;
}

static long cmsx_Vtx_onEnter(void)
{
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
    {"--- VTX ---", OME_Label, NULL, NULL, 0},
    {"BAND", OME_TAB, NULL, &entryVtxBand, 0},
    {"CHANNEL", OME_UINT8, NULL, &entryVtxChannel, 0},
    {"POWER", OME_TAB, NULL, &entryVtxPower, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuVtxRTC6705 = {
    .GUARD_text = "MENUVTX",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_Vtx_onEnter,
    .onExit= cmsx_Vtx_onExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuVtxEntries
};

#endif // CMS
