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

#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_vtx.h"

#include "config/feature.h"

#ifdef CMS

#if defined(VTX) || defined(USE_RTC6705)

static bool featureRead = false;
static uint8_t cmsx_featureVtx = 0, cmsx_vtxBand, cmsx_vtxChannel;

static long cmsx_Vtx_FeatureRead(void)
{
    if (!featureRead) {
        cmsx_featureVtx = feature(FEATURE_VTX) ? 1 : 0;
        featureRead = true;
    }

    return 0;
}

static long cmsx_Vtx_FeatureWriteback(void)
{
    if (cmsx_featureVtx)
        featureSet(FEATURE_VTX);
    else
        featureClear(FEATURE_VTX);

    return 0;
}

static const char * const vtxBandNames[] = {
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
};

static OSD_TAB_t entryVtxBand = {&cmsx_vtxBand,4,&vtxBandNames[0]};
static OSD_UINT8_t entryVtxChannel =  {&cmsx_vtxChannel, 1, 8, 1};

static void cmsx_Vtx_ConfigRead(void)
{
#ifdef VTX
    cmsx_vtxBand = masterConfig.vtx_band;
    cmsx_vtxChannel = masterConfig.vtx_channel + 1;
#endif // VTX

#ifdef USE_RTC6705
    cmsx_vtxBand = masterConfig.vtx_channel / 8;
    cmsx_vtxChannel = masterConfig.vtx_channel % 8 + 1;
#endif // USE_RTC6705
}

static void cmsx_Vtx_ConfigWriteback(void)
{
#ifdef VTX
    masterConfig.vtx_band = cmsx_vtxBand;
    masterConfig.vtx_channel = cmsx_vtxChannel - 1;
#endif // VTX

#ifdef USE_RTC6705
    masterConfig.vtx_channel = cmsx_vtxBand * 8 + cmsx_vtxChannel - 1;
#endif // USE_RTC6705
}

static long cmsx_Vtx_onEnter(void)
{
    cmsx_Vtx_FeatureRead();
    cmsx_Vtx_ConfigRead();

    return 0;
}

static long cmsx_Vtx_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    cmsx_Vtx_ConfigWriteback();

    return 0;
}

#ifdef VTX
static OSD_UINT8_t entryVtxMode =  {&masterConfig.vtx_mode, 0, 2, 1};
static OSD_UINT16_t entryVtxMhz =  {&masterConfig.vtx_mhz, 5600, 5950, 1};
#endif // VTX

static OSD_Entry cmsx_menuVtxEntries[] =
{
    {"--- VTX ---", OME_Label, NULL, NULL, 0},
    {"ENABLED", OME_Bool, NULL, &cmsx_featureVtx, 0},
#ifdef VTX
    {"VTX MODE", OME_UINT8, NULL, &entryVtxMode, 0},
    {"VTX MHZ", OME_UINT16, NULL, &entryVtxMhz, 0},
#endif // VTX
    {"BAND", OME_TAB, NULL, &entryVtxBand, 0},
    {"CHANNEL", OME_UINT8, NULL, &entryVtxChannel, 0},
#ifdef USE_RTC6705
    {"LOW POWER", OME_Bool, NULL, &masterConfig.vtx_power, 0},
#endif // USE_RTC6705
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuVtx = {
    .GUARD_text = "MENUVTX",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_Vtx_onEnter,
    .onExit= cmsx_Vtx_onExit,
    .onGlobalExit = cmsx_Vtx_FeatureWriteback,
    .entries = cmsx_menuVtxEntries
};

#endif // VTX || USE_RTC6705
#endif // CMS
