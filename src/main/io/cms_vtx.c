#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#include "io/cms.h"
#include "io/cms_types.h"
#include "io/cms_vtx.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#ifdef CMS

#if defined(VTX) || defined(USE_RTC6705)

uint8_t cmsx_featureVtx = 0, cmsx_vtxBand, cmsx_vtxChannel;

static const char * const vtxBandNames[] = {
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
};

OSD_TAB_t entryVtxBand = {&cmsx_vtxBand,4,&vtxBandNames[0]};
OSD_UINT8_t entryVtxChannel =  {&cmsx_vtxChannel, 1, 8, 1};

#ifdef VTX
OSD_UINT8_t entryVtxMode =  {&masterConfig.vtx_mode, 0, 2, 1};
OSD_UINT16_t entryVtxMhz =  {&masterConfig.vtx_mhz, 5600, 5950, 1};
#endif // VTX

OSD_Entry cmsx_menuVtx[] =
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

void cmsx_Vtx_FeatureRead(void)
{
    cmsx_featureVtx = feature(FEATURE_VTX) ? 1 : 0;
}

void cmsx_Vtx_FeatureWriteback(void)
{
    if (cmsx_featureVtx)
        featureSet(FEATURE_VTX);
    else
        featureClear(FEATURE_VTX);
}

void cmsx_Vtx_ConfigRead(void)
{
#ifdef VTX
    cmsx_vtxBand = masterConfig.vtxBand;
    cmsx_vtxChannel = masterConfig.vtx_channel + 1;
#endif // VTX

#ifdef USE_RTC6705
    cmsx_vtxBand = masterConfig.vtx_channel / 8;
    cmsx_vtxChannel = masterConfig.vtx_channel % 8 + 1;
#endif // USE_RTC6705
}

void cmsx_Vtx_ConfigWriteback(void)
{
#ifdef VTX
    masterConfig.vtxBand = cmsx_vtxBand;
    masterConfig.vtx_channel = cmsx_vtxChannel - 1;
#endif // VTX

#ifdef USE_RTC6705
    masterConfig.vtx_channel = cmsx_vtxBand * 8 + cmsx_vtxChannel - 1;
#endif // USE_RTC6705
}

#endif // VTX || USE_RTC6705

#endif // CMS
