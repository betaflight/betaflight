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

#if defined(USE_CMS) && defined(USE_VTX_BEESIGN)

#include "common/printf.h"
#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_vtx_beesign.h"

#include "drivers/vtx_common.h"

#include "fc/config.h"

#include "io/vtx_string.h"
#include "io/vtx_beesign.h"
#include "io/vtx.h"

static uint8_t bs_vtxBand;
static uint8_t bs_vtxChannel;
static uint16_t bs_vtxFreq;
static uint16_t bs_showFreq;
static uint8_t bs_vtxPower;
static uint8_t bs_vtxmode;

static uint8_t porModeStr[10] =     {"  POR MODE"};
static uint8_t porModeFREQStr[10] = {"      5584"};

static OSD_TAB_t bsEntryVtxMode =         {&bs_vtxmode, VTX_BEESIGN_MODE_COUNT - 1, &bsModeNames[0]};
static OSD_TAB_t bsEntryVtxBand =         {&bs_vtxBand, VTX_BEESIGN_BAND_COUNT - 1, &vtx58BandNames[1]};
static OSD_UINT8_t bsEntryVtxChannel =    {&bs_vtxChannel, 1, VTX_SETTINGS_CHANNEL_COUNT, 1};
static OSD_UINT16_t bsEntryVtxFreq =      {&bs_vtxFreq, VTX_BEESIGN_MIN_FREQUENCY_MHZ, VTX_BEESIGN_MAX_FREQUENCY_MHZ, 1};
static OSD_UINT16_t bsShowVtxFreq =       {&bs_showFreq, VTX_BEESIGN_MIN_FREQUENCY_MHZ, VTX_BEESIGN_MAX_FREQUENCY_MHZ, 0};
static OSD_TAB_t bsEntryVtxPower =        {&bs_vtxPower, VTX_BEESIGN_POWER_COUNT - VTX_BEESIGN_MIN_POWER, &bsPowerNames[VTX_BEESIGN_MIN_POWER]};

CMS_Menu cmsx_menuVtxBeesign; // Forward
static long bsCmsConfigMode(const OSD_Entry *self);

static void bs_Vtx_ConfigRead(void)
{
    bs_vtxChannel = vtxSettingsConfig()->channel;
    if (bs_vtxmode == 0){
        if (vtxSettingsConfig()->band == 0) {
            bs_vtxBand = 0;
        } else {
            bs_vtxBand = vtxSettingsConfig()->band - 1;
        }
        bs_showFreq = vtx58frequencyTable[bs_vtxBand][bs_vtxChannel-1];
        bs_vtxFreq = vtx58frequencyTable[bs_vtxBand][bs_vtxChannel-1];
    } else {
        bs_vtxBand = vtxSettingsConfig()->band;
        bs_vtxFreq = vtxSettingsConfig()->freq;
    }
    if (vtxSettingsConfig()->power > 1) {
        bs_vtxPower = vtxSettingsConfig()->power - VTX_BEESIGN_MIN_POWER;
    } else {
        bs_vtxPower = 0;
    }

}

static long bsCmsConfigBandByGvar(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    bs_showFreq = vtx58frequencyTable[bs_vtxBand][bs_vtxChannel-1];

    return 0;
}

static long bsCmsConfigRaceSave(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    bsSetMode(VTX_BEESIGN_VTX_RACE_MODE);
    vtxSettingsConfigMutable()->band = bs_vtxBand + 1;
    vtxSettingsConfigMutable()->channel = bs_vtxChannel;
    vtxSettingsConfigMutable()->power = bs_vtxPower + VTX_BEESIGN_MIN_POWER;
    vtxSettingsConfigMutable()->freq = bs_showFreq;

    saveConfigAndNotify();
    return 0;
}

static long bsCmsConfigManualSave(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    bsSetMode(VTX_BEESIGN_VTX_MANUAL_MODE);
    vtxSettingsConfigMutable()->band = 0;
    vtxSettingsConfigMutable()->channel = bs_vtxChannel;
    vtxSettingsConfigMutable()->power = bs_vtxPower + VTX_BEESIGN_MIN_POWER;
    vtxSettingsConfigMutable()->freq = bs_vtxFreq;

    saveConfigAndNotify();
    return 0;
}

static long bsCmsConfigPorSave(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    bsSetMode(VTX_BEESIGN_VTX_POR_MODE);
    vtxSettingsConfigMutable()->band = 0;
    vtxSettingsConfigMutable()->channel = bs_vtxChannel;
    vtxSettingsConfigMutable()->power = VTX_PWR_PIT + VTX_BEESIGN_MIN_POWER;
    vtxSettingsConfigMutable()->freq = VTX_BEESIGN_POR_FREQUENCY_MHZ;

    saveConfigAndNotify();
    return 0;
}


static long bs_Vtx_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    return 0;
}

static OSD_Entry bsCmsMenuModeEntries[] =
{
    {"--- BEESIGN MODE ---", OME_Label, NULL, NULL, 0},
    {"MODE",        OME_TAB,        NULL,                   &bsEntryVtxMode,         0},
    {"BACK",        OME_Back,       NULL,                   NULL,                   0},
    {NULL,          OME_END,        NULL,                   NULL,                   0}
};


CMS_Menu cmsx_menuVtxBsMode = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "VTXBS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit= bsCmsConfigMode,
    .entries = bsCmsMenuModeEntries
};

static OSD_Entry bsCmsMenuRaceModeEntries[] =
{
    {"--- BEESIGN RACE---", OME_Label, NULL, NULL, 0},
    {"MODE",        OME_Submenu,    cmsMenuChange,          &cmsx_menuVtxBsMode,    0},
    {"BAND",        OME_TAB,        bsCmsConfigBandByGvar,  &bsEntryVtxBand,        0},
    {"CHANNEL",     OME_UINT8,      bsCmsConfigBandByGvar,  &bsEntryVtxChannel,     0},
    {"POWER",       OME_TAB,        NULL,                   &bsEntryVtxPower,       0},
    {"(FREQ)",      OME_UINT16,     NULL,                   &bsShowVtxFreq,         DYNAMIC },
    {"SAVE",        OME_Funcall,    bsCmsConfigRaceSave,    NULL,                   0},
    {"BACK",        OME_Back,       NULL,                   NULL,                   0},
    {NULL,          OME_END,        NULL,                   NULL,                   0}
};

static OSD_Entry bsCmsMenuManualModeEntries[] =
{
    {"--- BEESIGN MAMUAL---", OME_Label, NULL, NULL, 0},
    {"MODE",        OME_Submenu,    cmsMenuChange,          &cmsx_menuVtxBsMode,    0},
    {"FREQ",        OME_UINT16,     NULL,                   &bsEntryVtxFreq,        0},
    {"POWER",       OME_TAB,        NULL,                   &bsEntryVtxPower,       0},
    {"SAVE",        OME_Funcall,    bsCmsConfigManualSave,  NULL,                   0},
    {"BACK",        OME_Back,       NULL,                   NULL,                   0},
    {NULL,          OME_END,        NULL,                   NULL,                   0}
};


static OSD_Entry bsCmsMenuPorModeEntries[] =
{
    {"--- BEESIGN POR---", OME_Label, NULL, NULL, 0},
    {"MODE",        OME_Submenu,    cmsMenuChange,          &cmsx_menuVtxBsMode,    0},
    {"(FREQ)",      OME_String,     NULL,                   &porModeFREQStr,        0},
    {"(POWER)",     OME_String,     NULL,                   &porModeStr,            0},
    {"SAVE",        OME_Funcall,    bsCmsConfigPorSave,     NULL,                   0},
    {"BACK",        OME_Back,       NULL,                   NULL,                   0},
    {NULL,          OME_END,        NULL,                   NULL,                   0}
};

static long bsCmsConfigMode(const OSD_Entry *self)
{
    UNUSED(self);
    bs_Vtx_ConfigRead();
    switch (bs_vtxmode) {
        case 0:
            cmsx_menuVtxBeesign.entries = bsCmsMenuRaceModeEntries;
            break;
        case 1:
           cmsx_menuVtxBeesign.entries = bsCmsMenuManualModeEntries;
            break;
        case 2:
            cmsx_menuVtxBeesign.entries = bsCmsMenuPorModeEntries;
            break;
        default:
            cmsx_menuVtxBeesign.entries = bsCmsMenuRaceModeEntries;
            break;
    }
    return 0;
}

static long bs_Vtx_onEnter(void)
{
    bs_vtxmode = bsDevice.mode;
    bsCmsConfigMode(NULL);
    return 0;
}

CMS_Menu cmsx_menuVtxBeesign = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "VTXBS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = bs_Vtx_onEnter,
    .onExit= bs_Vtx_onExit,
    .entries = bsCmsMenuRaceModeEntries
};

#endif // CMS
