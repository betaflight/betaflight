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

/* Created by jflyper */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#if defined(VTX_RTC6705) && defined(VTX_CONTROL)
#include "build/build_config.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "common/printf.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/system.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_rtc6705.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/vtx_rtc6705.h"
#include "io/vtx_string.h"

#include "build/debug.h"

bool canUpdateVTX(void);

PG_REGISTER_WITH_RESET_TEMPLATE(vtxRTC6705Config_t, vtxRTC6705Config, PG_VTX_RTC6705_CONFIG, 0);

PG_RESET_TEMPLATE(vtxRTC6705Config_t, vtxRTC6705Config,
    .band = 4,    //Fatshark/Airwaves
    .channel = 1, //CH1
    .power = VTX_RTC6705_DEFAULT_POWER
);

#define WAIT_FOR_VTX while (!canUpdateVTX()) {}

#if defined(CMS) || defined(VTX_COMMON)
#ifdef RTC6705_POWER_PIN
static const char * const rtc6705PowerNames[RTC6705_POWER_COUNT] = {
    "---", "25 ", "200",
};
#else
static const char * const rtc6705PowerNames[RTC6705_POWER_COUNT] = {
    "25 ", "200",
};
#endif
#endif

#ifdef VTX_COMMON
static vtxVTable_t rtc6705VTable;    // Forward
static vtxDevice_t vtxRTC6705 = {
    .vTable = &rtc6705VTable,
    .capability.bandCount = 5,
    .capability.channelCount = 8,
    .capability.powerCount = RTC6705_POWER_COUNT,
    .bandNames = (char **)vtx58BandNames,
    .channelNames = (char **)vtx58ChannelNames,
    .powerNames = (char **)rtc6705PowerNames,
};
#endif

bool vtxRTC6705Init(void)
{
    vtxCommonRegisterDevice(&vtxRTC6705);

    return true;
}

static void vtxRTC6705Configure(void)
{
    rtc6705SetRFPower(vtxRTC6705.powerIndex - 1);
    rtc6705SetBandAndChannel(vtxRTC6705.band - 1, vtxRTC6705.channel - 1);
}

#ifdef RTC6705_POWER_PIN
static void vtxRTC6705EnableAndConfigure(void)
{
    WAIT_FOR_VTX;

    rtc6705Enable();

    delay(RTC6705_BOOT_DELAY);

    vtxRTC6705Configure();
}
#endif

void vtxRTC6705Process(uint32_t now)
{
    UNUSED(now);

    static bool configured = false;
    if (!configured) {
        vtxRTC6705.band = vtxRTC6705Config()->band;
        vtxRTC6705.channel = vtxRTC6705Config()->channel;
        vtxRTC6705.powerIndex = vtxRTC6705Config()->power;

#ifdef RTC6705_POWER_PIN
        if (vtxRTC6705.powerIndex > 0) {
            vtxRTC6705EnableAndConfigure();
        } else {
            rtc6705Disable();
        }
#else
        vtxRTC6705Configure();
#endif

        configured = true;
    }
}

#ifdef VTX_COMMON
// Interface to common VTX API

vtxDevType_e vtxRTC6705GetDeviceType(void)
{
    return VTXDEV_RTC6705;
}

bool vtxRTC6705IsReady(void)
{
    return true;
}

void vtxRTC6705SetBandAndChannel(uint8_t band, uint8_t channel)
{
    WAIT_FOR_VTX;

    if (band && channel) {
        if (vtxRTC6705.powerIndex > 0) {
            rtc6705SetBandAndChannel(band - 1, channel - 1);
        }

        vtxRTC6705.band = band;
        vtxRTC6705.channel = channel;

        vtxRTC6705ConfigMutable()->band = band;
        vtxRTC6705ConfigMutable()->channel = channel;
    }
}

void vtxRTC6705SetPowerByIndex(uint8_t index)
{
    WAIT_FOR_VTX;

    vtxRTC6705ConfigMutable()->power = index;

#ifdef RTC6705_POWER_PIN
    if (index == 0) {
        // power device off

        if (vtxRTC6705.powerIndex > 0) {
            // on, power it off
            vtxRTC6705.powerIndex = index;
            rtc6705Disable();
            return;
        } else {
            // already off
        }
    } else {
        // change rf power and maybe turn the device on first
        if (vtxRTC6705.powerIndex == 0) {
            // if it's powered down, power it up, wait and configure channel, band and power.
            vtxRTC6705.powerIndex = index;
            vtxRTC6705EnableAndConfigure();
            return;
        } else {
            // if it's powered up, just set the rf power
            vtxRTC6705.powerIndex = index;
            rtc6705SetRFPower(index);
        }
    }
#else
    vtxRTC6705.powerIndex = index;
    rtc6705SetRFPower(index);
#endif
}

void vtxRTC6705SetPitMode(uint8_t onoff)
{
    UNUSED(onoff);
    return;
}

bool vtxRTC6705GetBandAndChannel(uint8_t *pBand, uint8_t *pChannel)
{
    *pBand = vtxRTC6705.band;
    *pChannel = vtxRTC6705.channel;
    return true;
}

bool vtxRTC6705GetPowerIndex(uint8_t *pIndex)
{
    *pIndex = vtxRTC6705.powerIndex;
    return true;
}

bool vtxRTC6705GetPitMode(uint8_t *pOnOff)
{
    UNUSED(pOnOff);
    return false;
}

static vtxVTable_t rtc6705VTable = {
    .process = vtxRTC6705Process,
    .getDeviceType = vtxRTC6705GetDeviceType,
    .isReady = vtxRTC6705IsReady,
    .setBandAndChannel = vtxRTC6705SetBandAndChannel,
    .setPowerByIndex = vtxRTC6705SetPowerByIndex,
    .setPitMode = vtxRTC6705SetPitMode,
    .getBandAndChannel = vtxRTC6705GetBandAndChannel,
    .getPowerIndex = vtxRTC6705GetPowerIndex,
    .getPitMode = vtxRTC6705GetPitMode,
};
#endif // VTX_COMMON

#ifdef CMS

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

#endif // VTX_RTC6705
