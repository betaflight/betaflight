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
#include <string.h>
#include <ctype.h>

#include "platform.h"

#define VTX_COMMON
#ifdef VTX_COMMON

#include "build/build_config.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"
#include "drivers/opentco.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/vtx_opentco.h"
#include "io/vtx_string.h"

#include "build/debug.h"

char *vtxOpentcoSupportedPowerNames[OPENTCO_POWER_COUNT][4];

const char * const vtxOpentcoPowerNames[OPENTCO_POWER_COUNT] = {
    "---", "5  ", "10 ", "25 ", "100", "200", "500", "800"
};

static uint16_t vtxOpentcoSupportedPower;
static bool vtxOpentcoPitmodeActive;

static vtxVTable_t opentcoVTable;    // Forward
static vtxDevice_t vtxOpentco = {
    .vTable = &rtc6705VTable,
    .capability.bandCount = 5,
    .capability.channelCount = 8,
    .capability.powerCount = OPENTCO_POWER_COUNT,
    .bandNames = (char **)vtx58BandNames,
    .channelNames = (char **)vtx58ChannelNames,
    .powerNames = (char **)vtxOpentcoSupportedPowerNames,
};


static opentcoDevice_t vtxOpentcoDevice;
static opentcoDevice_t *device = &vtxOpentcoDevice;


bool vtxOpentcoInit(void)
{
    // configure device
    device->id = OPENTCO_DEVICE_VTX;

    // open serial port
    if (!opentcoInit(device)) {
        // no device found
        return false;
    }

    // fetch supported power
    if (!vtxOpentcoQuerySupportedFeatures()) return false;

    // register device
    vtxCommonRegisterDevice(&vtxOpentco);

    return true;
}

static bool vtxOpentcoQuerySupportedFeatures()
{
    // fetch available power rates
    if (!opentcoReadRegister(device, OPENTCO_VTX_REGISTER_SUPPORTED_POWER,  &vtxOpentcoSupportedPower)) {
        // failed to fetch supported power register, this is bad..
        return false;
    }

    // pointer to update supported power levels
    vtxOpentcoSupportedPowerNames[OPENTCO_POWER_COUNT][4]
    uint32_t powerIndex = 0;

    for (uint32_t i = 0; i < OPENTCO_VTX_POWER_COUNT; i++) {
        if (vtxOpentcoSupportedPower & (1 << i)) {
            // copy string to supported feature list
            memcpy(vtxOpentcoSupportedPowerNames[powerIndex], vtxOpentcoPowerNames[i], 4);
            powerIndex++;
        }
    }

    return true;
}

bool vtxOpentcoConfigure(void)
{
    // transfer all properties to device
    if (!vtxOpentcoSetPitMode(vtxConfig()->pitMode)) return false;
    if (!vtxOpentcoSetPowerByIndex(vtxConfig()->powerIndex)) return false;
    if (!vtxOpentcoSetBandAndChannel(vtxConfig()->.band - 1, vtxConfig()->.channel - 1)) return false;

    // sucess
    return true;
}

void vtxOpentcoProcess(uint32_t now)
{
    UNUSED(now);

    static bool configured = false;
    if (!configured) {
        // configured is set to true only when configure suceeded
        configured  = vtxOpentcoConfigure();
    }
}

// Interface to common VTX API
vtxDevType_e vtxRTC6705GetDeviceType(void)
{
    return VTXDEV_OPENTCO;
}

bool vtxOpentcoIsReady(void)
{
    return true;
}

void vtxOpentcoSetBandAndChannel(uint8_t band, uint8_t channel)
{
    if (band && channel) {
        // FIXME: add some security emasures like writing to a second reg to enable freq changes!
        if (!opentcoWriteRegister(device, OPENTCO_VTX_REGISTER_BAND, band - 1)){
            // failed to store setting
            return;
        }
        if (!opentcoWriteRegister(device, OPENTCO_VTX_REGISTER_CHANNEL, channel - 1)){
            // failed to store setting
            return;
        }

        // config suceeded, store settings
        vtxConfigMutable()->band = band;
        vtxConfigMutable()->channel = channel;
    }
}

void vtxOpentcoSetPowerByIndex(uint8_t index)
{
    // check if this is supported:
    while((!(vtxOpentcoSupportedPower & (1 << index))) && (index > 0)) {
        // not supported, fall back to a smaller value
        index--;
    }

    // try to store the setting:
    if (opentcoWriteRegister(device, OPENTCO_VTX_REGISTER_POWER, index)) {
        // sucess, store value
        vtxConfigMutable()->powerIndex = index;
    }
}

void vtxOpentcoSetPitMode(uint8_t onoff)
{
    // pitmode supported?
    if (!(vtxOpentcoSupportedPower & OPENTCO_VTX_POWER_PITMODE)){
        return;
    }


    uint16_t value = OPENTCO_VTX_STATUS_ENABLE;
    if (onoff) {
        // activate pitmode
        value |= OPENTCO_VTX_STATUS_PITMODE;
    }

    if (opentcoWriteRegister(device, OPENTCO_VTX_REGISTER_STATUS, value)) {
        // setting was stored sucessfully
        vtxOpentcoPitmodeActive = onoff;
    }

    return;
}

bool vtxOpentcoGetBandAndChannel(uint8_t *pBand, uint8_t *pChannel)
{
    *pBand = vtxConfig()->band;
    *pChannel = vtxConfig()->channel;
    return true;
}

bool vtxOpentcoGetPowerIndex(uint8_t *pIndex)
{
    *pIndex = vtxConfig()->powerIndex;
    return true;
}

bool vtxOpentcoGetPitMode(uint8_t *pOnOff)
{
    *pOnOff = vtxOpentcoPitmodeActive;
    return true;
}

static vtxVTable_t rtc6705VTable = {
    .process = vtxOpentcoProcess,
    .getDeviceType =vvtxOpentcoGetDeviceType,
    .isReady = vtxOpentcoIsReady,
    .setBandAndChannel = vtxOpentcoSetBandAndChannel,
    .setPowerByIndex = vtxOpentcoSetPowerByIndex,
    .setPitMode = vtxOpentcoSetPitMode,
    .getBandAndChannel = vtxOpentcoGetBandAndChannel,
    .getPowerIndex = vtxOpentcoGetPowerIndex,
    .getPitMode = vtxOpentcoGetPitMode,
};

#endif // VTX_COMMON
