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

char *vtxOpentcoSupportedPowerNames[OPENTCO_VTX_POWER_COUNT] = {
    "---", "5  ", "10 ", "25 ", "100", "200", "500", "600", "800"
};

static uint16_t vtxOpentcoSupportedPower;
static bool vtxOpentcoPitmodeActive;

static vtxVTable_t opentcoVTable;    // Forward
static vtxDevice_t vtxOpentco = {
    .vTable = &opentcoVTable,
    .capability.bandCount = 5,
    .capability.channelCount = 8,
    .capability.powerCount = OPENTCO_VTX_POWER_COUNT,
    .bandNames = (char **)vtx58BandNames,
    .channelNames = (char **)vtx58ChannelNames,
    .powerNames = (char **)vtxOpentcoSupportedPowerNames,
};

static opentcoDevice_t vtxOpentcoDevice;
static opentcoDevice_t *device = &vtxOpentcoDevice;

static bool vtxOpentcoQuerySupportedFeatures(void);

vtxDevice_t *vtxOpentcoInit(void)
{
    // configure device
    device->id = OPENTCO_DEVICE_VTX;

    // open serial port
    if (!opentcoInit(device)) {
        // no device found
        return NULL;
    }

    // fetch supported power
    if (!vtxOpentcoQuerySupportedFeatures()) return NULL;

    // register device
    return &vtxOpentco;
}

static bool vtxOpentcoQuerySupportedFeatures(void)
{
    // fetch available power rates
    if (!opentcoReadRegister(device, OPENTCO_VTX_REGISTER_SUPPORTED_POWER,  &vtxOpentcoSupportedPower)) {
        // failed to fetch supported power register, this is bad..
        return false;
    }

    // start with entry 0
    uint32_t powerIndex = 0;

    for (uint32_t i = 0; i < OPENTCO_VTX_POWER_COUNT; i++) {
        // check for device support
        if (!(vtxOpentcoSupportedPower & (1 << i))) {
            // this index is not supported, disable
            vtxOpentcoSupportedPowerNames[powerIndex][0] = 0;
        }
    }

    // store maximum power index
    vtxOpentco.capability.powerCount = powerIndex;

    return true;
}

static void vtxOpentcoProcess(uint32_t now)
{
    UNUSED(now);

    static bool configured = false;
    if (!configured) {
        // configured is set to true only when configure suceeded
        configured  = vtxOpentcoConfigure();
    }
}

// Interface to common VTX API
/*static vtxDevType_e vtxOpentcoGetDeviceType(void)
{
    return VTXDEV_OPENTCO;
}*/

static bool vtxOpentcoIsReady(void)
{
    return true;
}

static bool vtxOpentcoSetBandAndChannel(uint8_t band, uint8_t channel)
{
    if (band && channel) {
        // FIXME: add some security measures like writing to a second reg to enable freq changes!
        if (!opentcoWriteRegister(device, OPENTCO_VTX_REGISTER_BAND, band - 1)){
            // failed to store setting
            return false;
        }
        if (!opentcoWriteRegister(device, OPENTCO_VTX_REGISTER_CHANNEL, channel - 1)){
            // failed to store setting
            return false;
        }

        // config suceeded, store settings
        vtxDeviceConfigMutable()->band = band;
        vtxDeviceConfigMutable()->channel = channel;
    }

    // all fine
    return true;
}

static bool vtxOpentcoSetPowerByIndex(uint8_t index)
{
    // check if this is supported:
    while((!(vtxOpentcoSupportedPower & (1 << index))) && (index > 0)) {
        // not supported, fall back to a smaller value
        index--;
    }

    // try to store the setting:
    if (!opentcoWriteRegister(device, OPENTCO_VTX_REGISTER_POWER, index)) {
        // failed
        return false;
    }

    // sucess, store value
    vtxDeviceConfigMutable()->powerIndex = index;

    return true;
}

static bool vtxOpentcoSetPitMode(uint8_t onoff)
{
    // pitmode supported?
    if (!(vtxOpentcoSupportedPower & OPENTCO_VTX_POWER_PITMODE)){
        // set value and return true anyway (this is not critical)
        return true;
    }


    uint16_t value = OPENTCO_VTX_STATUS_ENABLE;
    if (onoff) {
        // activate pitmode
        value |= OPENTCO_VTX_STATUS_PITMODE;
    }

    if (!opentcoWriteRegister(device, OPENTCO_VTX_REGISTER_STATUS, value)) {
        // failed!
        return false;
    }

    // setting was stored sucessfully
    vtxOpentcoPitmodeActive = onoff;

    return true;
}

static bool vtxOpentcoGetBandAndChannel(uint8_t *pBand, uint8_t *pChannel)
{
    *pBand = vtxDeviceConfig()->band;
    *pChannel = vtxDeviceConfig()->channel;
    return true;
}

static bool vtxOpentcoGetPowerIndex(uint8_t *pIndex)
{
    *pIndex = vtxDeviceConfig()->powerIndex;
    return true;
}

static bool vtxOpentcoGetPitMode(uint8_t *pOnOff)
{
    *pOnOff = vtxOpentcoPitmodeActive;
    return true;
}


bool vtxOpentcoConfigure(void)
{
    // transfer all properties to device
    if (!vtxOpentcoSetPitMode(vtxDeviceConfig()->pitMode)) return false;
    if (!vtxOpentcoSetPowerByIndex(vtxDeviceConfig()->powerIndex)) return false;
    if (!vtxOpentcoSetBandAndChannel(vtxDeviceConfig()->band - 1, vtxDeviceConfig()->channel - 1)) return false;

    // sucess
    return true;
}

static vtxVTable_t opentcoVTable = {
    .process = vtxOpentcoProcess,
    //.getDeviceType = vtxOpentcoGetDeviceType,
    .isReady = vtxOpentcoIsReady,
    .setBandAndChannel = vtxOpentcoSetBandAndChannel,
    .setPowerByIndex = vtxOpentcoSetPowerByIndex,
    .setPitMode = vtxOpentcoSetPitMode,
    .getBandAndChannel = vtxOpentcoGetBandAndChannel,
    .getPowerIndex = vtxOpentcoGetPowerIndex,
    .getPitMode = vtxOpentcoGetPitMode,
};

#endif // VTX_COMMON
