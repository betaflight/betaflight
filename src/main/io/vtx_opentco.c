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
#include "cms/cms_menu_vtx_opentco.h"

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

//const char *vtxOpentcoPowerNameDisabled = "";




// power names are 3 chars (+ zero termination char)
char vtxOpentcoPowerNames[OPENTCO_VTX_MAX_POWER_COUNT+1][3 + 1];

char *vtxOpentcoSupportedPowerNames[OPENTCO_VTX_MAX_POWER_COUNT+1] = {
    &vtxOpentcoPowerNames[0][0],
    &vtxOpentcoPowerNames[1][0],
    &vtxOpentcoPowerNames[2][0],
    &vtxOpentcoPowerNames[3][0],
    &vtxOpentcoPowerNames[4][0],
    &vtxOpentcoPowerNames[5][0],
    &vtxOpentcoPowerNames[6][0],
    &vtxOpentcoPowerNames[7][0],
    &vtxOpentcoPowerNames[8][0],
    &vtxOpentcoPowerNames[9][0]
};

// channel names are 1 chars (+ zero termination char)
char vtxOpentcoChannelNames[OPENTCO_VTX_MAX_CHANNEL_COUNT+1][1 + 1];

char *vtxOpentcoSupportedChannelNames[OPENTCO_VTX_MAX_CHANNEL_COUNT+1] = {
    &vtxOpentcoChannelNames[0][0],
    &vtxOpentcoChannelNames[1][0],
    &vtxOpentcoChannelNames[2][0],
    &vtxOpentcoChannelNames[3][0],
    &vtxOpentcoChannelNames[4][0],
    &vtxOpentcoChannelNames[5][0],
    &vtxOpentcoChannelNames[6][0],
    &vtxOpentcoChannelNames[7][0],
    &vtxOpentcoChannelNames[8][0],
};

// band names are 8 chars (+ zero termination char)
char vtxOpentcoBandNames[OPENTCO_VTX_MAX_BAND_COUNT+1][8 + 1];

char *vtxOpentcoSupportedBandNames[OPENTCO_VTX_MAX_BAND_COUNT+1] = {
    &vtxOpentcoBandNames[0][0],
    &vtxOpentcoBandNames[1][0],
    &vtxOpentcoBandNames[2][0],
    &vtxOpentcoBandNames[3][0],
    &vtxOpentcoBandNames[4][0],
    &vtxOpentcoBandNames[5][0],
    &vtxOpentcoBandNames[6][0],
    &vtxOpentcoBandNames[7][0],
    &vtxOpentcoBandNames[8][0]
};



static uint16_t vtxOpentcoSupportedPower;
static bool vtxOpentcoPitmodeActive;

static vtxVTable_t opentcoVTable;    // Forward
static vtxDevice_t vtxOpentco = {
    .vTable = &opentcoVTable,
    .capability.bandCount    = 0,  // will be updated after device query
    .capability.channelCount = 0,  // will be updated after device query
    .capability.powerCount   = 0,  // will be updated after device query
    // band, channel, and power names will be queried from device
    .bandNames    = (char **)vtxOpentcoSupportedBandNames,
    .channelNames = (char **)vtxOpentcoSupportedChannelNames,
    .powerNames   = (char **)vtxOpentcoSupportedPowerNames,
    .cmsMenu = &cmsx_menuVtxOpenTCO
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


static bool vtxOpentcoQueryRegister(uint8_t reg, char **descr, uint8_t *max_descr, uint8_t max_descr_strlen)
{
    // pre init to none
    for (uint32_t i = 0; i < *max_descr; i++) {
        descr[i][0] = 0;
    }

    // fetch register description
    if (!opentcoWriteRegisterUint16(device, reg,  0)) {
        return false;
    }

    // fetch all available power indices:
    char result[OPENTCO_MAX_DATA_LENGTH];
    uint16_t current_index;
    if (!opentcoReadRegisterString(device, reg, &current_index, result)) {
        // failed to fetch supported power register, this is bad..
        return false;
    }

    // fine, sucessfully retrieved string
    // should be something like "DESC:VAL1|VAL2|VAL3|"

    // extract the register name, start at beginning:
    char *begin = result;
    char *end = strchr(begin, ':');
    if ((*begin == 0) || (end == NULL)) {
        // failed to extract register name
        return false;
    }
    //
    // register name is now [begin,end], ignore for now!
    //
    begin = end + 1;

    // parse and extract supported values
    uint32_t array_idx = 0;
    end = strchr(begin, '|');
    while ((*begin != 0) && (end != NULL) && (array_idx < (*max_descr))) {
        // found next occurance at [begin, end]
        // copy no more than n allowed characters:
        strncpy(descr[array_idx], begin, max_descr_strlen);
        // add zero temination
        descr[array_idx][max_descr_strlen] = 0;
        // search for next
        array_idx++;
        begin = end + 1;
        end   = strchr(begin, '|');
    }

    if (!array_idx) {
        // no allowed indices could be extracted, error
        return false;
    }

    // store correct max value
    *max_descr = array_idx;

    return true;
}


static bool vtxOpentcoQuerySupportedFeatures(void)
{
    // fetch power count, start with max allowed power count, will be set to correct value on sucess
    vtxOpentco.capability.powerCount = OPENTCO_VTX_MAX_POWER_COUNT;
    if (!vtxOpentcoQueryRegister(OPENTCO_VTX_REGISTER_POWER, &vtxOpentcoSupportedPowerNames[1], &vtxOpentco.capability.powerCount, 3)){
        return false;
    }
    // add bf specific [0] entry:
    strcpy(vtxOpentcoPowerNames[0], "-");

    // channels
    vtxOpentco.capability.channelCount = OPENTCO_VTX_MAX_CHANNEL_COUNT;
    if (!vtxOpentcoQueryRegister(OPENTCO_VTX_REGISTER_CHANNEL, &vtxOpentcoSupportedChannelNames[1], &vtxOpentco.capability.channelCount, 1)){
        return false;
    }
    // add bf specific [0] entry:
    strcpy(vtxOpentcoChannelNames[0], "-");

    // bands
    vtxOpentco.capability.bandCount = OPENTCO_VTX_MAX_BAND_COUNT;
    if (!vtxOpentcoQueryRegister(OPENTCO_VTX_REGISTER_BAND, &vtxOpentcoSupportedBandNames[1], &vtxOpentco.capability.bandCount, 8)){
        return false;
    }
    // add bf specific [0] entry:
    strcpy(vtxOpentcoBandNames[0], "---");

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

// channel and band are 1 origin (channel=0 -> "1", .., channel=7 -> "8")
static bool vtxOpentcoSetBandAndChannel(uint8_t band, uint8_t channel)
{
    if (band && channel) {
        // FIXME: add some security measures like writing to a second reg to enable freq changes!

        // backup current enable state
        uint16_t status;
        if (!opentcoReadRegisterUint16(device, OPENTCO_VTX_REGISTER_STATUS, &status)){
            // failed to fetch status
            return false;
        }

        // disable vtx
        if (!opentcoWriteRegisterUint16(device, OPENTCO_VTX_REGISTER_STATUS, status ^ OPENTCO_VTX_STATUS_ENABLE)) {
            // failed to disable vtx, abort
            return false;
        }

        // bf uses band 0 (none) ... N+1 -> correct this here by substracting 1
        // set band
        if (!opentcoWriteRegisterUint16(device, OPENTCO_VTX_REGISTER_BAND, band - 1)){
            // failed to store setting
            return false;
        }
        // bf uses channel 0 (none) ... N+1 -> correct this here by substracting 1
        if (!opentcoWriteRegisterUint16(device, OPENTCO_VTX_REGISTER_CHANNEL, channel - 1)){
            // failed to store setting
            return false;
        }

        // config suceeded, store settings
        vtxDeviceConfigMutable()->band = band;
        vtxDeviceConfigMutable()->channel = channel;

        // re-enable vtx
        if (!opentcoWriteRegisterUint16(device, OPENTCO_VTX_REGISTER_STATUS, status)) {
            // failed to disable vtx, abort
            return false;
        }
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
    if (!opentcoWriteRegisterUint16(device, OPENTCO_VTX_REGISTER_POWER, index)) {
        // failed
        return false;
    }

    // sucess, store value
    vtxDeviceConfigMutable()->powerIndex = index;

    return true;
}

static bool vtxOpentcoSetPitMode(uint8_t onoff)
{
    uint16_t value = OPENTCO_VTX_STATUS_ENABLE;
    if (onoff) {
        // activate pitmode
        value |= OPENTCO_VTX_STATUS_PITMODE;
    }

    if (!opentcoWriteRegisterUint16(device, OPENTCO_VTX_REGISTER_STATUS, value)) {
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
    if (!vtxOpentcoSetBandAndChannel(vtxDeviceConfig()->band, vtxDeviceConfig()->channel)) return false;

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
    .getPitMode = vtxOpentcoGetPitMode
};

#endif // VTX_COMMON
