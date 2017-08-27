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
#include <ctype.h>
#include <string.h>

#include "platform.h"
#include "config/parameter_group_ids.h"
#include "config/parameter_group.h"

#include "build/debug.h"
#define VTX_COMMON
#if defined(VTX_COMMON)

#include "vtx_common.h"

PG_REGISTER_WITH_RESET_TEMPLATE(vtxDeviceConfig_t, vtxDeviceConfig, PG_VTX_DEVICE_CONFIG, 0);

#ifdef VTX_RTC6705
    // preselect on board vtx
    #define VTX_DEVICE_DEFAULT          VTX_DEVICE_RTC6705
#else
   #define VTX_DEVICE_DEFAULT          VTX_DEVICE_NONE
#endif

PG_RESET_TEMPLATE(vtxDeviceConfig_t, vtxDeviceConfig,
    .device = VTX_DEVICE_DEFAULT,
    .band = 4,    //Fatshark/Airwaves
    .channel = 1, //CH1
    .powerIndex = 1
);

vtxDevice_t *vtxDevice = NULL;

void vtxCommonInit(void)
{
}

// Whatever registered last will win

void vtxCommonRegisterDevice(vtxDevice_t *pDevice)
{
    vtxDevice = pDevice;
}

void vtxCommonProcess(uint32_t currentTimeUs)
{
    if (!vtxDevice)
        return;

    if (vtxDevice->vTable->process)
        vtxDevice->vTable->process(currentTimeUs);
}

vtxDevice_e vtxCommonGetDeviceType(void)
{
    if (!vtxDevice) {
        return VTX_DEVICE_UNKNOWN;
    }

    return vtxDeviceConfig()->device;
}

bool vtxCommonGetBandName(uint8_t band, char **name)
{
    if (band > vtxDevice->capability.bandCount) {
        return false;
    }
    *name = vtxDevice->bandNames[band];
    return true;
}

bool vtxCommonGetChannelName(uint8_t ch, char **name)
{
    if (ch > vtxDevice->capability.channelCount) {
        return false;
    }
    *name = vtxDevice->channelNames[ch];
    return true;
}

bool vtxCommonGetPowerName(uint8_t index, char **name)
{
    if (index > vtxDevice->capability.powerCount) {
        return false;
    }
    *name = vtxDevice->powerNames[index];
    return true;
}

// band and channel are 1 origin
bool vtxCommonSetBandAndChannel(uint8_t band, uint8_t channel)
{
    if (!vtxDevice)
        return false;

    if ((band > vtxDevice->capability.bandCount) || (channel > vtxDevice->capability.channelCount))
        return false;

    if (vtxDevice->vTable->setBandAndChannel)
        return vtxDevice->vTable->setBandAndChannel(band, channel);

    return false;
}

// index is zero origin, zero = power off completely
bool vtxCommonSetPowerByIndex(uint8_t index)
{
    if (!vtxDevice)
        return false;

    if (index > vtxDevice->capability.powerCount)
        return false;

    if (vtxDevice->vTable->setPowerByIndex)
        return vtxDevice->vTable->setPowerByIndex(index);

    return false;
}

// on = 1, off = 0
bool vtxCommonSetPitMode(uint8_t onoff)
{
    if (!vtxDevice)
        return false;

    if (vtxDevice->vTable->setPitMode)
        return vtxDevice->vTable->setPitMode(onoff);

    return false;
}

bool vtxCommonGetBandAndChannel(uint8_t *pBand, uint8_t *pChannel)
{
    if (!vtxDevice)
        return false;

    if (vtxDevice->vTable->getBandAndChannel)
        return vtxDevice->vTable->getBandAndChannel(pBand, pChannel);
    else
        return false;
}

bool vtxCommonGetPowerIndex(uint8_t *pIndex)
{
    if (!vtxDevice)
        return false;

    if (vtxDevice->vTable->getPowerIndex)
        return vtxDevice->vTable->getPowerIndex(pIndex);
    else
        return false;
}

bool vtxCommonGetPitMode(uint8_t *pOnOff)
{
    if (!vtxDevice)
        return false;

    if (vtxDevice->vTable->getPitMode)
        return vtxDevice->vTable->getPitMode(pOnOff);
    else
        return false;
}

bool vtxCommonGetDeviceCapability(vtxDeviceCapability_t *pDeviceCapability)
{
    if (!vtxDevice)
        return false;

    memcpy(pDeviceCapability, &vtxDevice->capability, sizeof(vtxDeviceCapability_t));
    return true;
}
#endif
