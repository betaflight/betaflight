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
#include "build/debug.h"

#if defined(VTX_COMMON)

#include "vtx_common.h"

vtxDevice_t *vtxDevice = NULL;

void vtxCommonInit(void)
{
}

// Whatever registered last will win

void vtxCommonRegisterDevice(vtxDevice_t *pDevice)
{
    vtxDevice = pDevice;
}

bool vtxCommonDeviceRegistered(void)
{
    return vtxDevice;
}

void vtxCommonProcess(uint32_t currentTimeUs)
{
    if (!vtxDevice)
        return;

    if (vtxDevice->vTable->process)
        vtxDevice->vTable->process(currentTimeUs);
}

vtxDevType_e vtxCommonGetDeviceType(void)
{
    if (!vtxDevice || !vtxDevice->vTable->getDeviceType)
        return VTXDEV_UNKNOWN;

    return vtxDevice->vTable->getDeviceType();
}

// band and channel are 1 origin
void vtxCommonSetBandAndChannel(uint8_t band, uint8_t channel)
{
    if (!vtxDevice)
        return;

    if ((band > vtxDevice->capability.bandCount) || (channel > vtxDevice->capability.channelCount))
        return;

    if (vtxDevice->vTable->setBandAndChannel)
        vtxDevice->vTable->setBandAndChannel(band, channel);
}

// index is zero origin, zero = power off completely
void vtxCommonSetPowerByIndex(uint8_t index)
{
    if (!vtxDevice)
        return;

    if (index > vtxDevice->capability.powerCount)
        return;

    if (vtxDevice->vTable->setPowerByIndex)
        vtxDevice->vTable->setPowerByIndex(index);
}

// on = 1, off = 0
void vtxCommonSetPitMode(uint8_t onoff)
{
    if (!vtxDevice)
        return;

    if (vtxDevice->vTable->setPitMode)
        vtxDevice->vTable->setPitMode(onoff);
}

void vtxCommonSetFrequency(uint16_t freq)
{
    if (!vtxDevice) {
        return;
    }
    if (vtxDevice->vTable->setFrequency) {
        vtxDevice->vTable->setFrequency(freq);
    }
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

bool vtxCommonGetFrequency(uint16_t *pFreq)
{
    if (!vtxDevice) {
        return false;
    }
    if (vtxDevice->vTable->getFrequency) {
        return vtxDevice->vTable->getFrequency(pFreq);
    } else {
        return false;
    }
}

bool vtxCommonGetDeviceCapability(vtxDeviceCapability_t *pDeviceCapability)
{
    if (!vtxDevice)
        return false;

    memcpy(pDeviceCapability, &vtxDevice->capability, sizeof(vtxDeviceCapability_t));
    return true;
}
#endif
