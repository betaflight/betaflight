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

#if defined(USE_VTX_COMMON)

#include "common/time.h"
#include "drivers/vtx_common.h"


static vtxDevice_t *vtxDevice = NULL;

void vtxCommonInit(void)
{
}

void vtxCommonSetDevice(vtxDevice_t *pDevice)
{
    vtxDevice = pDevice;
}

vtxDevice_t *vtxCommonDevice(void)
{
    return vtxDevice;
}

vtxDevType_e vtxCommonGetDeviceType(const vtxDevice_t *vtxDevice)
{
    if (!vtxDevice) {
        return VTXDEV_UNKNOWN;
    }
    return vtxDevice->vTable->getDeviceType(vtxDevice);
}

void vtxCommonProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs)
{
    if (vtxDevice) {
        vtxDevice->vTable->process(vtxDevice, currentTimeUs);
    }
}

// band and channel are 1 origin
void vtxCommonSetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
    if (band <= vtxDevice->capability.bandCount && channel <= vtxDevice->capability.channelCount) {
        vtxDevice->vTable->setBandAndChannel(vtxDevice, band, channel);
    }
}

// index is zero origin, zero = power off completely
void vtxCommonSetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    if (index <= vtxDevice->capability.powerCount) {
        vtxDevice->vTable->setPowerByIndex(vtxDevice, index);
    }
}

// on = 1, off = 0
void vtxCommonSetPitMode(vtxDevice_t *vtxDevice, uint8_t onOff)
{
    vtxDevice->vTable->setPitMode(vtxDevice, onOff);
}

void vtxCommonSetFrequency(vtxDevice_t *vtxDevice, uint16_t frequency)
{
    vtxDevice->vTable->setFrequency(vtxDevice, frequency);
}

bool vtxCommonGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    return vtxDevice->vTable->getBandAndChannel(vtxDevice, pBand, pChannel);
}

bool vtxCommonGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    return vtxDevice->vTable->getPowerIndex(vtxDevice, pIndex);
}

bool vtxCommonGetPitMode(const vtxDevice_t *vtxDevice, uint8_t *pOnOff)
{
    return vtxDevice->vTable->getPitMode(vtxDevice, pOnOff);
}

bool vtxCommonGetFrequency(const vtxDevice_t *vtxDevice, uint16_t *pFrequency)
{
    return vtxDevice->vTable->getFrequency(vtxDevice, pFrequency);
}

bool vtxCommonGetDeviceCapability(const vtxDevice_t *vtxDevice, vtxDeviceCapability_t *pDeviceCapability)
{
    memcpy(pDeviceCapability, &vtxDevice->capability, sizeof(vtxDeviceCapability_t));
    return true;
}

#endif
