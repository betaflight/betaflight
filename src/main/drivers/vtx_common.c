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

void vtxCommonProcess(uint32_t currentTimeUs)
{
    if (!vtxDevice)
        return;

    if (vtxDevice->vTable->process)
        vtxDevice->vTable->process(currentTimeUs);
}

vtxDevType_e vtxCommonGetDeviceType(void)
{
    if (!vtxDevice)
        return VTXDEV_UNKNOWN;

    return vtxDevice->devtype;
}

// band and chan are 1 origin
void vtxCommonSetBandChan(uint8_t band, uint8_t chan)
{
    if (!vtxDevice)
        return;

    if (vtxDevice->vTable->setBandChan)
        vtxDevice->vTable->setBandChan(band, chan);
}

// index is zero origin, zero = power off completely
void vtxCommonSetPowerByIndex(uint8_t index)
{
    if (!vtxDevice)
        return;

    if (vtxDevice->vTable->setPowerByIndex)
        vtxDevice->vTable->setPowerByIndex(index);
}

// on = 1, off = 0
void vtxCommonSetPitmode(uint8_t onoff)
{
    if (!vtxDevice)
        return;

    if (vtxDevice->vTable->setPitmode)
        vtxDevice->vTable->setPitmode(onoff);
}

bool vtxCommonGetBandChan(uint8_t *pBand, uint8_t *pChan)
{
    if (!vtxDevice)
        return false;

    if (vtxDevice->vTable->getBandChan)
        return vtxDevice->vTable->getBandChan(pBand, pChan);
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

bool vtxCommonGetPitmode(uint8_t *pOnoff)
{
    if (!vtxDevice)
        return false;

    if (vtxDevice->vTable->getPitmode)
        return vtxDevice->vTable->getPitmode(pOnoff);
    else
        return false;
}
#endif
