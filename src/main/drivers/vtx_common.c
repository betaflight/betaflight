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

bool vtxCommonDeviceIsReady(const vtxDevice_t *vtxDevice)
{
    if (vtxDevice && vtxDevice->vTable->isReady) {
        return vtxDevice->vTable->isReady(vtxDevice);
    }

    return false;
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

const char *vtxCommonLookupBandName(const vtxDevice_t *vtxDevice, int band)
{
    if (vtxDevice) {
        return vtxDevice->bandNames[band];
    } else {
        return "?";
    }
}

char vtxCommonLookupBandLetter(const vtxDevice_t *vtxDevice, int band)
{
    if (vtxDevice) {
        return vtxDevice->bandLetters[band];
    } else {
        return '?';
    }
}

const char *vtxCommonLookupChannelName(const vtxDevice_t *vtxDevice, int channel)
{
    if (vtxDevice) {
        return vtxDevice->channelNames[channel];
    } else {
        return "?";
    }
}

// XXX FIXME Size of a band in the frequency table is now fixed at
// VTX_SETTINGS_MAX_CHANNEL (or VTX_TABLE_MAX_CHANNELS).
// Size constant should be consolidated soon or later.

//Converts frequency (in MHz) to band and channel values.
bool vtxCommonLookupBandChan(const vtxDevice_t *vtxDevice, uint16_t freq, uint8_t *pBand, uint8_t *pChannel)
{
    if (vtxDevice) {
        // Use reverse lookup order so that 5880Mhz
        // get Raceband 7 instead of Fatshark 8.
        for (int band = vtxDevice->capability.bandCount - 1 ; band >= 0 ; band--) {
            for (int channel = 0 ; channel < vtxDevice->capability.channelCount ; channel++) {
                if (vtxDevice->frequencyTable[band * VTX_SETTINGS_MAX_CHANNEL + channel] == freq) {
                    *pBand = band + 1;
                    *pChannel = channel + 1;
                    return true;
                }
            }
        }
    }

    *pBand = 0;
    *pChannel = 0;

    return false;
}

//Converts band and channel values to a frequency (in MHz) value.
// band:  Band value (1 to 5).
// channel:  Channel value (1 to 8).
// Returns frequency value (in MHz), or 0 if band/channel out of range.
uint16_t vtxCommonLookupFrequency(const vtxDevice_t *vtxDevice, int band, int channel)
{
    if (vtxDevice) {
        if (band > 0 && band <= vtxDevice->capability.bandCount &&
                              channel > 0 && channel <= vtxDevice->capability.channelCount) {
            return vtxDevice->frequencyTable[(band - 1) * VTX_SETTINGS_MAX_CHANNEL + (channel - 1)];
        }
    }

    return 0;
}

const char *vtxCommonLookupPowerName(const vtxDevice_t *vtxDevice, int index)
{
    if (vtxDevice) {
        return vtxDevice->powerNames[index];
    } else {
        return "?";
    }
}

uint16_t vtxCommonLookupPowerValue(const vtxDevice_t *vtxDevice, int index)
{
    if (vtxDevice) {
        return vtxDevice->powerValues[index];
    } else {
        return 0;
    }
}
#endif
