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
#include "drivers/vtx_table.h"

static vtxDevice_t *vtxDevice = NULL;
static uint8_t selectedBand = 0;
static uint8_t selectedChannel = 0;

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
    uint16_t freq = vtxCommonLookupFrequency(vtxDevice, band, channel);
    if (freq != 0) {
        selectedChannel = channel;
        selectedBand = band;
        if (vtxTableIsFactoryBand[band - 1]) {
            vtxDevice->vTable->setBandAndChannel(vtxDevice, band, channel);
        } else {
            vtxDevice->vTable->setFrequency(vtxDevice, freq);
        }
    }
}

// index is one origin, zero = unknown power level
void vtxCommonSetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    if (index <= vtxTablePowerLevels) {
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
    selectedBand = 0;
    selectedChannel = 0;
    vtxDevice->vTable->setFrequency(vtxDevice, frequency);
}

bool vtxCommonGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    bool result = vtxDevice->vTable->getBandAndChannel(vtxDevice, pBand, pChannel);
    if ((!result || (*pBand == 0 && *pChannel == 0)) && selectedBand != 0 && selectedChannel != 0
        && !vtxTableIsFactoryBand[selectedBand - 1]) {
        uint16_t freq;
        result = vtxCommonGetFrequency(vtxDevice, &freq);
        if (result) {
            vtxCommonLookupBandChan(vtxDevice, freq, pBand, pChannel);
        }
    }
    return result;
}

bool vtxCommonGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    return vtxDevice->vTable->getPowerIndex(vtxDevice, pIndex);
}

bool vtxCommonGetFrequency(const vtxDevice_t *vtxDevice, uint16_t *pFrequency)
{
    return vtxDevice->vTable->getFrequency(vtxDevice, pFrequency);
}

bool vtxCommonGetStatus(const vtxDevice_t *vtxDevice, unsigned *status)
{
    return vtxDevice->vTable->getStatus(vtxDevice, status);
}

uint8_t vtxCommonGetVTXPowerLevels(const vtxDevice_t *vtxDevice, uint16_t *levels, uint16_t *powers)
{
    return vtxDevice->vTable->getPowerLevels(vtxDevice, levels, powers);
}

const char *vtxCommonLookupBandName(const vtxDevice_t *vtxDevice, int band)
{
    if (vtxDevice && band > 0 && band <= vtxTableBandCount) {
        return vtxTableBandNames[band];
    } else {
        return "?";
    }
}

char vtxCommonLookupBandLetter(const vtxDevice_t *vtxDevice, int band)
{
    if (vtxDevice && band > 0 && band <= vtxTableBandCount) {
        return vtxTableBandLetters[band];
    } else {
        return '?';
    }
}

const char *vtxCommonLookupChannelName(const vtxDevice_t *vtxDevice, int channel)
{
    if (vtxDevice && channel > 0 && channel <= vtxTableChannelCount) {
        return vtxTableChannelNames[channel];
    } else {
        return "?";
    }
}

//Converts frequency (in MHz) to band and channel values.
//If frequency not found in the vtxtable then band and channel will return 0
void vtxCommonLookupBandChan(const vtxDevice_t *vtxDevice, uint16_t freq, uint8_t *pBand, uint8_t *pChannel)
{
    *pBand = 0;
    *pChannel = 0;
    if (vtxDevice) {
        // Use reverse lookup order so that 5880Mhz
        // get Raceband 7 instead of Fatshark 8.
        for (int band = vtxTableBandCount - 1 ; band >= 0 ; band--) {
            for (int channel = 0 ; channel < vtxTableChannelCount ; channel++) {
                if (vtxTableFrequency[band][channel] == freq) {
                    *pBand = band + 1;
                    *pChannel = channel + 1;
                    return;
                }
            }
        }
    }
}

//Converts band and channel values to a frequency (in MHz) value.
// band:  Band value (1 to 5).
// channel:  Channel value (1 to 8).
// Returns frequency value (in MHz), or 0 if band/channel out of range.
uint16_t vtxCommonLookupFrequency(const vtxDevice_t *vtxDevice, int band, int channel)
{
    if (vtxDevice) {
        if (band > 0 && band <= vtxTableBandCount &&
            channel > 0 && channel <= vtxTableChannelCount) {
            return vtxTableFrequency[band - 1][channel - 1];
        }
    }

    return 0;
}

const char *vtxCommonLookupPowerName(const vtxDevice_t *vtxDevice, int index)
{
    if (vtxDevice && index > 0 && index <= vtxTablePowerLevels) {
        return vtxTablePowerLabels[index];
    } else {
        return "?";
    }
}

bool vtxCommonLookupPowerValue(const vtxDevice_t *vtxDevice, int index, uint16_t *pPowerValue)
{
    if (vtxDevice && index > 0 && index <= vtxTablePowerLevels) {
        *pPowerValue = vtxTablePowerValues[index - 1];
        return true;
    } else {
        return false;
    }
}

static void vtxCommonSerializeCustomDeviceStatus(const vtxDevice_t *vtxDevice, sbuf_t *dst)
{
    const bool customDeviceStatusAvailable = vtxDevice && vtxDevice->vTable->serializeCustomDeviceStatus;

    if (customDeviceStatusAvailable) {
        vtxDevice->vTable->serializeCustomDeviceStatus(vtxDevice, dst);
    } else {
        sbufWriteU8(dst, 0);
    }
}

static void vtxCommonSerializePowerLevels(const vtxDevice_t *vtxDevice, sbuf_t *dst)
{
    uint16_t levels[VTX_TABLE_MAX_POWER_LEVELS];
    uint16_t powers[VTX_TABLE_MAX_POWER_LEVELS];

    const uint8_t powerLevelCount = vtxCommonGetVTXPowerLevels(vtxDevice, levels, powers);

    sbufWriteU8(dst, powerLevelCount);

    for (int i = 0; i < powerLevelCount; i++) {
        sbufWriteU16(dst, levels[i]);
        sbufWriteU16(dst, powers[i]);
    }
}

void vtxCommonSerializeDeviceStatus(const vtxDevice_t *vtxDevice, sbuf_t *dst)
{
    if (vtxDevice) {
        const vtxDevType_e vtxType = vtxCommonGetDeviceType(vtxDevice);
        const bool deviceReady = vtxCommonDeviceIsReady(vtxDevice);

        uint8_t band = 0;
        uint8_t channel = 0;
        const bool bandAndChannelAvailable = vtxCommonGetBandAndChannel(vtxDevice, &band, &channel);

        uint8_t powerIndex = 0;
        const bool powerIndexAvailable = vtxCommonGetPowerIndex(vtxDevice, &powerIndex);

        uint16_t frequency = 0;
        const bool frequencyAvailable = vtxCommonGetFrequency(vtxDevice, &frequency);

        unsigned vtxStatus = 0; // pitmode and/or locked
        const bool vtxStatusAvailable = vtxCommonGetStatus(vtxDevice, &vtxStatus);

        sbufWriteU8(dst, vtxType);
        sbufWriteU8(dst, deviceReady);

        sbufWriteU8(dst, bandAndChannelAvailable);
        sbufWriteU8(dst, band);
        sbufWriteU8(dst, channel);

        sbufWriteU8(dst, powerIndexAvailable);
        sbufWriteU8(dst, powerIndex);

        sbufWriteU8(dst, frequencyAvailable);
        sbufWriteU16(dst, frequency);

        sbufWriteU8(dst, vtxStatusAvailable);
        sbufWriteU32(dst, vtxStatus);

        vtxCommonSerializePowerLevels(vtxDevice, dst);
        vtxCommonSerializeCustomDeviceStatus(vtxDevice, dst);
    }
}

#endif
