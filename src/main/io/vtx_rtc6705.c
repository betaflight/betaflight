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

#include "platform.h"

#if defined(USE_VTX_RTC6705) && defined(USE_VTX_CONTROL)

#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/max7456.h"
#include "drivers/time.h"
#include "drivers/vtx_rtc6705.h"

#include "io/vtx.h"
#include "io/vtx_rtc6705.h"
#include "io/vtx_string.h"


#if defined(USE_CMS) || defined(USE_VTX_COMMON)
const char * rtc6705PowerNames[] = {
    "OFF", "MIN", "MAX"
};
#endif

#ifdef USE_VTX_COMMON
static vtxVTable_t rtc6705VTable;    // Forward
static vtxDevice_t vtxRTC6705 = {
    .vTable = &rtc6705VTable,
};
#endif

static void vtxRTC6705SetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel);
static void vtxRTC6705SetFrequency(vtxDevice_t *vtxDevice, uint16_t frequency);

bool vtxRTC6705Init(void)
{
    vtxRTC6705.capability.bandCount = VTX_SETTINGS_BAND_COUNT,
    vtxRTC6705.capability.channelCount = VTX_SETTINGS_CHANNEL_COUNT,
    vtxRTC6705.capability.powerCount = VTX_RTC6705_POWER_COUNT,
    vtxRTC6705.frequencyTable = vtxStringFrequencyTable();
    vtxRTC6705.bandNames = vtxStringBandNames();
    vtxRTC6705.bandLetters = vtxStringBandLetters();
    vtxRTC6705.channelNames = vtxStringChannelNames();
    vtxRTC6705.powerNames = rtc6705PowerNames,

    vtxCommonSetDevice(&vtxRTC6705);

    vtxInit();

    return true;
}

bool vtxRTC6705CanUpdate(void)
{
#if defined(MAX7456_SPI_INSTANCE) && defined(RTC6705_SPI_INSTANCE) && defined(SPI_SHARED_MAX7456_AND_RTC6705)
    if (featureIsEnabled(FEATURE_OSD)) {
        return !max7456DmaInProgress();
    }
#endif
    return true;
}

#ifdef RTC6705_POWER_PIN
static void vtxRTC6705Configure(vtxDevice_t *vtxDevice)
{
    rtc6705SetRFPower(vtxDevice->powerIndex);
    vtxRTC6705SetBandAndChannel(vtxDevice, vtxDevice->band, vtxDevice->channel);
}

static void vtxRTC6705EnableAndConfigure(vtxDevice_t *vtxDevice)
{
    while (!vtxRTC6705CanUpdate());

    rtc6705Enable();

    delay(VTX_RTC6705_BOOT_DELAY);

    vtxRTC6705Configure(vtxDevice);
}
#endif

static void vtxRTC6705Process(vtxDevice_t *vtxDevice, timeUs_t now)
{
    UNUSED(vtxDevice);
    UNUSED(now);
}

#ifdef USE_VTX_COMMON
// Interface to common VTX API

static vtxDevType_e vtxRTC6705GetDeviceType(const vtxDevice_t *vtxDevice)
{
    UNUSED(vtxDevice);
    return VTXDEV_RTC6705;
}

static bool vtxRTC6705IsReady(const vtxDevice_t *vtxDevice)
{
    return vtxDevice != NULL;
}

static void vtxRTC6705SetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel)
{
    while (!vtxRTC6705CanUpdate());

    if (band >= 1 && band <= VTX_SETTINGS_BAND_COUNT && channel >= 1 && channel <= VTX_SETTINGS_CHANNEL_COUNT) {
        if (vtxDevice->powerIndex > 0) {
            vtxDevice->band = band;
            vtxDevice->channel = channel;
            vtxRTC6705SetFrequency(vtxDevice, vtxCommonLookupFrequency(vtxDevice, band, channel));
        }
    }
}

static void vtxRTC6705SetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    while (!vtxRTC6705CanUpdate());

#ifdef RTC6705_POWER_PIN
    if (index == 0) {
        // power device off
        if (vtxDevice->powerIndex > 0) {
            // on, power it off
            vtxDevice->powerIndex = index;
            rtc6705Disable();
            return;
        } else {
            // already off
        }
    } else {
        // change rf power and maybe turn the device on first
        if (vtxDevice->powerIndex == 0) {
            // if it's powered down, power it up, wait and configure channel, band and power.
            vtxDevice->powerIndex = index;
            vtxRTC6705EnableAndConfigure(vtxDevice);
            return;
        } else {
            // if it's powered up, just set the rf power
            vtxDevice->powerIndex = index;
            rtc6705SetRFPower(index);
        }
    }
#else
    vtxDevice->powerIndex = MAX(index, VTX_RTC6705_MIN_POWER);
    rtc6705SetRFPower(index);
#endif
}

static void vtxRTC6705SetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff)
{
    UNUSED(vtxDevice);
    UNUSED(onoff);
}

static void vtxRTC6705SetFrequency(vtxDevice_t *vtxDevice, uint16_t frequency)
{
    if (frequency >= VTX_RTC6705_FREQ_MIN &&  frequency <= VTX_RTC6705_FREQ_MAX) {
        frequency = constrain(frequency, VTX_RTC6705_FREQ_MIN, VTX_RTC6705_FREQ_MAX);
        vtxDevice->frequency = frequency;
        rtc6705SetFrequency(frequency);
    }
}

static bool vtxRTC6705GetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    *pBand = vtxDevice->band;
    *pChannel = vtxDevice->channel;
    return true;
}

static bool vtxRTC6705GetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    *pIndex = vtxDevice->powerIndex;
    return true;
}

static bool vtxRTC6705GetPitMode(const vtxDevice_t *vtxDevice, uint8_t *pOnOff)
{
    UNUSED(vtxDevice);
    UNUSED(pOnOff);
    return false;
}

static bool vtxRTC6705GetFreq(const vtxDevice_t *vtxDevice, uint16_t *pFrequency)
{
    *pFrequency = vtxDevice->frequency;
    return true;
}

static vtxVTable_t rtc6705VTable = {
    .process = vtxRTC6705Process,
    .getDeviceType = vtxRTC6705GetDeviceType,
    .isReady = vtxRTC6705IsReady,
    .setBandAndChannel = vtxRTC6705SetBandAndChannel,
    .setPowerByIndex = vtxRTC6705SetPowerByIndex,
    .setPitMode = vtxRTC6705SetPitMode,
    .setFrequency = vtxRTC6705SetFrequency,
    .getBandAndChannel = vtxRTC6705GetBandAndChannel,
    .getPowerIndex = vtxRTC6705GetPowerIndex,
    .getPitMode = vtxRTC6705GetPitMode,
    .getFrequency = vtxRTC6705GetFreq,
};
#endif // VTX_COMMON

#endif // VTX_RTC6705
