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
#include "drivers/vtx_table.h"
#include "drivers/vtx_rtc6705.h"

#include "io/vtx.h"
#include "io/vtx_rtc6705.h"

#if (defined(USE_CMS) || defined(USE_VTX_COMMON)) && !defined(USE_VTX_TABLE)
const char *rtc6705PowerNames[VTX_RTC6705_POWER_COUNT + 1] = {
    "---", "MIN", "MAX"
};
#endif

#ifdef USE_VTX_COMMON
static vtxVTable_t rtc6705VTable;    // Forward
static vtxDevice_t vtxRTC6705 = {
    .vTable = &rtc6705VTable,
};
#endif

static uint16_t rtc6705Frequency;
static int8_t rtc6705PowerIndex;
static bool rtc6705PitModeActive;

static void vtxRTC6705SetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel);
static void vtxRTC6705SetFrequency(vtxDevice_t *vtxDevice, uint16_t frequency);

bool vtxRTC6705Init(void)
{
    vtxCommonSetDevice(&vtxRTC6705);
#ifndef USE_VTX_TABLE
    //without USE_VTX_TABLE, fill vtxTable variables with default settings (instead of loading them from PG)
    vtxTablePowerLevels = VTX_RTC6705_POWER_COUNT;
    for (int i = 0; i < VTX_RTC6705_POWER_COUNT + 1; i++) {
        vtxTablePowerLabels[i] = rtc6705PowerNames[i];
    }
    for (int i = 0; i < VTX_RTC6705_POWER_COUNT; i++) {
        vtxTablePowerValues[i] = i;
    }

#endif

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
    uint16_t newPowerValue = 0;
    vtxCommonLookupPowerValue(vtxDevice, rtc6705PowerIndex, &newPowerValue);
    rtc6705SetRFPower(newPowerValue);
    vtxRTC6705SetFrequency(vtxDevice, rtc6705Frequency);
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
    UNUSED(vtxDevice);
    UNUSED(band);
    UNUSED(channel);
    //rtc6705 does not support bands and channels, only frequencies
}

static void vtxRTC6705SetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t index)
{
    while (!vtxRTC6705CanUpdate());

    uint16_t newPowerValue = 0;
    if (!vtxCommonLookupPowerValue(vtxDevice, index, &newPowerValue)) {
        return;
    }
    rtc6705PowerIndex = index;
    rtc6705SetRFPower(newPowerValue);
}

static void vtxRTC6705SetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff)
{
    UNUSED(vtxDevice);
#ifdef RTC6705_POWER_PIN
    if (onoff) {
        // power device off
        if (!rtc6705PitModeActive) {
            // on, power it off
            rtc6705PitModeActive = onoff;
            rtc6705Disable();
            return;
        } else {
            // already off
        }
    } else {
        // change rf power and maybe turn the device on first
        if (rtc6705PitModeActive) {
            // if it's powered down, power it up, wait and configure channel, band and power.
            rtc6705PitModeActive = onoff;
            vtxRTC6705EnableAndConfigure(vtxDevice);
            return;
        } else {
            //already on
        }
    }
#else
    UNUSED(onoff);
#endif
}

static void vtxRTC6705SetFrequency(vtxDevice_t *vtxDevice, uint16_t frequency)
{
    UNUSED(vtxDevice);
    if (frequency >= VTX_RTC6705_FREQ_MIN &&  frequency <= VTX_RTC6705_FREQ_MAX) {
        frequency = constrain(frequency, VTX_RTC6705_FREQ_MIN, VTX_RTC6705_FREQ_MAX);
        rtc6705Frequency = frequency;
        rtc6705SetFrequency(frequency);
    }
}

static bool vtxRTC6705GetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel)
{
    UNUSED(vtxDevice);
    *pBand = 0;
    *pChannel = 0;
    //rtc6705 does not support bands and channels, only frequencies.
    //therefore always return 0
    return true;
}

static bool vtxRTC6705GetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex)
{
    UNUSED(vtxDevice);
    *pIndex = rtc6705PowerIndex;
    return true;
}

static bool vtxRTC6705GetFreq(const vtxDevice_t *vtxDevice, uint16_t *pFrequency)
{
    UNUSED(vtxDevice);
    *pFrequency = rtc6705Frequency;
    return true;
}

static bool vtxRTC6705GetStatus(const vtxDevice_t *vtxDevice, unsigned *status)
{
    UNUSED(vtxDevice);
    *status = rtc6705PitModeActive ? VTX_STATUS_PIT_MODE : 0;
    return true;
}

static uint8_t vtxRTC6705GetPowerLevels(const vtxDevice_t *vtxDevice, uint16_t *levels, uint16_t *powers)
{
    UNUSED(vtxDevice);
    UNUSED(levels);
    UNUSED(powers);

    return 0;
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
    .getFrequency = vtxRTC6705GetFreq,
    .getStatus = vtxRTC6705GetStatus,
    .getPowerLevels = vtxRTC6705GetPowerLevels,
    .serializeCustomDeviceStatus = NULL,
};

#endif // VTX_COMMON

#endif // VTX_RTC6705
