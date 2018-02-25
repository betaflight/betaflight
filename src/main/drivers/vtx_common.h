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

#pragma once

#include <stdint.h>

#include "platform.h"
#include "common/time.h"


#define VTX_SETTINGS_MIN_BAND       1
#define VTX_SETTINGS_MAX_BAND       5
#define VTX_SETTINGS_MIN_CHANNEL    1
#define VTX_SETTINGS_MAX_CHANNEL    8

#define VTX_SETTINGS_BAND_COUNT     (VTX_SETTINGS_MAX_BAND - VTX_SETTINGS_MIN_BAND + 1)
#define VTX_SETTINGS_CHANNEL_COUNT  (VTX_SETTINGS_MAX_CHANNEL - VTX_SETTINGS_MIN_CHANNEL + 1)

#define VTX_SETTINGS_DEFAULT_BAND               4
#define VTX_SETTINGS_DEFAULT_CHANNEL            1
#define VTX_SETTINGS_DEFAULT_FREQ               5740
#define VTX_SETTINGS_DEFAULT_PITMODE_FREQ       0
#define VTX_SETTINGS_DEFAULT_LOW_POWER_DISARM   0
#define VTX_SETTINGS_DEFAULT_AKK_HACK           0

#define VTX_SETTINGS_MAX_FREQUENCY_MHZ 5999          //max freq (in MHz) for 'vtx_freq' setting

#if defined(USE_VTX_RTC6705)

#include "drivers/vtx_rtc6705.h"

#endif

#if defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)

#define VTX_SETTINGS_POWER_COUNT        5
#define VTX_SETTINGS_DEFAULT_POWER      1
#define VTX_SETTINGS_MIN_POWER          0
#define VTX_SETTINGS_MIN_USER_FREQ      5000
#define VTX_SETTINGS_MAX_USER_FREQ      5999
#define VTX_SETTINGS_FREQCMD

#elif defined(USE_VTX_RTC6705)

#define VTX_SETTINGS_POWER_COUNT    VTX_RTC6705_POWER_COUNT
#define VTX_SETTINGS_DEFAULT_POWER  VTX_RTC6705_DEFAULT_POWER
#define VTX_SETTINGS_MIN_POWER      VTX_RTC6705_MIN_POWER

#endif

// check value for MSP_SET_VTX_CONFIG to determine if value is encoded
// band/channel or frequency in MHz (3 bits for band and 3 bits for channel)
#define VTXCOMMON_MSP_BANDCHAN_CHKVAL ((uint16_t)((7 << 3) + 7))

typedef enum {
    VTXDEV_UNSUPPORTED = 0, // reserved for MSP
    VTXDEV_RTC6705     = 1,
    // 2 reserved
    VTXDEV_SMARTAUDIO  = 3,
    VTXDEV_TRAMP       = 4,
    VTXDEV_UNKNOWN     = 0xFF,
} vtxDevType_e;

// VTX magic numbers
#define VTX_COMMON_BAND_USER      0
#define VTX_COMMON_BAND_A         1
#define VTX_COMMON_BAND_B         2
#define VTX_COMMON_BAND_E         3
#define VTX_COMMON_BAND_FS        4
#define VTX_COMMON_BAND_RACE      5

// RTC6705 RF Power index "---", 25 or 200 mW
#define VTX_6705_POWER_OFF     0
#define VTX_6705_POWER_25      1
#define VTX_6705_POWER_200     2

// SmartAudio "---", 25, 200, 500, 800 mW
#define VTX_SA_POWER_OFF          0
#define VTX_SA_POWER_25           1
#define VTX_SA_POWER_200          2
#define VTX_SA_POWER_500          3
#define VTX_SA_POWER_800          4

// Tramp "---", 25, 100, 200, 400, 600 mW
#define VTX_TRAMP_POWER_OFF       0
#define VTX_TRAMP_POWER_25        1
#define VTX_TRAMP_POWER_100       2
#define VTX_TRAMP_POWER_200       3
#define VTX_TRAMP_POWER_400       4
#define VTX_TRAMP_POWER_600       5


typedef struct vtxDeviceCapability_s {
    uint8_t bandCount;
    uint8_t channelCount;
    uint8_t powerCount;
    uint8_t filler;
} vtxDeviceCapability_t;

struct vtxVTable_s;
typedef struct vtxDevice_s {
    const struct vtxVTable_s * const vTable;

    vtxDeviceCapability_t capability;

    uint16_t *frequencyTable;  // Array of [bandCount][channelCount]
    char **bandNames;    // char *bandNames[bandCount]
    char **channelNames;    // char *channelNames[channelCount]
    char **powerNames;   // char *powerNames[powerCount]

    uint16_t frequency;
    uint8_t band; // Band = 1, 1-based
    uint8_t channel; // CH1 = 1, 1-based
    uint8_t powerIndex; // Lowest/Off = 0
    uint8_t pitMode; // 0 = non-PIT, 1 = PIT
} vtxDevice_t;


// {set,get}BandAndChannel: band and channel are 1 origin
// {set,get}PowerByIndex: 0 = Power OFF, 1 = device dependent
// {set,get}PitMode: 0 = OFF, 1 = ON

typedef struct vtxVTable_s {
    void (*process)(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs);
    vtxDevType_e (*getDeviceType)(const vtxDevice_t *vtxDevice);
    bool (*isReady)(const vtxDevice_t *vtxDevice);

    void (*setBandAndChannel)(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel);
    void (*setPowerByIndex)(vtxDevice_t *vtxDevice, uint8_t level);
    void (*setPitMode)(vtxDevice_t *vtxDevice, uint8_t onoff);
    void (*setFrequency)(vtxDevice_t *vtxDevice, uint16_t freq);

    bool (*getBandAndChannel)(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel);
    bool (*getPowerIndex)(const vtxDevice_t *vtxDevice, uint8_t *pIndex);
    bool (*getPitMode)(const vtxDevice_t *vtxDevice, uint8_t *pOnOff);
    bool (*getFrequency)(const vtxDevice_t *vtxDevice, uint16_t *pFreq);
} vtxVTable_t;

// 3.1.0
// PIT mode is defined as LOWEST POSSIBLE RF POWER.
// - It can be a dedicated mode, or lowest RF power possible.
// - It is *NOT* RF on/off control ?

void vtxCommonInit(void);
void vtxCommonSetDevice(vtxDevice_t *vtxDevice);
vtxDevice_t *vtxCommonDevice(void);

// VTable functions
void vtxCommonProcess(vtxDevice_t *vtxDevice, timeUs_t currentTimeUs);
uint8_t vtxCommonGetDeviceType(const vtxDevice_t *vtxDevice);
void vtxCommonSetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel);
void vtxCommonSetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t level);
void vtxCommonSetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff);
void vtxCommonSetFrequency(vtxDevice_t *vtxDevice, uint16_t freq);
bool vtxCommonGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel);
bool vtxCommonGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex);
bool vtxCommonGetPitMode(const vtxDevice_t *vtxDevice, uint8_t *pOnOff);
bool vtxCommonGetFrequency(const vtxDevice_t *vtxDevice, uint16_t *pFreq);
bool vtxCommonGetDeviceCapability(const vtxDevice_t *vtxDevice, vtxDeviceCapability_t *pDeviceCapability);
