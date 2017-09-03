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
#pragma once
#include "cms/cms_types.h"

/* Created by jflyper */
/*
typedef enum {
    VTXDEV_UNSUPPORTED = 0, // reserved for MSP
    VTXDEV_RTC6705    = 1,
    // 2 reserved
    VTXDEV_SMARTAUDIO = 3,
    VTXDEV_TRAMP      = 4,
    VTXDEV_OPENTCO    = 5,
    VTXDEV_UNKNOWN    = 0xFF,
} vtxDevType_e;
*/
struct vtxVTable_s;

typedef struct vtxDeviceCapability_s {
    uint8_t bandCount;
    uint8_t channelCount;
    uint8_t powerCount;
} vtxDeviceCapability_t;

#define VTX_COMMON_MAX_CHANNEL 8
#define VTX_COMMON_MAX_BAND 5
#define VTX_COMMON_MAX_POWER_COUNT 8

typedef enum {
    VTX_DEVICE_NONE    = 0,
    VTX_DEVICE_SMARTAUDIO = 1,
    VTX_DEVICE_TRAMP = 2,
    VTX_DEVICE_RTC6705 = 3,
    VTX_DEVICE_OPENTCO = 4,
    VTX_DEVICE_UNKNOWN = 0xFF
} vtxDevice_e;

typedef struct vtxDeviceConfig_s {
    vtxDevice_e device;
    uint8_t band; // Band = 1, 1-based
    uint8_t channel; // CH1 = 1, 1-based
    uint8_t powerIndex; // Lowest/Off = 0
    uint8_t pitMode; // 0 = non-PIT, 1 = PIT
} vtxDeviceConfig_t;

typedef struct vtxDevice_s {
    const struct vtxVTable_s * const vTable;

    vtxDeviceCapability_t capability;

    uint16_t *frequencyTable;  // Array of [bandCount][channelCount]
    char **bandNames;    // char *bandNames[bandCount]
    char **channelNames;    // char *channelNames[channelCount]
    char **powerNames;   // char *powerNames[powerCount]

    CMS_Menu *cmsMenu;
} vtxDevice_t;

PG_DECLARE(vtxDeviceConfig_t, vtxDeviceConfig);

// {set,get}BandAndChannel: band and channel are 1 origin
// {set,get}PowerByIndex: 0 = Power OFF, 1 = device dependent
// {set,get}PitMode: 0 = OFF, 1 = ON

typedef struct vtxVTable_s {
    void (*process)(uint32_t currentTimeUs);
    //vtxDevType_e (*getDeviceType)(void);
    bool (*isReady)(void);

    bool (*setBandAndChannel)(uint8_t band, uint8_t channel);
    bool (*setPowerByIndex)(uint8_t level);
    bool (*setPitMode)(uint8_t onoff);

    bool (*getBandAndChannel)(uint8_t *pBand, uint8_t *pChannel);
    bool (*getPowerIndex)(uint8_t *pIndex);
    bool (*getPitMode)(uint8_t *pOnOff);
} vtxVTable_t;

// 3.1.0
// PIT mode is defined as LOWEST POSSIBLE RF POWER.
// - It can be a dedicated mode, or lowest RF power possible.
// - It is *NOT* RF on/off control ?

void vtxCommonInit(void);
void vtxCommonRegisterDevice(vtxDevice_t *pDevice);

// VTable functions
void vtxCommonProcess(uint32_t currentTimeUs);
uint8_t vtxCommonGetDeviceType(void);
bool vtxCommonSetBandAndChannel(uint8_t band, uint8_t channel);
bool vtxCommonSetPowerByIndex(uint8_t level);
bool vtxCommonSetPitMode(uint8_t onoff);
bool vtxCommonGetBandAndChannel(uint8_t *pBand, uint8_t *pChannel);
bool vtxCommonGetPowerIndex(uint8_t *pIndex);
bool vtxCommonGetPitMode(uint8_t *pOnOff);
bool vtxCommonGetDeviceCapability(vtxDeviceCapability_t *pDeviceCapability);

bool vtxCommonGetBandName(uint8_t band, char **name);
bool vtxCommonGetPowerName(uint8_t index, char **name);
bool vtxCommonGetChannelName(uint8_t ch, char **name);

CMS_Menu *vtxCommonGetCmsMenu();
