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

#pragma once

#include <stdint.h>

#include "platform.h"
#include "common/time.h"
#include "common/streambuf.h"

#define VTX_SETTINGS_MAX_FREQUENCY_MHZ 5999          //max freq (in MHz) for 'vtx_freq' setting

#if defined(USE_VTX_RTC6705)

#include "drivers/vtx_rtc6705.h"

#endif

#if defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP) || defined(USE_VTX_MSP)

#define VTX_SETTINGS_FREQCMD

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
    VTXDEV_MSP         = 5,
    VTXDEV_UNKNOWN     = 0xFF,
} vtxDevType_e;

// VTX magic numbers used for spektrum vtx control
#define VTX_COMMON_BAND_USER      0
#define VTX_COMMON_BAND_A         1
#define VTX_COMMON_BAND_B         2
#define VTX_COMMON_BAND_E         3
#define VTX_COMMON_BAND_FS        4
#define VTX_COMMON_BAND_RACE      5

// RTC6705 RF Power index 25 or 200 mW
#define VTX_6705_POWER_25      1
#define VTX_6705_POWER_200     2

// SmartAudio "---", 25, 200, 500, 800 mW
#define VTX_SA_POWER_OFF          1 //1 goes to min power whereas 0 doesnt do anything (illegal index).
#define VTX_SA_POWER_25           1
#define VTX_SA_POWER_200          2
#define VTX_SA_POWER_500          3
#define VTX_SA_POWER_800          4

// Tramp "---", 25, 100, 200, 400, 600 mW
#define VTX_TRAMP_POWER_OFF       1 //same as with SmartAudio
#define VTX_TRAMP_POWER_25        1
#define VTX_TRAMP_POWER_100       2
#define VTX_TRAMP_POWER_200       3
#define VTX_TRAMP_POWER_400       4
#define VTX_TRAMP_POWER_600       5

// VTX status flags
enum {
    VTX_STATUS_PIT_MODE = 1 << 0,
    VTX_STATUS_LOCKED = 1 << 1,
};

struct vtxVTable_s;
typedef struct vtxDevice_s {
    const struct vtxVTable_s *const vTable;
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
    bool (*getFrequency)(const vtxDevice_t *vtxDevice, uint16_t *pFreq);
    bool (*getStatus)(const vtxDevice_t *vtxDevice, unsigned *status);
    uint8_t (*getPowerLevels)(const vtxDevice_t *vtxDevice, uint16_t *levels, uint16_t *powers);

    void (*serializeCustomDeviceStatus)(const vtxDevice_t *vtxDevice, sbuf_t *dst);
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
vtxDevType_e vtxCommonGetDeviceType(const vtxDevice_t *vtxDevice);
bool vtxCommonDeviceIsReady(const vtxDevice_t *vtxDevice);

void vtxCommonSetBandAndChannel(vtxDevice_t *vtxDevice, uint8_t band, uint8_t channel);
void vtxCommonSetPowerByIndex(vtxDevice_t *vtxDevice, uint8_t level);
void vtxCommonSetPitMode(vtxDevice_t *vtxDevice, uint8_t onoff);
void vtxCommonSetFrequency(vtxDevice_t *vtxDevice, uint16_t freq);

bool vtxCommonGetBandAndChannel(const vtxDevice_t *vtxDevice, uint8_t *pBand, uint8_t *pChannel);
bool vtxCommonGetPowerIndex(const vtxDevice_t *vtxDevice, uint8_t *pIndex);
bool vtxCommonGetFrequency(const vtxDevice_t *vtxDevice, uint16_t *pFreq);
bool vtxCommonGetStatus(const vtxDevice_t *vtxDevice, unsigned *status);
uint8_t vtxCommonGetVTXPowerLevels(const vtxDevice_t *vtxDevice, uint16_t *levels, uint16_t *powers);
// end of VTable functions

const char *vtxCommonLookupBandName(const vtxDevice_t *vtxDevice, int band);
char vtxCommonLookupBandLetter(const vtxDevice_t *vtxDevice, int band);
char vtxCommonGetBandLetter(const vtxDevice_t *vtxDevice, int band);
const char *vtxCommonLookupChannelName(const vtxDevice_t *vtxDevice, int channel);
uint16_t vtxCommonLookupFrequency(const vtxDevice_t *vtxDevice, int band, int channel);
void vtxCommonLookupBandChan(const vtxDevice_t *vtxDevice, uint16_t freq, uint8_t *pBand, uint8_t *pChannel);
const char *vtxCommonLookupPowerName(const vtxDevice_t *vtxDevice, int index);
bool vtxCommonLookupPowerValue(const vtxDevice_t *vtxDevice, int index, uint16_t *pPowerValue);
void vtxCommonSerializeDeviceStatus(const vtxDevice_t *vtxDevice, sbuf_t *dst);
