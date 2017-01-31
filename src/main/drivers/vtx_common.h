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

typedef enum {
    VTXDEV_UNSUPPORTED = 0, // reserved for MSP
    // 1 reserved
    // 2 reserved
    VTXDEV_SMARTAUDIO = 3,
    VTXDEV_TRAMP      = 4,
    VTXDEV_UNKNOWN    = 0xFF,
} vtxDevType_e;

typedef struct vtxDeviceParameter_s {
    uint8_t deviceType;

    uint8_t numBand;
    uint8_t numChan;
    uint8_t numPower;

    uint16_t *freqTable;  // Array of [numBand][numChan]
    char **bandNames;    // char *bandNames[numBand + 1]
    char *bandLetters;   // char bandLetters[numBand + 1]
    char **chanNames;    // char *chanNames[numChan + 1]
    char **powerNames;   // char *powerNames[numPower + 1]
} vtxDeviceParameter_t;

struct vtxVTable_s;

typedef struct vtxDevice_s {
    const struct vtxVTable_s *vTable;
    const vtxDeviceParameter_t devParam;
    uint8_t curBand;
    uint8_t curChan;
    uint8_t curPowerIndex;
    uint8_t curPitState; // 0 = non-PIT, 1 = PIT
} vtxDevice_t;

// {set,get}BandChan: band and chan are 1 origin
// {set,get}PowerByIndex: 0 = Power OFF, 1 = device dependent
// {set,get}Pitmode: 0 = OFF, 1 = ON

typedef struct vtxVTable_s {
    void (*process)(uint32_t currentTimeUs);
    vtxDevType_e (*getDeviceType)(void);
    bool (*isReady)(void);

    void (*setBandChan)(uint8_t band, uint8_t chan);
    void (*setPowerByIndex)(uint8_t level);
    void (*setPitmode)(uint8_t onoff);

    bool (*getBandChan)(uint8_t *pBand, uint8_t *pChan);
    bool (*getPowerIndex)(uint8_t *pIndex);
    bool (*getPitmode)(uint8_t *pOnoff);

    vtxDeviceParameter_t (*getDeviceParameter)(void);
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
void vtxCommonSetBandChan(uint8_t band, uint8_t chan);
void vtxCommonSetPowerByIndex(uint8_t level);
void vtxCommonSetPitmode(uint8_t onoff);
bool vtxCommonGetBandChan(uint8_t *pBand, uint8_t *pChan);
bool vtxCommonGetPowerIndex(uint8_t *pIndex);
bool vtxCommonGetPitmode(uint8_t *pOnoff);

// API 1.1
const vtxDeviceParameter_t *vtxCommonGetDeviceParameter(void);
char *vtxCommonGetBandChanString(void);
char *vtxCommonGetFreqString(void);
