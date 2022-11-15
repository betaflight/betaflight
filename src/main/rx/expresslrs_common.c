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

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 */

#include <string.h>
#include "platform.h"

#ifdef USE_RX_EXPRESSLRS

#include "build/build_config.h"
#include "common/utils.h"
#include "common/maths.h"
#include "rx/crsf.h"
#include "rx/expresslrs_common.h"
#include "drivers/rx/rx_sx127x.h"
#include "drivers/rx/rx_sx1280.h"

STATIC_UNIT_TESTED uint16_t crc14tab[ELRS_CRC_LEN] = {0};

static uint8_t volatile fhssIndex = 0;
STATIC_UNIT_TESTED uint8_t fhssSequence[ELRS_NR_SEQUENCE_ENTRIES] = {0};
static uint16_t seqCount = 0;
static uint8_t syncChannel = 0;
static uint32_t freqSpread = 0;

#define MS_TO_US(ms) (ms * 1000)

// Regarding failsafe timeout values:
// @CapnBry - Higher rates shorter timeout. Usually it runs 1-1.5 seconds with complete sync 500Hz.
//            250Hz is 2-5s. 150Hz 2.5s. 50Hz stays in sync all 5 seconds of my test.
// The failsafe timeout values come from the ELRS project's ExpressLRS_AirRateConfig definitions.
elrsModSettings_t airRateConfig[][ELRS_RATE_MAX] = {
#ifdef USE_RX_SX127X
    {
        {0, RATE_LORA_200HZ, SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_7, 5000, TLM_RATIO_1_64, 4, 8},
        {1, RATE_LORA_100HZ, SX127x_BW_500_00_KHZ, SX127x_SF_7, SX127x_CR_4_7, 10000, TLM_RATIO_1_64, 4, 8},
        {2, RATE_LORA_50HZ, SX127x_BW_500_00_KHZ, SX127x_SF_8, SX127x_CR_4_7, 20000, TLM_RATIO_1_16, 4, 10},
        {3, RATE_LORA_25HZ, SX127x_BW_500_00_KHZ, SX127x_SF_9, SX127x_CR_4_7, 40000, TLM_RATIO_1_8, 2, 10}
    },
#endif
#ifdef USE_RX_SX1280
    {
        {0, RATE_LORA_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 2000, TLM_RATIO_1_128, 4, 12},
        {1, RATE_LORA_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000, TLM_RATIO_1_64, 4, 14},
        {2, RATE_LORA_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 6666, TLM_RATIO_1_32, 4, 12},
        {3, RATE_LORA_50HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7, 20000, TLM_RATIO_1_16, 2, 12}
    },
#endif
#if !defined(USE_RX_SX127X) && !defined(USE_RX_SX1280)
    {{0}},
#endif
};

elrsRfPerfParams_t rfPerfConfig[][ELRS_RATE_MAX] = {
#ifdef USE_RX_SX127X
    {
        {0, RATE_LORA_200HZ, -112, 4380, 3000, 2500, 600, 5000},
        {1, RATE_LORA_100HZ, -117, 8770, 3500, 2500, 600, 5000},
        {2, RATE_LORA_50HZ, -120, 17540, 4000, 2500, 600, 5000},
        {3, RATE_LORA_25HZ, -123, 17540, 6000, 4000, 0, 5000}
    },
#endif
#ifdef USE_RX_SX1280
    {
        {0, RATE_LORA_500HZ, -105, 1665, 2500, 2500, 3, 5000},
        {1, RATE_LORA_250HZ, -108, 3300, 3000, 2500, 6, 5000},
        {2, RATE_LORA_150HZ, -112, 5871, 3500, 2500, 10, 5000},
        {3, RATE_LORA_50HZ, -115, 10798, 4000, 2500, 0, 5000}
    },
#endif
#if !defined(USE_RX_SX127X) && !defined(USE_RX_SX1280)
    {{0}},
#endif
};

const elrsFhssConfig_t fhssConfigs[] = {
#ifdef USE_RX_SX127X
    {AU433,  FREQ_HZ_TO_REG_VAL_900(433420000), FREQ_HZ_TO_REG_VAL_900(434420000), 3},
    {AU915,  FREQ_HZ_TO_REG_VAL_900(915500000), FREQ_HZ_TO_REG_VAL_900(926900000), 20},
    {EU433,  FREQ_HZ_TO_REG_VAL_900(433100000), FREQ_HZ_TO_REG_VAL_900(434450000), 3},
    {EU868,  FREQ_HZ_TO_REG_VAL_900(865275000), FREQ_HZ_TO_REG_VAL_900(869575000), 13},
    {IN866,  FREQ_HZ_TO_REG_VAL_900(865375000), FREQ_HZ_TO_REG_VAL_900(866950000), 4},
    {FCC915, FREQ_HZ_TO_REG_VAL_900(903500000), FREQ_HZ_TO_REG_VAL_900(926900000), 40},
#endif
#ifdef USE_RX_SX1280
    {ISM2400, FREQ_HZ_TO_REG_VAL_24(2400400000), FREQ_HZ_TO_REG_VAL_24(2479400000), 80},
    {CE2400,  FREQ_HZ_TO_REG_VAL_24(2400400000), FREQ_HZ_TO_REG_VAL_24(2479400000), 80},
#endif
#if !defined(USE_RX_SX127X) && !defined(USE_RX_SX1280)
    {0},
#endif
};

const elrsFhssConfig_t *fhssConfig;

void generateCrc14Table(void)
{
    uint16_t crc;
    for (uint16_t i = 0; i < ELRS_CRC_LEN; i++) {
        crc = i << (14 - 8);
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x2000) ? ELRS_CRC14_POLY : 0);
        }
        crc14tab[i] = crc;
    }
}

uint16_t calcCrc14(uint8_t *data, uint8_t len, uint16_t crc)
{
    while (len--) {
        crc = (crc << 8) ^ crc14tab[((crc >> 6) ^ (uint16_t) *data++) & 0x00FF];
    }
    return crc & 0x3FFF;
}

uint32_t fhssGetInitialFreq(const int32_t freqCorrection)
{
    return fhssConfig->freqStart + (syncChannel * freqSpread / ELRS_FREQ_SPREAD_SCALE) - freqCorrection;
}

uint8_t fhssGetNumEntries(void)
{
    return fhssConfig->freqCount;
}

uint8_t fhssGetCurrIndex(void)
{
    return fhssIndex;
}

void fhssSetCurrIndex(const uint8_t value)
{
    fhssIndex = value % seqCount;
}

uint32_t fhssGetNextFreq(const int32_t freqCorrection)
{
    fhssIndex = (fhssIndex + 1) % seqCount;
    uint32_t freq = fhssConfig->freqStart + (freqSpread * fhssSequence[fhssIndex] / ELRS_FREQ_SPREAD_SCALE) - freqCorrection;
    return freq;
}

static uint32_t seed = 0;

// returns 0 <= x < max where max < 256
static uint8_t rngN(const uint8_t max)
{
    const uint32_t m = 2147483648;
    const uint32_t a = 214013;
    const uint32_t c = 2531011;
    seed = (a * seed + c) % m;
    return (seed >> 16) % max;
}

/**
Requirements:
1. 0 every n hops
2. No two repeated channels
3. Equal occurance of each (or as even as possible) of each channel
4. Pseudorandom

Approach:
  Fill the sequence array with the sync channel every FHSS_FREQ_CNT
  Iterate through the array, and for each block, swap each entry in it with
  another random entry, excluding the sync channel.

*/
void fhssGenSequence(const uint8_t UID[], const elrsFreqDomain_e dom)
{
    seed = (((long)UID[2] << 24) + ((long)UID[3] << 16) + ((long)UID[4] << 8) + UID[5]) ^ ELRS_OTA_VERSION_ID;
    fhssConfig = &fhssConfigs[dom];
    seqCount = (256 / MAX(fhssConfig->freqCount, 1)) * fhssConfig->freqCount;
    syncChannel = (fhssConfig->freqCount / 2) + 1;
    freqSpread = (fhssConfig->freqStop - fhssConfig->freqStart) * ELRS_FREQ_SPREAD_SCALE / MAX((fhssConfig->freqCount - 1), 1);

    // initialize the sequence array
    for (uint16_t i = 0; i < seqCount; i++) {
        if (i % fhssConfig->freqCount == 0) {
            fhssSequence[i] = syncChannel;
        } else if (i % fhssConfig->freqCount == syncChannel) {
            fhssSequence[i] = 0;
        } else {
            fhssSequence[i] = i % fhssConfig->freqCount;
        }
    }

    for (uint16_t i = 0; i < seqCount; i++) {
        // if it's not the sync channel
        if (i % fhssConfig->freqCount != 0) {
            uint8_t offset = (i / fhssConfig->freqCount) * fhssConfig->freqCount; // offset to start of current block
            uint8_t rand = rngN(fhssConfig->freqCount - 1) + 1; // random number between 1 and numFreqs

            // switch this entry and another random entry in the same block
            uint8_t temp = fhssSequence[i];
            fhssSequence[i] = fhssSequence[offset + rand];
            fhssSequence[offset + rand] = temp;
        }
    }
}

uint8_t tlmRatioEnumToValue(const elrsTlmRatio_e enumval)
{
    // !! TLM_RATIO_STD/TLM_RATIO_DISARMED should be converted by the caller !!
    if (enumval == TLM_RATIO_NO_TLM) {
        return 1;
    }

    // 1 << (8 - (enumval - TLM_RATIO_NO_TLM))
    // 1_128 = 128, 1_64 = 64, 1_32 = 32, etc
    return 1 << (8 + TLM_RATIO_NO_TLM - enumval);
}

uint16_t rateEnumToHz(const elrsRfRate_e eRate)
{
    switch (eRate) {
    case RATE_LORA_500HZ: return 500;
    case RATE_LORA_250HZ: return 250;
    case RATE_LORA_200HZ: return 200;
    case RATE_LORA_150HZ: return 150;
    case RATE_LORA_100HZ: return 100;
    case RATE_LORA_50HZ: return 50;
    case RATE_LORA_25HZ: return 25;
    case RATE_LORA_4HZ: return 4;
    default: return 1;
    }
}

uint16_t txPowerIndexToValue(const uint8_t index)
{
    switch (index) {
    case 0: return 0;
    case 1: return 10;
    case 2: return 25;
    case 3: return 100;
    case 4: return 500;
    case 5: return 1000;
    case 6: return 2000;
    case 7: return 250;
    case 8: return 50;
    default: return 0;
    }
}

#define ELRS_LQ_DEPTH 4 //100 % 32

typedef struct linkQuality_s {
    uint32_t array[ELRS_LQ_DEPTH];
    uint8_t value;
    uint8_t index;
    uint32_t mask;
} linkQuality_t;

static linkQuality_t lq;

void lqIncrease(void)
{
    if (!lqPeriodIsSet()) {
        lq.array[lq.index] |= lq.mask;
        lq.value += 1;
    }
}

void lqNewPeriod(void)
{
    lq.mask <<= 1;
    if (lq.mask == 0) {
        lq.mask = (1 << 0);
        lq.index += 1;
    }

    // At idx N / 32 and bit N % 32, wrap back to idx=0, bit=0
    if ((lq.index == 3) && (lq.mask & (1 << ELRS_LQ_DEPTH))) {
        lq.index = 0;
        lq.mask = (1 << 0);
    }

    if ((lq.array[lq.index] & lq.mask) != 0) {
        lq.array[lq.index] &= ~lq.mask;
        lq.value -= 1;
    }
}

uint8_t lqGet(void)
{
    return lq.value;
}

bool lqPeriodIsSet(void)
{
    return lq.array[lq.index] & lq.mask;
}

void lqReset(void)
{
    memset(&lq, 0, sizeof(lq));
    lq.mask = (1 << 0);
}

uint16_t convertSwitch1b(const uint16_t val)
{
    return val ? 2000 : 1000;
}

// 3b to decode 7 pos switches
uint16_t convertSwitch3b(const uint16_t val) 
{
    switch (val) {
    case 0: return 1000;
    case 1: return 1275;
    case 2: return 1425;
    case 3: return 1575;
    case 4: return 1725;
    case 5: return 2000;
    default: return 1500;
    }
}

uint16_t convertSwitchNb(const uint16_t val, const uint16_t max)
{
    return (val > max) ? 1500 : val * 1000 / max + 1000;
}

uint8_t hybridWideNonceToSwitchIndex(const uint8_t nonce)
{
    // Returns the sequence (0 to 7, then 0 to 7 rotated left by 1):
    // 0, 1, 2, 3, 4, 5, 6, 7,
    // 1, 2, 3, 4, 5, 6, 7, 0
    // Because telemetry can occur on every 2, 4, 8, 16, 32, 64, 128th packet
    // this makes sure each of the 8 values is sent at least once every 16 packets
    // regardless of the TLM ratio
    // Index 7 also can never fall on a telemetry slot
    return ((nonce & 0x07) + ((nonce >> 3) & 0x01)) % 8;
}

uint8_t airRateIndexToIndex900(uint8_t airRate, uint8_t currentIndex)
{
    switch (airRate) {
    case 0:
        return 0;
    case 1:
        return currentIndex;
    case 2:
        return 1;
    case 3:
        return 2;
    case 4:
        return 3;
    default:
        return currentIndex;
    }
}

uint8_t airRateIndexToIndex24(uint8_t airRate, uint8_t currentIndex)
{
    switch (airRate) {
    case 0:
        return currentIndex;
    case 1:
        return currentIndex;
    case 2:
        return currentIndex;
    case 3:
        return currentIndex;
    case 4:
        return 0;
    case 5:
        return currentIndex;
    case 6:
        return 1;
    case 7:
        return 2;
    case 8:
        return currentIndex;
    case 9:
        return 3;
    default:
        return currentIndex;
    }
}

#endif /* USE_RX_EXPRESSLRS */
