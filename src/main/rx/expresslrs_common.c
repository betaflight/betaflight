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
static const uint32_t *fhssFreqs;
static uint8_t numFreqs = 0; // The number of FHSS frequencies in the table
static uint8_t seqCount = 0;
static uint8_t syncChannel = 0;

#define MS_TO_US(ms) (ms * 1000)

// Regarding failsafe timeout values:
// @CapnBry - Higher rates shorter timeout. Usually it runs 1-1.5 seconds with complete sync 500Hz.
//            250Hz is 2-5s. 150Hz 2.5s. 50Hz stays in sync all 5 seconds of my test.
// The failsafe timeout values come from the ELRS project's ExpressLRS_AirRateConfig definitions.
elrsModSettings_t airRateConfig[][ELRS_RATE_MAX] = {
#ifdef USE_RX_SX127X
    {
        {0, RATE_200HZ, SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_7, 5000, TLM_RATIO_1_64, 4, 8},
        {1, RATE_100HZ, SX127x_BW_500_00_KHZ, SX127x_SF_7, SX127x_CR_4_7, 10000, TLM_RATIO_1_64, 4, 8},
        {2, RATE_50HZ, SX127x_BW_500_00_KHZ, SX127x_SF_8, SX127x_CR_4_7, 20000, TLM_RATIO_NO_TLM, 4, 10},
        {3, RATE_25HZ, SX127x_BW_500_00_KHZ, SX127x_SF_9, SX127x_CR_4_7, 40000, TLM_RATIO_NO_TLM, 2, 10}
    },
#endif
#ifdef USE_RX_SX1280
    {
        {0, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 2000, TLM_RATIO_1_128, 4, 12},
        {1, RATE_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000, TLM_RATIO_1_64, 4, 14},
        {2, RATE_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 6666, TLM_RATIO_1_32, 4, 12},
        {3, RATE_50HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_6, 20000, TLM_RATIO_NO_TLM, 2, 12}
    },
#endif
#if !defined(USE_RX_SX127X) && !defined(USE_RX_SX1280)
    {{0}},
#endif
};

elrsRfPerfParams_t rfPerfConfig[][ELRS_RATE_MAX] = {
#ifdef USE_RX_SX127X
    {
        {0, RATE_200HZ, -112, 4380, 3000, 2500, 600, 5000},
        {1, RATE_100HZ, -117, 8770, 3500, 2500, 600, 5000},
        {2, RATE_50HZ, -120, 17540, 4000, 2500, 600, 5000},
        {3, RATE_25HZ, -123, 17540, 6000, 4000, 0, 5000}
    },
#endif
#ifdef USE_RX_SX1280
    {
        {0, RATE_500HZ, -105, 1665, 2500, 2500, 3, 5000},
        {1, RATE_250HZ, -108, 3300, 3000, 2500, 6, 5000},
        {2, RATE_150HZ, -112, 5871, 3500, 2500, 10, 5000},
        {3, RATE_50HZ, -117, 18443, 4000, 2500, 0, 5000}
    },
#endif
#if !defined(USE_RX_SX127X) && !defined(USE_RX_SX1280)
    {{0}},
#endif
};

#ifdef USE_RX_SX127X
const uint32_t fhssFreqsAU433[] = {
    FREQ_HZ_TO_REG_VAL_900(433420000),
    FREQ_HZ_TO_REG_VAL_900(433920000),
    FREQ_HZ_TO_REG_VAL_900(434420000)};

const uint32_t fhssFreqsAU915[] = {
    FREQ_HZ_TO_REG_VAL_900(915500000),
    FREQ_HZ_TO_REG_VAL_900(916100000),
    FREQ_HZ_TO_REG_VAL_900(916700000),
    FREQ_HZ_TO_REG_VAL_900(917300000),

    FREQ_HZ_TO_REG_VAL_900(917900000),
    FREQ_HZ_TO_REG_VAL_900(918500000),
    FREQ_HZ_TO_REG_VAL_900(919100000),
    FREQ_HZ_TO_REG_VAL_900(919700000),

    FREQ_HZ_TO_REG_VAL_900(920300000),
    FREQ_HZ_TO_REG_VAL_900(920900000),
    FREQ_HZ_TO_REG_VAL_900(921500000),
    FREQ_HZ_TO_REG_VAL_900(922100000),

    FREQ_HZ_TO_REG_VAL_900(922700000),
    FREQ_HZ_TO_REG_VAL_900(923300000),
    FREQ_HZ_TO_REG_VAL_900(923900000),
    FREQ_HZ_TO_REG_VAL_900(924500000),

    FREQ_HZ_TO_REG_VAL_900(925100000),
    FREQ_HZ_TO_REG_VAL_900(925700000),
    FREQ_HZ_TO_REG_VAL_900(926300000),
    FREQ_HZ_TO_REG_VAL_900(926900000)};

/* Frequency bands taken from https://wetten.overheid.nl/BWBR0036378/2016-12-28#Bijlagen
 * Note: these frequencies fall in the license free H-band, but in combination with 500kHz 
 * LoRa modem bandwidth used by ExpressLRS (EU allows up to 125kHz modulation BW only) they
 * will never pass RED certification and they are ILLEGAL to use. 
 * 
 * Therefore we simply maximize the usage of available spectrum so laboratory testing of the software won't disturb existing
 * 868MHz ISM band traffic too much.
 */
const uint32_t fhssFreqsEU868[] = {
    FREQ_HZ_TO_REG_VAL_900(863275000), // band H1, 863 - 865MHz, 0.1% duty cycle or CSMA techniques, 25mW EIRP
    FREQ_HZ_TO_REG_VAL_900(863800000),
    FREQ_HZ_TO_REG_VAL_900(864325000),
    FREQ_HZ_TO_REG_VAL_900(864850000),
    FREQ_HZ_TO_REG_VAL_900(865375000), // Band H2, 865 - 868.6MHz, 1.0% dutycycle or CSMA, 25mW EIRP
    FREQ_HZ_TO_REG_VAL_900(865900000),
    FREQ_HZ_TO_REG_VAL_900(866425000),
    FREQ_HZ_TO_REG_VAL_900(866950000),
    FREQ_HZ_TO_REG_VAL_900(867475000),
    FREQ_HZ_TO_REG_VAL_900(868000000),
    FREQ_HZ_TO_REG_VAL_900(868525000), // Band H3, 868.7-869.2MHz, 0.1% dutycycle or CSMA, 25mW EIRP
    FREQ_HZ_TO_REG_VAL_900(869050000),
    FREQ_HZ_TO_REG_VAL_900(869575000)};

/**
 * India currently delicensed the 865-867 MHz band with a maximum of 1W Transmitter power,
 * 4Watts Effective Radiated Power and 200Khz carrier bandwidth as per
 * https://dot.gov.in/sites/default/files/Delicensing%20in%20865-867%20MHz%20band%20%5BGSR%20564%20%28E%29%5D_0.pdf .
 * There is currently no mention of Direct-sequence spread spectrum,
 * So these frequencies are a subset of Regulatory_Domain_EU_868 frequencies.
 */
const uint32_t fhssFreqsIN866[] = {
    FREQ_HZ_TO_REG_VAL_900(865375000),
    FREQ_HZ_TO_REG_VAL_900(865900000),
    FREQ_HZ_TO_REG_VAL_900(866425000),
    FREQ_HZ_TO_REG_VAL_900(866950000)};

/* Frequency band G, taken from https://wetten.overheid.nl/BWBR0036378/2016-12-28#Bijlagen
 * Note: As is the case with the 868Mhz band, these frequencies only comply to the license free portion
 * of the spectrum, nothing else. As such, these are likely illegal to use. 
 */
const uint32_t fhssFreqsEU433[] = {
    FREQ_HZ_TO_REG_VAL_900(433100000),
    FREQ_HZ_TO_REG_VAL_900(433925000),
    FREQ_HZ_TO_REG_VAL_900(434450000)};

/* Very definitely not fully checked. An initial pass at increasing the hops
*/
const uint32_t fhssFreqsFCC915[] = {
    FREQ_HZ_TO_REG_VAL_900(903500000),
    FREQ_HZ_TO_REG_VAL_900(904100000),
    FREQ_HZ_TO_REG_VAL_900(904700000),
    FREQ_HZ_TO_REG_VAL_900(905300000),

    FREQ_HZ_TO_REG_VAL_900(905900000),
    FREQ_HZ_TO_REG_VAL_900(906500000),
    FREQ_HZ_TO_REG_VAL_900(907100000),
    FREQ_HZ_TO_REG_VAL_900(907700000),

    FREQ_HZ_TO_REG_VAL_900(908300000),
    FREQ_HZ_TO_REG_VAL_900(908900000),
    FREQ_HZ_TO_REG_VAL_900(909500000),
    FREQ_HZ_TO_REG_VAL_900(910100000),

    FREQ_HZ_TO_REG_VAL_900(910700000),
    FREQ_HZ_TO_REG_VAL_900(911300000),
    FREQ_HZ_TO_REG_VAL_900(911900000),
    FREQ_HZ_TO_REG_VAL_900(912500000),

    FREQ_HZ_TO_REG_VAL_900(913100000),
    FREQ_HZ_TO_REG_VAL_900(913700000),
    FREQ_HZ_TO_REG_VAL_900(914300000),
    FREQ_HZ_TO_REG_VAL_900(914900000),

    FREQ_HZ_TO_REG_VAL_900(915500000), // as per AU..
    FREQ_HZ_TO_REG_VAL_900(916100000),
    FREQ_HZ_TO_REG_VAL_900(916700000),
    FREQ_HZ_TO_REG_VAL_900(917300000),

    FREQ_HZ_TO_REG_VAL_900(917900000),
    FREQ_HZ_TO_REG_VAL_900(918500000),
    FREQ_HZ_TO_REG_VAL_900(919100000),
    FREQ_HZ_TO_REG_VAL_900(919700000),

    FREQ_HZ_TO_REG_VAL_900(920300000),
    FREQ_HZ_TO_REG_VAL_900(920900000),
    FREQ_HZ_TO_REG_VAL_900(921500000),
    FREQ_HZ_TO_REG_VAL_900(922100000),

    FREQ_HZ_TO_REG_VAL_900(922700000),
    FREQ_HZ_TO_REG_VAL_900(923300000),
    FREQ_HZ_TO_REG_VAL_900(923900000),
    FREQ_HZ_TO_REG_VAL_900(924500000),

    FREQ_HZ_TO_REG_VAL_900(925100000),
    FREQ_HZ_TO_REG_VAL_900(925700000),
    FREQ_HZ_TO_REG_VAL_900(926300000),
    FREQ_HZ_TO_REG_VAL_900(926900000)};
#endif
#ifdef USE_RX_SX1280
const uint32_t fhssFreqsISM2400[] = {
    FREQ_HZ_TO_REG_VAL_24(2400400000),
    FREQ_HZ_TO_REG_VAL_24(2401400000),
    FREQ_HZ_TO_REG_VAL_24(2402400000),
    FREQ_HZ_TO_REG_VAL_24(2403400000),
    FREQ_HZ_TO_REG_VAL_24(2404400000),

    FREQ_HZ_TO_REG_VAL_24(2405400000),
    FREQ_HZ_TO_REG_VAL_24(2406400000),
    FREQ_HZ_TO_REG_VAL_24(2407400000),
    FREQ_HZ_TO_REG_VAL_24(2408400000),
    FREQ_HZ_TO_REG_VAL_24(2409400000),

    FREQ_HZ_TO_REG_VAL_24(2410400000),
    FREQ_HZ_TO_REG_VAL_24(2411400000),
    FREQ_HZ_TO_REG_VAL_24(2412400000),
    FREQ_HZ_TO_REG_VAL_24(2413400000),
    FREQ_HZ_TO_REG_VAL_24(2414400000),

    FREQ_HZ_TO_REG_VAL_24(2415400000),
    FREQ_HZ_TO_REG_VAL_24(2416400000),
    FREQ_HZ_TO_REG_VAL_24(2417400000),
    FREQ_HZ_TO_REG_VAL_24(2418400000),
    FREQ_HZ_TO_REG_VAL_24(2419400000),

    FREQ_HZ_TO_REG_VAL_24(2420400000),
    FREQ_HZ_TO_REG_VAL_24(2421400000),
    FREQ_HZ_TO_REG_VAL_24(2422400000),
    FREQ_HZ_TO_REG_VAL_24(2423400000),
    FREQ_HZ_TO_REG_VAL_24(2424400000),

    FREQ_HZ_TO_REG_VAL_24(2425400000),
    FREQ_HZ_TO_REG_VAL_24(2426400000),
    FREQ_HZ_TO_REG_VAL_24(2427400000),
    FREQ_HZ_TO_REG_VAL_24(2428400000),
    FREQ_HZ_TO_REG_VAL_24(2429400000),

    FREQ_HZ_TO_REG_VAL_24(2430400000),
    FREQ_HZ_TO_REG_VAL_24(2431400000),
    FREQ_HZ_TO_REG_VAL_24(2432400000),
    FREQ_HZ_TO_REG_VAL_24(2433400000),
    FREQ_HZ_TO_REG_VAL_24(2434400000),

    FREQ_HZ_TO_REG_VAL_24(2435400000),
    FREQ_HZ_TO_REG_VAL_24(2436400000),
    FREQ_HZ_TO_REG_VAL_24(2437400000),
    FREQ_HZ_TO_REG_VAL_24(2438400000),
    FREQ_HZ_TO_REG_VAL_24(2439400000),

    FREQ_HZ_TO_REG_VAL_24(2440400000),
    FREQ_HZ_TO_REG_VAL_24(2441400000),
    FREQ_HZ_TO_REG_VAL_24(2442400000),
    FREQ_HZ_TO_REG_VAL_24(2443400000),
    FREQ_HZ_TO_REG_VAL_24(2444400000),

    FREQ_HZ_TO_REG_VAL_24(2445400000),
    FREQ_HZ_TO_REG_VAL_24(2446400000),
    FREQ_HZ_TO_REG_VAL_24(2447400000),
    FREQ_HZ_TO_REG_VAL_24(2448400000),
    FREQ_HZ_TO_REG_VAL_24(2449400000),

    FREQ_HZ_TO_REG_VAL_24(2450400000),
    FREQ_HZ_TO_REG_VAL_24(2451400000),
    FREQ_HZ_TO_REG_VAL_24(2452400000),
    FREQ_HZ_TO_REG_VAL_24(2453400000),
    FREQ_HZ_TO_REG_VAL_24(2454400000),

    FREQ_HZ_TO_REG_VAL_24(2455400000),
    FREQ_HZ_TO_REG_VAL_24(2456400000),
    FREQ_HZ_TO_REG_VAL_24(2457400000),
    FREQ_HZ_TO_REG_VAL_24(2458400000),
    FREQ_HZ_TO_REG_VAL_24(2459400000),

    FREQ_HZ_TO_REG_VAL_24(2460400000),
    FREQ_HZ_TO_REG_VAL_24(2461400000),
    FREQ_HZ_TO_REG_VAL_24(2462400000),
    FREQ_HZ_TO_REG_VAL_24(2463400000),
    FREQ_HZ_TO_REG_VAL_24(2464400000),

    FREQ_HZ_TO_REG_VAL_24(2465400000),
    FREQ_HZ_TO_REG_VAL_24(2466400000),
    FREQ_HZ_TO_REG_VAL_24(2467400000),
    FREQ_HZ_TO_REG_VAL_24(2468400000),
    FREQ_HZ_TO_REG_VAL_24(2469400000),

    FREQ_HZ_TO_REG_VAL_24(2470400000),
    FREQ_HZ_TO_REG_VAL_24(2471400000),
    FREQ_HZ_TO_REG_VAL_24(2472400000),
    FREQ_HZ_TO_REG_VAL_24(2473400000),
    FREQ_HZ_TO_REG_VAL_24(2474400000),

    FREQ_HZ_TO_REG_VAL_24(2475400000),
    FREQ_HZ_TO_REG_VAL_24(2476400000),
    FREQ_HZ_TO_REG_VAL_24(2477400000),
    FREQ_HZ_TO_REG_VAL_24(2478400000),
    FREQ_HZ_TO_REG_VAL_24(2479400000)};
#endif

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

static void initializeFhssFrequencies(const elrsFreqDomain_e dom) {
    switch (dom) {
#ifdef USE_RX_SX127X
    case AU433:
        fhssFreqs = fhssFreqsAU433;
        numFreqs = sizeof(fhssFreqsAU433) / sizeof(uint32_t);
        break;
    case AU915:
        fhssFreqs = fhssFreqsAU915;
        numFreqs = sizeof(fhssFreqsAU915) / sizeof(uint32_t);
        break;
    case EU433:
        fhssFreqs = fhssFreqsEU433;
        numFreqs = sizeof(fhssFreqsEU433) / sizeof(uint32_t);
        break;
    case EU868:
        fhssFreqs = fhssFreqsEU868;
        numFreqs = sizeof(fhssFreqsEU868) / sizeof(uint32_t);
        break;
    case IN866:
        fhssFreqs = fhssFreqsIN866;
        numFreqs = sizeof(fhssFreqsIN866) / sizeof(uint32_t);
        break;
    case FCC915:
        fhssFreqs = fhssFreqsFCC915;
        numFreqs = sizeof(fhssFreqsFCC915) / sizeof(uint32_t);
        break;
#endif
#ifdef USE_RX_SX1280
    case ISM2400:
        fhssFreqs = fhssFreqsISM2400;
        numFreqs = sizeof(fhssFreqsISM2400) / sizeof(uint32_t);
        break;
#endif
    default:
        fhssFreqs = NULL;
        numFreqs = 0;
    }
}

uint32_t fhssGetInitialFreq(const int32_t freqCorrection)
{
    return fhssFreqs[syncChannel] - freqCorrection;
}

uint8_t fhssGetNumEntries(void)
{
    return numFreqs;
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
    return fhssFreqs[fhssSequence[fhssIndex]] - freqCorrection;
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
    seed = ((long)UID[2] << 24) + ((long)UID[3] << 16) + ((long)UID[4] << 8) + UID[5];

    initializeFhssFrequencies(dom);

    seqCount = (256 / MAX(numFreqs, 1)) * numFreqs;

    syncChannel = numFreqs / 2;

    // initialize the sequence array
    for (uint8_t i = 0; i < seqCount; i++) {
        if (i % numFreqs == 0) {
            fhssSequence[i] = syncChannel;
        } else if (i % numFreqs == syncChannel) {
            fhssSequence[i] = 0;
        } else {
            fhssSequence[i] = i % numFreqs;
        }
    }

    for (uint8_t i = 0; i < seqCount; i++) {
        // if it's not the sync channel
        if (i % numFreqs != 0) {
            uint8_t offset = (i / numFreqs) * numFreqs; // offset to start of current block
            uint8_t rand = rngN(numFreqs - 1) + 1; // random number between 1 and numFreqs

            // switch this entry and another random entry in the same block
            uint8_t temp = fhssSequence[i];
            fhssSequence[i] = fhssSequence[offset + rand];
            fhssSequence[offset + rand] = temp;
        }
    }
}

uint8_t tlmRatioEnumToValue(const elrsTlmRatio_e enumval)
{
    switch (enumval) {
    case TLM_RATIO_NO_TLM:
        return 1;
        break;
    case TLM_RATIO_1_2:
        return 2;
        break;
    case TLM_RATIO_1_4:
        return 4;
        break;
    case TLM_RATIO_1_8:
        return 8;
        break;
    case TLM_RATIO_1_16:
        return 16;
        break;
    case TLM_RATIO_1_32:
        return 32;
        break;
    case TLM_RATIO_1_64:
        return 64;
        break;
    case TLM_RATIO_1_128:
        return 128;
        break;
    default:
        return 0;
    }
}

uint16_t rateEnumToHz(const elrsRfRate_e eRate)
{
    switch (eRate) {
    case RATE_500HZ: return 500;
    case RATE_250HZ: return 250;
    case RATE_200HZ: return 200;
    case RATE_150HZ: return 150;
    case RATE_100HZ: return 100;
    case RATE_50HZ: return 50;
    case RATE_25HZ: return 25;
    case RATE_4HZ: return 4;
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
    if (lqPeriodIsSet()) {
        return;
    }
    lq.array[lq.index] |= lq.mask;
    lq.value += 1;
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
    case 0:
        return 1000;
    case 1:
        return 1275;
    case 2:
        return 1425;
    case 3:
        return 1575;
    case 4:
        return 1725;
    case 5:
        return 2000;
    default:
        return 1500;
    }
}

uint16_t convertSwitchNb(const uint16_t val, const uint16_t max)
{
    return (val > max) ? 1500 : val * 1000 / max + 1000;
}

uint16_t convertAnalog(const uint16_t val)
{
    return CRSF_RC_CHANNEL_SCALE_LEGACY * val + 881;
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

#endif /* USE_RX_EXPRESSLRS */
