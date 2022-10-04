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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/io_types.h"
#include "drivers/time.h"

#include "rx/expresslrs_telemetry.h"

#define ELRS_OTA_VERSION_ID 3

#define ELRS_CRC_LEN 256
#define ELRS_CRC14_POLY 0x2E57

#define ELRS_NR_SEQUENCE_ENTRIES 256
#define ELRS_FREQ_SPREAD_SCALE 256

#define ELRS_RX_TX_BUFF_SIZE 8

#define ELRS_TELEMETRY_TYPE_LINK 0x01
#define ELRS_TELEMETRY_TYPE_DATA 0x02
#define ELRS_MSP_BIND 0x09
#define ELRS_MSP_MODEL_ID 0x0A
#define ELRS_MSP_SET_RX_CONFIG 45

#define ELRS_MODELMATCH_MASK 0x3F

#define FREQ_HZ_TO_REG_VAL_900(freq) ((uint32_t)(freq / SX127x_FREQ_STEP))
#define FREQ_HZ_TO_REG_VAL_24(freq) ((uint32_t)(freq / SX1280_FREQ_STEP))

#define ELRS_RATE_MAX 4
#define ELRS_BINDING_RATE_24 3
#define ELRS_BINDING_RATE_900 2

#define ELRS_MAX_CHANNELS 16
#define ELRS_RSSI_CHANNEL 15
#define ELRS_LQ_CHANNEL 14

#define ELRS_CONFIG_CHECK_MS 200
#define ELRS_LINK_STATS_CHECK_MS 100
#define ELRS_CONSIDER_CONNECTION_GOOD_MS 1000

#define ELRS_MODE_CYCLE_MULTIPLIER_SLOW 10

typedef enum {
#ifdef USE_RX_SX127X
    AU433,
    AU915,
    EU433,
    EU868,
    IN866,
    FCC915,
#endif
#ifdef USE_RX_SX1280
    ISM2400,
    CE2400,
#endif
#if !defined(USE_RX_SX127X) && !defined(USE_RX_SX1280)
    NONE,
#endif
} elrsFreqDomain_e;

typedef enum {
    SM_WIDE = 0,
    SM_HYBRID = 1
} elrsSwitchMode_e;

typedef enum {
    TLM_RATIO_STD = 0,   // Use suggested ratio from ModParams
    TLM_RATIO_NO_TLM,
    TLM_RATIO_1_128,
    TLM_RATIO_1_64,
    TLM_RATIO_1_32,
    TLM_RATIO_1_16,
    TLM_RATIO_1_8,
    TLM_RATIO_1_4,
    TLM_RATIO_1_2,
    TLM_RATIO_DISARMED, // TLM_RATIO_STD when disarmed, TLM_RATIO_NO_TLM when armed
} elrsTlmRatio_e;

typedef enum {
    RATE_LORA_4HZ = 0,
    RATE_LORA_25HZ,
    RATE_LORA_50HZ,
    RATE_LORA_100HZ,
    RATE_LORA_100HZ_8CH,
    RATE_LORA_150HZ,
    RATE_LORA_200HZ,
    RATE_LORA_250HZ,
    RATE_LORA_333HZ_8CH,
    RATE_LORA_500HZ,
    RATE_DVDA_250HZ,
    RATE_DVDA_500HZ,
    RATE_FLRC_500HZ,
    RATE_FLRC_1000HZ,
} elrsRfRate_e; // Max value of 16 since only 4 bits have been assigned in the sync package.

typedef struct elrsModSettings_s {
    uint8_t index;
    elrsRfRate_e enumRate;            // Max value of 16 since only 4 bits have been assigned in the sync package.
    uint8_t bw;
    uint8_t sf;
    uint8_t cr;
    uint32_t interval;                  // interval in us seconds that corresponds to that frequency
    elrsTlmRatio_e tlmInterval;       // every X packets is a response TLM packet, should be a power of 2
    uint8_t fhssHopInterval;            // every X packets we hop to a new frequency. Max value of 16 since only 4 bits have been assigned in the sync package.
    uint8_t preambleLen;
} elrsModSettings_t;

typedef struct elrsRfPerfParams_s {
    int8_t index;
    elrsRfRate_e enumRate;        // Max value of 16 since only 4 bits have been assigned in the sync package.
    int16_t sensitivity;            // expected RF sensitivity based on
    uint16_t toa;                   // time on air in microseconds
    uint16_t disconnectTimeoutMs;   // Time without a packet before receiver goes to disconnected (ms)
    uint16_t rxLockTimeoutMs;       // Max time to go from tentative -> connected state on receiver (ms)
    uint16_t syncPktIntervalDisconnected; // how often to send the SYNC_PACKET packet (ms) when there is no response from RX
    uint16_t syncPktIntervalConnected;    // how often to send the SYNC_PACKET packet (ms) when there we have a connection
} elrsRfPerfParams_t;

typedef struct elrsFhssConfig_s {
    uint8_t  domain;
    uint32_t freqStart;
    uint32_t freqStop;
    uint8_t freqCount;
} elrsFhssConfig_t;

typedef struct elrsOtaPacket_s {
    // The packet type must always be the low two bits of the first byte of the
    // packet to match the same placement in OTA_Packet8_s
    uint8_t type : 2,
            crcHigh : 6;
    union {
        /** PACKET_TYPE_RCDATA **/
        struct {
            uint8_t ch[5];
            uint8_t switches : 7,
                    ch4 : 1;
        } rc;
        /** PACKET_TYPE_MSP **/
        struct {
            uint8_t packageIndex;
            uint8_t payload[ELRS_MSP_BYTES_PER_CALL];
        } msp_ul;
        /** PACKET_TYPE_SYNC **/
        struct {
            uint8_t fhssIndex;
            uint8_t nonce;
            uint8_t switchEncMode : 1,
                    newTlmRatio : 3,
                    rateIndex : 4;
            uint8_t UID3;
            uint8_t UID4;
            uint8_t UID5;
        } sync;
        /** PACKET_TYPE_TLM **/
        struct {
            uint8_t type : ELRS_TELEMETRY_SHIFT,
                    packageIndex : (8 - ELRS_TELEMETRY_SHIFT);
            union {
                struct {
                    uint8_t uplink_RSSI_1 : 7,
                            antenna : 1;
                    uint8_t uplink_RSSI_2 : 7,
                            modelMatch : 1;
                    uint8_t lq : 7,
                            mspConfirm : 1;
                    int8_t SNR;
                    uint8_t free;
                } ul_link_stats;
                uint8_t payload[ELRS_TELEMETRY_BYTES_PER_CALL];
            };
        } tlm_dl;
    };
    uint8_t crcLow;
} __attribute__ ((__packed__)) elrsOtaPacket_t;

typedef bool (*elrsRxInitFnPtr)(IO_t resetPin, IO_t busyPin);
typedef void (*elrsRxConfigFnPtr)(const uint8_t bw, const uint8_t sf, const uint8_t cr, const uint32_t freq, const uint8_t preambleLen, const bool iqInverted);
typedef void (*elrsRxStartReceivingFnPtr)(void);
typedef void (*elrsRxISRFnPtr)(void);
typedef void (*elrsRxHandleFromTockFnPtr)(void);
typedef bool (*elrsRxBusyTimeoutFnPtr)(void);
typedef void (*elrsRxTransmitDataFnPtr)(const uint8_t *data, const uint8_t length);
typedef void (*elrsRxReceiveDataFnPtr)(uint8_t *data, const uint8_t length);
typedef void (*elrsRxgetRfLinkInfoFnPtr)(int8_t *rssi, int8_t *snr);
typedef void (*elrsRxSetFrequencyFnPtr)(const uint32_t freq);
typedef void (*elrsRxHandleFreqCorrectionFnPtr)(int32_t offset, const uint32_t freq);

extern elrsModSettings_t airRateConfig[][ELRS_RATE_MAX];
extern elrsRfPerfParams_t rfPerfConfig[][ELRS_RATE_MAX];

void generateCrc14Table(void);
uint16_t calcCrc14(uint8_t *data, uint8_t len, uint16_t crc);

uint32_t fhssGetInitialFreq(const int32_t freqCorrection);
uint8_t fhssGetNumEntries(void);
uint8_t fhssGetCurrIndex(void);
void fhssSetCurrIndex(const uint8_t value);
uint32_t fhssGetNextFreq(const int32_t freqCorrection);
void fhssGenSequence(const uint8_t UID[], const elrsFreqDomain_e dom);
uint8_t tlmRatioEnumToValue(const elrsTlmRatio_e enumval);
uint16_t rateEnumToHz(const elrsRfRate_e eRate);
uint16_t txPowerIndexToValue(const uint8_t index);

//
// Link Quality
//
void lqReset(void);
void lqNewPeriod(void);
bool lqPeriodIsSet(void);
void lqIncrease(void);
uint8_t lqGet(void);

uint16_t convertSwitch1b(const uint16_t val);
uint16_t convertSwitch3b(const uint16_t val);
uint16_t convertSwitchNb(const uint16_t val, const uint16_t max);
uint8_t hybridWideNonceToSwitchIndex(const uint8_t nonce);

uint8_t airRateIndexToIndex24(uint8_t airRate, uint8_t currentIndex);
uint8_t airRateIndexToIndex900(uint8_t airRate, uint8_t currentIndex);
