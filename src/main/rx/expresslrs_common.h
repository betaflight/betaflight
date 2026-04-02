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

#ifdef USE_ELRSV3
#define ELRS_OTA_VERSION_ID 3
#else
#define ELRS_OTA_VERSION_ID 4
#endif

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
#define FREQ_HZ_TO_REG_VAL_24(freq)  ((uint32_t)(freq / SX1280_FREQ_STEP))

#define ELRS_RATE_MAX_24  6
#define ELRS_RATE_MAX_900 4
#define ELRS_RATE_MAX     ((ELRS_RATE_MAX_24 > ELRS_RATE_MAX_900) ? ELRS_RATE_MAX_24 : ELRS_RATE_MAX_900)
#define ELRS_BINDING_RATE_24  5
#define ELRS_BINDING_RATE_900 2

#define ELRS_MAX_CHANNELS 16
#define ELRS_RSSI_CHANNEL 15
#define ELRS_LQ_CHANNEL 14
#define ELRS_ARM_V3_CHANNEL 4
#define ELRS_ARM_V4_CHANNEL 13

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

typedef enum : uint8_t
{
    // RATE_MODULATION_BAND_RATE_MODE for ELRS V4
    RATE_LORA_900_25HZ = 0,
    RATE_LORA_900_50HZ,
    RATE_LORA_900_100HZ,
    RATE_LORA_900_100HZ_8CH,
    RATE_LORA_900_150HZ,
    RATE_LORA_900_200HZ,
    RATE_LORA_900_200HZ_8CH,
    RATE_LORA_900_250HZ,
    RATE_LORA_900_333HZ_8CH,
    RATE_LORA_900_500HZ,
    RATE_LORA_900_50HZ_DVDA,
    RATE_FSK_900_1000HZ_8CH,

    RATE_LORA_2G4_25HZ = 20,
    RATE_LORA_2G4_50HZ,
    RATE_LORA_2G4_100HZ,
    RATE_LORA_2G4_100HZ_8CH,
    RATE_LORA_2G4_150HZ,
    RATE_LORA_2G4_200HZ,
    RATE_LORA_2G4_200HZ_8CH,
    RATE_LORA_2G4_250HZ,
    RATE_LORA_2G4_333HZ_8CH,
    RATE_LORA_2G4_500HZ,
    RATE_FLRC_2G4_250HZ_DVDA,
    RATE_FLRC_2G4_500HZ_DVDA,
    RATE_FLRC_2G4_500HZ,
    RATE_FLRC_2G4_1000HZ,
    RATE_FSK_2G4_250HZ_DVDA,
    RATE_FSK_2G4_500HZ_DVDA,
    RATE_FSK_2G4_1000HZ,

    RATE_LORA_DUAL_100HZ_8CH = 100,
    RATE_LORA_DUAL_150HZ,
} expresslrs_SyncV4_RFrates_e;

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
} elrsRfRate_e;

typedef enum {
    RADIO_TYPE_SX127x_LORA,
    RADIO_TYPE_SX128x_LORA,
    RADIO_TYPE_SX128x_FLRC,
} elrsRadioType_e;

typedef struct elrsModSettings_s {
    uint8_t index;
    elrsRadioType_e radioType;        // elrsRadioType_e
    elrsRfRate_e enumRate;            // Max value of 16 since only 4 bits have been assigned in the sync package.
    expresslrs_SyncV4_RFrates_e v4Rate; // Band-independent rate sent in V4 sync packet
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

#ifdef USE_ELRSV3

typedef struct {
    uint8_t uplink_RSSI_1:7,
            antenna:1;
    uint8_t uplink_RSSI_2:7,
            modelMatch:1;
    uint8_t lq:7,
            mspConfirm:1;
    int8_t SNR;
} __attribute__ ((__packed__)) OTA_LinkStats_s;

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
                    OTA_LinkStats_s stats;
                    uint8_t free;
                } ul_link_stats;
                uint8_t payload[ELRS_TELEMETRY_BYTES_PER_CALL];
            };
        } tlm_dl;
    };
    uint8_t crcLow;
} __attribute__ ((__packed__)) elrsOtaPacket_t;
#else

typedef struct {
    uint8_t uplink_RSSI_1:7,
            antenna:1;
    uint8_t uplink_RSSI_2:7,
            modelMatch:1;
    uint8_t lq:7,
            trueDiversityAvailable:1;
    int8_t SNR;
} __attribute__ ((__packed__)) OTA_LinkStats_s;

typedef struct elrsOtaPacket_s {
    // The packet type must always be the low two bits of the first byte of the
    // packet to match the same placement in OTA_Packet8_s
    uint8_t type: 2,
            crcHigh: 6;
    union {
        /** PACKET_TYPE_RCDATA **/
        struct {
            uint8_t ch[5]; // OTA_Channels_4x10
            uint8_t switches:7, // includes stubbornAck
                    ch4:1;
        } rc;
        /** PACKET_TYPE_DATA uplink (to RX) **/
        struct {
            uint8_t packageIndex:7,
                    mspConfirm:1;
            uint8_t payload[ELRS_TELEMETRY_BYTES_PER_CALL];
        } msp_ul; // data_ul
        /** PACKET_TYPE_SYNC **/
         struct {
            uint8_t fhssIndex;
            uint8_t nonce;
            uint8_t rateIndex; // Actually rfRateEnum in V4
            uint8_t switchEncMode:1,
                newTlmRatio:3,
                geminiMode:1,
                otaProtocol:2,
                free:1;
            uint8_t UID4;
            uint8_t UID5;
        } __attribute__ ((__packed__)) sync; // OTA_Sync_s
        /** PACKET_TYPE_DATA / PACKET_TYPE_LINKSTATS downlink (to TX) **/
        struct {
            uint8_t packageIndex:7,
                    mspConfirm:1;
            union {
                struct {
                    OTA_LinkStats_s stats;
                    uint8_t payload[ELRS_TELEMETRY_BYTES_PER_CALL - sizeof(OTA_LinkStats_s)];
                } __attribute__ ((__packed__)) ul_link_stats;
                uint8_t payload[ELRS_TELEMETRY_BYTES_PER_CALL];
            };
        } tlm_dl; // data_dl
    };
    uint8_t crcLow;
} __attribute__ ((__packed__)) elrsOtaPacket_t;
#endif

typedef bool (*elrsRxInitFnPtr)(IO_t resetPin, IO_t busyPin);
typedef void (*elrsRxConfigFnPtr)(const uint8_t bw, const uint8_t sfbt, const uint8_t cr,
    const uint32_t freq, const uint8_t preambleLength, const bool iqInverted,
    const uint32_t flrcSyncWord, const uint16_t flrcCrcSeed, const bool isFlrc);
typedef void (*elrsRxStartReceivingFnPtr)(void);
typedef void (*elrsRxISRFnPtr)(void);
typedef void (*elrsRxHandleFromTockFnPtr)(void);
typedef bool (*elrsRxBusyTimeoutFnPtr)(void);
typedef void (*elrsRxTransmitDataFnPtr)(const uint8_t *data, const uint8_t length);
typedef void (*elrsRxReceiveDataFnPtr)(uint8_t *data, const uint8_t length);
typedef void (*elrsRxgetRfLinkInfoFnPtr)(int8_t *rssi, int8_t *snr);
typedef void (*elrsRxSetFrequencyFnPtr)(const uint32_t freq);
typedef void (*elrsRxHandleFreqCorrectionFnPtr)(int32_t *offset, const uint32_t freq);

extern elrsModSettings_t airRateConfig[][ELRS_RATE_MAX];
extern elrsRfPerfParams_t rfPerfConfig[][ELRS_RATE_MAX];

void generateCrc14Table(void);
uint16_t calcCrc14(uint8_t *data, uint8_t len, uint16_t crc);

uint32_t fhssGetInitialFreq(const int32_t freqCorrection);
uint8_t fhssGetNumEntries(void);
uint8_t fhssGetCurrIndex(void);
void fhssSetCurrIndex(const uint8_t value);
uint32_t fhssGetNextFreq(const int32_t freqCorrection);
void fhssGenSequence(const uint32_t uidSeed, const elrsFreqDomain_e dom);
uint8_t tlmRatioEnumToValue(const elrsTlmRatio_e enumval);
uint16_t rateEnumToHz(const elrsRfRate_e eRate);
uint16_t txPowerIndexToValue(const uint8_t index);
uint32_t elrsUidToSeed(const uint8_t UID[]);
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
