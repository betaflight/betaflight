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

// expresslrs packet header types
// 00 -> standard 4 channel data packet
// 01 -> switch data packet
// 11 -> tlm packet
// 10 -> sync packet with hop data
typedef enum {
    ELRS_RC_DATA_PACKET = 0x00,
    ELRS_MSP_DATA_PACKET = 0x01,
    ELRS_SYNC_PACKET = 0x02,
    ELRS_TLM_PACKET = 0x03,
} elrsPacketType_e;

typedef enum {
    ELRS_DIO_UNKNOWN = 0,
    ELRS_DIO_RX_DONE = 1,
    ELRS_DIO_TX_DONE = 2,
    ELRS_DIO_RX_AND_TX_DONE = 3,
} dioReason_e;

typedef enum {
    ELRS_CONNECTED,
    ELRS_TENTATIVE,
    ELRS_DISCONNECTED,
    ELRS_DISCONNECT_PENDING // used on modelmatch change to drop the connection
} connectionState_e;

typedef enum {
    ELRS_TIM_DISCONNECTED = 0,
    ELRS_TIM_TENTATIVE = 1,
    ELRS_TIM_LOCKED = 2
} timerState_e;

typedef struct elrsReceiver_s {

    IO_t resetPin;
    IO_t busyPin;

    int32_t freqOffset;
    uint32_t currentFreq;

    volatile uint8_t nonceRX; // nonce that we THINK we are up to.

    elrsModSettings_t *modParams;
    elrsRfPerfParams_t *rfPerfParams;

    const uint8_t *UID;

    int8_t rssi;
    int8_t snr;
    int8_t rssiFiltered;
#ifdef USE_RX_RSNR
    int8_t rsnrFiltered;
#endif //USE_RX_RSNR

    uint8_t uplinkLQ;

    bool alreadyFhss;
    bool alreadyTelemResp;
    bool lockRFmode;
    bool started;

    timerState_e timerState;
    connectionState_e connectionState;

    uint8_t rfModeCycleMultiplier;
    uint16_t cycleIntervalMs;
    uint32_t rfModeCycledAtMs;
    uint8_t rateIndex;
    uint8_t nextRateIndex;

    uint32_t gotConnectionMs;
    uint32_t lastSyncPacketMs;
    uint32_t lastValidPacketMs;

    uint32_t configCheckedAtMs;
    bool configChanged;

    bool inBindingMode;
    volatile bool initializeReceiverPending;
    volatile bool fhssRequired;
    volatile bool didFhss;

    uint32_t statsUpdatedAtMs;

    elrsRxInitFnPtr init;
    elrsRxConfigFnPtr config;
    elrsRxStartReceivingFnPtr startReceiving;
    elrsRxISRFnPtr rxISR;
    elrsRxHandleFromTockFnPtr rxHandleFromTock;
    elrsRxBusyTimeoutFnPtr rxHandleFromTick;
    elrsRxgetRfLinkInfoFnPtr getRfLinkInfo;
    elrsRxSetFrequencyFnPtr setFrequency;
    elrsRxHandleFreqCorrectionFnPtr handleFreqCorrection;

    timerOvrHandlerRec_t timerUpdateCb;
} elrsReceiver_t;

