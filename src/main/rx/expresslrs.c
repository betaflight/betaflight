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
 *
 * Authors:
 * Phobos- - Original port.
 * Dominic Clifton/Hydra - Timer-based timeout implementation.
 * Phobos- - Port of v2.0
 */

#include <stdlib.h>
#include <string.h>
#include "platform.h"

#ifdef USE_RX_EXPRESSLRS

#include "build/atomic.h"
#include "build/debug.h"
#include "build/debug_pin.h"

#include "common/maths.h"
#include "common/filter.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/rx/rx_sx127x.h"
#include "drivers/rx/rx_sx1280.h"
#include "drivers/rx/expresslrs_driver.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/init.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_expresslrs.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/rx_spi_common.h"

#include "rx/expresslrs.h"
#include "rx/expresslrs_common.h"
#include "rx/expresslrs_impl.h"
#include "rx/expresslrs_telemetry.h"

UNIT_TESTED elrsReceiver_t receiver;
static const uint8_t BindingUID[6] = {0,1,2,3,4,5}; // Special binding UID values
static uint16_t crcInitializer = 0;
static uint8_t bindingRateIndex = 0;
static bool connectionHasModelMatch = false;
static uint8_t txPower = 0;
static uint8_t wideSwitchIndex = 0;
static uint8_t currTlmDenom = 1;
static simpleLowpassFilter_t rssiFilter;
#ifdef USE_RX_RSNR
static simpleLowpassFilter_t rsnrFilter;
#endif //USE_RX_RSNR
static meanAccumulator_t snrFilter; //for dyn power purposes

static volatile DMA_DATA uint8_t dmaBuffer[ELRS_RX_TX_BUFF_SIZE];
static volatile DMA_DATA uint8_t telemetryPacket[ELRS_RX_TX_BUFF_SIZE];
static volatile rx_spi_received_e rfPacketStatus = RX_SPI_RECEIVED_NONE;
static volatile uint8_t *payload;

static void rssiFilterReset(void)
{
    simpleLPFilterInit(&rssiFilter, 2, 5);
#ifdef USE_RX_RSNR
    simpleLPFilterInit(&rsnrFilter, 2, 5);
#endif //USE_RX_RSNR
}

#define PACKET_HANDLING_TO_TOCK_ISR_DELAY_US 250

//
// Event pair recorder
//

typedef enum {
    EPR_FIRST,
    EPR_SECOND,
} eprEvent_e;

#define EPR_EVENT_COUNT 2

typedef struct eprState_s {
    uint32_t eventAtUs[EPR_EVENT_COUNT];
    bool eventRecorded[EPR_EVENT_COUNT];
} eprState_t;

eprState_t eprState = {
    .eventAtUs = {0},
    .eventRecorded = {0},
};

static void phaseLockEprEvent(eprEvent_e event, uint32_t currentTimeUs)
{
    eprState.eventAtUs[event] = currentTimeUs;
    eprState.eventRecorded[event] = true;
}

static bool phaseLockEprHaveBothEvents(void)
{
    bool bothEventsRecorded = eprState.eventRecorded[EPR_SECOND] && eprState.eventRecorded[EPR_FIRST];
    return bothEventsRecorded;
}

static int32_t phaseLockEprResult(void)
{
    if (phaseLockEprHaveBothEvents()) {
        return (int32_t)(eprState.eventAtUs[EPR_SECOND] - eprState.eventAtUs[EPR_FIRST]);
    }

    return 0;
}

static void phaseLockEprReset(void)
{
    memset(&eprState, 0, sizeof(eprState_t));
}


//
// Phase Lock
//


#define EPR_INTERNAL EPR_FIRST
#define EPR_EXTERNAL EPR_SECOND

typedef struct phaseLockState_s {
    simpleLowpassFilter_t offsetFilter;
    simpleLowpassFilter_t offsetDxFilter;

    int32_t rawOffsetUs;
    int32_t previousRawOffsetUs;

    int32_t offsetUs;
    int32_t offsetDeltaUs;
    int32_t previousOffsetUs;
} phaseLockState_t;

static phaseLockState_t pl;

static void expressLrsPhaseLockReset(void)
{
    simpleLPFilterInit(&pl.offsetFilter, 2, 5);
    simpleLPFilterInit(&pl.offsetDxFilter, 4, 5);

    phaseLockEprReset();
}

static uint8_t nextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;

static uint8_t telemetryBurstCount = 1;
static uint8_t telemetryBurstMax = 1;
static bool telemBurstValid = false;

// Maximum ms between LINK_STATISTICS packets for determining burst max
#define TELEM_MIN_LINK_INTERVAL 512U

#ifdef USE_MSP_OVER_TELEMETRY
static uint8_t mspBuffer[ELRS_MSP_BUFFER_SIZE];
#endif

//
// Stick unpacking
//

static void setRssiChannelData(uint16_t *rcData)
{
    rcData[ELRS_LQ_CHANNEL] = scaleRange(receiver.uplinkLQ, 0, 100, 988, 2011);
    rcData[ELRS_RSSI_CHANNEL] = scaleRange(constrain(receiver.rssiFiltered, receiver.rfPerfParams->sensitivity, -50), receiver.rfPerfParams->sensitivity, -50, 988, 2011); 
}

static void unpackAnalogChannelData(uint16_t *rcData, volatile elrsOtaPacket_t const * const otaPktPtr)
{
    const uint8_t numOfChannels = 4;
    const uint8_t srcBits = 10;
    const uint16_t inputChannelMask = (1 << srcBits) - 1;

    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    uint8_t readByteIndex = 0;
    for (uint8_t n = 0; n < numOfChannels; n++) {
        while (bitsMerged < srcBits) {
            uint8_t readByte = otaPktPtr->rc.ch[readByteIndex++];
            readValue |= ((uint32_t) readByte) << bitsMerged;
            bitsMerged += 8;
        }
        rcData[n] = 988 + (readValue & inputChannelMask);
        readValue >>= srcBits;
        bitsMerged -= srcBits;
    }

    // The low latency switch
    rcData[4] = convertSwitch1b(otaPktPtr->rc.ch4);
}

/**
 * Hybrid switches uses 10 bits for each analog channel,
 * 2 bits for the low latency switch[0]
 * 3 bits for the round-robin switch index and 2 bits for the value
 * 4 analog channels, 1 low latency switch and round robin switch data = 47 bits (1 free)
 * 
 * sets telemetry status bit
 */
static void unpackChannelDataHybridSwitch8(uint16_t *rcData, volatile elrsOtaPacket_t const * const otaPktPtr)
{
    unpackAnalogChannelData(rcData, otaPktPtr);

    const uint8_t switchByte = otaPktPtr->rc.switches;

    // The round-robin switch, switchIndex is actually index-1 
    // to leave the low bit open for switch 7 (sent as 0b11x)
    // where x is the high bit of switch 7
    uint8_t switchIndex = (switchByte & 0x38) >> 3;

    if (switchIndex >= 6) {
        // Because AUX1 (index 0) is the low latency switch, the low bit
        // of the switchIndex can be used as data, and arrives as index "6"
        rcData[11] = convertSwitchNb(switchByte & 0x0F, 15); //4-bit
    } else {
        rcData[5 + switchIndex] = convertSwitch3b(switchByte & 0x07);
    }

    setRssiChannelData(rcData);
}

/**
 * HybridWide switches decoding of over the air data
 *
 * Hybrid switches uses 10 bits for each analog channel,
 * 1 bits for the low latency switch[0]
 * 6 or 7 bits for the round-robin switch
 * 1 bit for the TelemetryStatus, which may be in every packet or just idx 7
 * depending on TelemetryRatio
 *
 * Output: crsf.PackedRCdataOut, crsf.LinkStatistics.uplink_TX_Power
 * Returns: TelemetryStatus bit
 */
static void unpackChannelDataHybridWide(uint16_t *rcData, volatile elrsOtaPacket_t const * const otaPktPtr)
{
    unpackAnalogChannelData(rcData, otaPktPtr);
    const uint8_t switchByte = otaPktPtr->rc.switches;

    // The round-robin switch, 6-7 bits with the switch index implied by the nonce. Some logic moved to processRFPacket
    if (wideSwitchIndex >= 7) {
        txPower = switchByte & 0x3F;
    } else {
        uint8_t bins;
        uint16_t switchValue;
        if (currTlmDenom < 8) {
            bins = 63;
            switchValue = switchByte & 0x3F; // 6-bit
        } else {
            bins = 127;
            switchValue = switchByte & 0x7F; // 7-bit
        }

        rcData[5 + wideSwitchIndex] = convertSwitchNb(switchValue, bins);
    }

    setRssiChannelData(rcData);
}

static uint8_t minLqForChaos(void)
{
    // Determine the most number of CRC-passing packets we could receive on
    // a single channel out of 100 packets that fill the LQcalc span.
    // The LQ must be GREATER THAN this value, not >=
    // The amount of time we coexist on the same channel is
    // 100 divided by the total number of packets in a FHSS loop (rounded up)
    // and there would be 4x packets received each time it passes by so
    // FHSShopInterval * ceil(100 / FHSShopInterval * numfhss) or
    // FHSShopInterval * trunc((100 + (FHSShopInterval * numfhss) - 1) / (FHSShopInterval * numfhss))
    // With a interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
    const uint32_t numfhss = fhssGetNumEntries();
    const uint8_t interval = receiver.modParams->fhssHopInterval;
    return interval * ((interval * numfhss + 99) / (interval * numfhss));
}

static bool domainIsTeam24(void)
{
#ifdef USE_RX_SX1280
    const elrsFreqDomain_e domain = rxExpressLrsSpiConfig()->domain;
    return (domain == ISM2400) || (domain == CE2400);
#else 
    return false;
#endif
}

static void setRfLinkRate(const uint8_t index)
{
#if defined(USE_RX_SX1280) && defined(USE_RX_SX127X)
    const uint8_t domainIdx = domainIsTeam24() ? 1 : 0;
    receiver.modParams = &airRateConfig[domainIdx][index];
    receiver.rfPerfParams = &rfPerfConfig[domainIdx][index];
#else
    receiver.modParams = &airRateConfig[0][index];
    receiver.rfPerfParams = &rfPerfConfig[0][index];
#endif
    receiver.currentFreq = fhssGetInitialFreq(receiver.freqOffset);
    // Wait for (11/10) 110% of time it takes to cycle through all freqs in FHSS table (in ms)
    receiver.cycleIntervalMs = ((uint32_t)11U * fhssGetNumEntries() * receiver.modParams->fhssHopInterval * receiver.modParams->interval) / (10U * 1000U);

    receiver.config(receiver.modParams->bw, receiver.modParams->sf, receiver.modParams->cr, receiver.currentFreq, receiver.modParams->preambleLen, receiver.UID[5] & 0x01);
#if defined(USE_RX_SX1280)
    if (rxExpressLrsSpiConfig()->domain == CE2400)
      sx1280SetOutputPower(10);
#endif

    expressLrsUpdateTimerInterval(receiver.modParams->interval);

    rssiFilterReset();
    receiver.nextRateIndex = index; // presumably we just handled this
    telemBurstValid = false;

#ifdef USE_RX_LINK_QUALITY_INFO
    rxSetRfMode((uint8_t)receiver.modParams->enumRate);
#endif
}

uint32_t expressLrsGetCurrentFreq(void)
{
    return receiver.currentFreq;
}

void expressLrsSetRfPacketStatus(rx_spi_received_e status)
{
    rfPacketStatus = status;
}

volatile uint8_t *expressLrsGetRxBuffer(void) {
    return dmaBuffer;
}

volatile uint8_t *expressLrsGetTelemetryBuffer(void)
{
    return telemetryPacket;
}

volatile uint8_t *expressLrsGetPayloadBuffer(void)
{
    return payload;
}

bool expressLrsIsFhssReq(void)
{
    uint8_t modresultFHSS = (receiver.nonceRX + 1) % receiver.modParams->fhssHopInterval;

    if ((receiver.modParams->fhssHopInterval == 0) || receiver.alreadyFhss == true || receiver.inBindingMode || (modresultFHSS != 0) || (receiver.connectionState == ELRS_DISCONNECTED)) {
        return false;
    }

    receiver.alreadyFhss = true;
    receiver.currentFreq = fhssGetNextFreq(receiver.freqOffset);

    return true;
}

bool expressLrsTelemRespReq(void)
{
    uint8_t modresult = (receiver.nonceRX + 1) % currTlmDenom;
    if (receiver.inBindingMode || (receiver.connectionState == ELRS_DISCONNECTED) || (currTlmDenom == 1) || (modresult != 0)) {
        return false; // don't bother sending tlm if disconnected or TLM is off
    } else {
        return true;
    }
}

static void expressLrsSendTelemResp(void)
{
    elrsOtaPacket_t otaPkt = {0};

    receiver.alreadyTelemResp = true;
    otaPkt.type = ELRS_TLM_PACKET;

    if (nextTelemetryType == ELRS_TELEMETRY_TYPE_LINK || !isTelemetrySenderActive()) {
        otaPkt.tlm_dl.type = ELRS_TELEMETRY_TYPE_LINK;
        otaPkt.tlm_dl.ul_link_stats.uplink_RSSI_1 = receiver.rssiFiltered > 0 ? 0 : -receiver.rssiFiltered; 
        otaPkt.tlm_dl.ul_link_stats.uplink_RSSI_2 = 0; //diversity not supported
        otaPkt.tlm_dl.ul_link_stats.antenna = 0;
        otaPkt.tlm_dl.ul_link_stats.modelMatch = connectionHasModelMatch;
        otaPkt.tlm_dl.ul_link_stats.lq = receiver.uplinkLQ;
        otaPkt.tlm_dl.ul_link_stats.SNR = meanAccumulatorCalc(&snrFilter, -16);
#ifdef USE_MSP_OVER_TELEMETRY
        otaPkt.tlm_dl.ul_link_stats.mspConfirm = getCurrentMspConfirm() ? 1 : 0;
#else
        otaPkt.tlm_dl.ul_link_stats.mspConfirm = 0;
#endif
        nextTelemetryType = ELRS_TELEMETRY_TYPE_DATA;
        // Start the count at 1 because the next will be DATA and doing +1 before checking
        // against Max below is for some reason 10 bytes more code
        telemetryBurstCount = 1;
    } else {
        if (telemetryBurstCount < telemetryBurstMax) {
            telemetryBurstCount++;
        } else {
            nextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
        }

        otaPkt.tlm_dl.type = ELRS_TELEMETRY_TYPE_DATA;
        otaPkt.tlm_dl.packageIndex = getCurrentTelemetryPayload(otaPkt.tlm_dl.payload);
    }

    uint16_t crc = calcCrc14((uint8_t *) &otaPkt, 7, crcInitializer);
    otaPkt.crcHigh = (crc >> 8);
    otaPkt.crcLow = crc;
    memcpy((uint8_t *) telemetryPacket, (uint8_t *) &otaPkt, ELRS_RX_TX_BUFF_SIZE);
}

static void updatePhaseLock(void)
{
    if (receiver.connectionState != ELRS_DISCONNECTED && phaseLockEprHaveBothEvents()) {
        int32_t maxOffset = receiver.modParams->interval / 4;
        pl.rawOffsetUs = constrain(phaseLockEprResult(), -maxOffset, maxOffset);

        pl.offsetUs = simpleLPFilterUpdate(&pl.offsetFilter, pl.rawOffsetUs);
        pl.offsetDeltaUs = simpleLPFilterUpdate(&pl.offsetDxFilter, pl.rawOffsetUs - pl.previousRawOffsetUs);

        if (receiver.timerState == ELRS_TIM_LOCKED) {
            // limit rate of freq offset adjustment, use slot 1 because
            // telemetry can fall on slot 1 and will never get here
            if (receiver.nonceRX % 8 == 1) {
                if (pl.offsetUs > 0) {
                    expressLrsTimerIncreaseFrequencyOffset();
                } else if (pl.offsetUs < 0) {
                    expressLrsTimerDecreaseFrequencyOffset();
                }
            }
        }

        if (receiver.connectionState != ELRS_CONNECTED) {
            expressLrsUpdatePhaseShift(pl.rawOffsetUs >> 1);
        } else {
            expressLrsUpdatePhaseShift(pl.offsetUs >> 2);
        }

        pl.previousOffsetUs = pl.offsetUs;
        pl.previousRawOffsetUs = pl.rawOffsetUs;

        expressLrsTimerDebug();

        DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 0, pl.rawOffsetUs);
        DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 1, pl.offsetUs);
    }

    phaseLockEprReset();
}

//hwTimerCallbackTick
void expressLrsOnTimerTickISR(void) // this is 180 out of phase with the other callback, occurs mid-packet reception
{
    updatePhaseLock();
    receiver.nonceRX += 1;

    // Save the LQ value before the inc() reduces it by 1
    receiver.uplinkLQ = lqGet();
    // Only advance the LQI period counter if we didn't send Telemetry this period
    if (!receiver.alreadyTelemResp) {
        lqNewPeriod();
    }

    receiver.alreadyTelemResp = false;
    receiver.alreadyFhss = false;

    receiver.rxHandleFromTick();
}

//hwTimerCallbackTock
void expressLrsOnTimerTockISR(void)
{
    uint32_t currentTimeUs = micros();

    phaseLockEprEvent(EPR_INTERNAL, currentTimeUs);

    receiver.rxHandleFromTock();
}

static uint16_t lostConnectionCounter = 0;

void lostConnection(void)
{
    lostConnectionCounter++;

    receiver.rfModeCycleMultiplier = 1;
    receiver.connectionState = ELRS_DISCONNECTED; //set lost connection
    receiver.timerState = ELRS_TIM_DISCONNECTED;
    expressLrsTimerResetFrequencyOffset();
    receiver.freqOffset = 0;
    pl.offsetUs = 0;
    pl.offsetDeltaUs = 0;
    pl.rawOffsetUs = 0;
    pl.previousRawOffsetUs = 0;
    receiver.gotConnectionMs = 0;
    receiver.uplinkLQ = 0;
    lqReset();
    expressLrsPhaseLockReset();
    receiver.alreadyTelemResp = false;
    receiver.alreadyFhss = false;

    if (!receiver.inBindingMode) {
        expressLrsTimerStop();
        setRfLinkRate(receiver.nextRateIndex); // also sets to initialFreq
        receiver.startReceiving();
    }
}

static void tentativeConnection(const uint32_t timeStampMs)
{
    receiver.connectionState = ELRS_TENTATIVE;
    connectionHasModelMatch = false;
    receiver.timerState = ELRS_TIM_DISCONNECTED;
    receiver.freqOffset = 0;
    pl.offsetUs = 0;
    pl.previousRawOffsetUs = 0;
    expressLrsPhaseLockReset(); //also resets PFD
    meanAccumulatorInit(&snrFilter);
    receiver.rfModeCycledAtMs = timeStampMs; // give another 3 sec for lock to occur

    // The caller MUST call hwTimer.resume(). It is not done here because
    // the timer ISR will fire immediately and preempt any other code
}

static void gotConnection(const uint32_t timeStampMs)
{
    if (receiver.connectionState == ELRS_CONNECTED) {
        return; // Already connected
    }

    receiver.lockRFmode = true; // currently works as if LOCK_ON_FIRST_CONNECTION was enabled

    receiver.connectionState = ELRS_CONNECTED; //we got a packet, therefore no lost connection
    receiver.timerState = ELRS_TIM_TENTATIVE;
    receiver.gotConnectionMs = timeStampMs;

    if (rxExpressLrsSpiConfig()->rateIndex != receiver.rateIndex) {
        rxExpressLrsSpiConfigMutable()->rateIndex = receiver.rateIndex;
        receiver.configChanged = true;
    }
}

//setup radio
static void initializeReceiver(void)
{
    fhssGenSequence(receiver.UID, rxExpressLrsSpiConfig()->domain);
    lqReset();
    receiver.nonceRX = 0;
    receiver.freqOffset = 0;
    receiver.configChanged = false;
    receiver.rssi = 0;
    receiver.rssiFiltered = 0;
#ifdef USE_RX_RSNR
    receiver.rsnrFiltered = 0;
#endif //USE_RX_RSNR
    receiver.snr = 0;
    receiver.uplinkLQ = 0;
    receiver.rateIndex = receiver.inBindingMode ? bindingRateIndex : rxExpressLrsSpiConfig()->rateIndex;
    setRfLinkRate(receiver.rateIndex);

    receiver.started = false;
    receiver.alreadyFhss = false;
    receiver.alreadyTelemResp = false;
    receiver.lockRFmode = false;
    receiver.timerState = ELRS_TIM_DISCONNECTED;
    receiver.connectionState = ELRS_DISCONNECTED;

    uint32_t timeStampMs = millis();

    receiver.rfModeCycledAtMs = timeStampMs;
    receiver.configCheckedAtMs = timeStampMs;
    receiver.statsUpdatedAtMs = timeStampMs;
    receiver.gotConnectionMs = timeStampMs;
    receiver.lastSyncPacketMs = timeStampMs;
    receiver.lastValidPacketMs = timeStampMs;

    receiver.rfModeCycleMultiplier = 1;
}

static void unpackBindPacket(volatile uint8_t *packet)
{
    rxExpressLrsSpiConfigMutable()->UID[2] = packet[0];
    rxExpressLrsSpiConfigMutable()->UID[3] = packet[1];
    rxExpressLrsSpiConfigMutable()->UID[4] = packet[2];
    rxExpressLrsSpiConfigMutable()->UID[5] = packet[3];

    receiver.UID = rxExpressLrsSpiConfigMutable()->UID;
    crcInitializer = (receiver.UID[4] << 8) | receiver.UID[5];
    crcInitializer ^= ELRS_OTA_VERSION_ID;
    receiver.inBindingMode = false;
    receiver.configChanged = true; //after initialize as it sets it to false
}

/**
 * Process the assembled MSP packet in mspBuffer[]
 **/
static void processRFMspPacket(volatile elrsOtaPacket_t const * const otaPktPtr)
{
    // Always examine MSP packets for bind information if in bind mode
    // [1] is the package index, first packet of the MSP
    if (receiver.inBindingMode && otaPktPtr->msp_ul.packageIndex == 1 && otaPktPtr->msp_ul.payload[0] == ELRS_MSP_BIND) {
        unpackBindPacket((uint8_t *) &otaPktPtr->msp_ul.payload[1]); //onELRSBindMSP
        return;
    }

#ifdef USE_MSP_OVER_TELEMETRY
    // Must be fully connected to process MSP, prevents processing MSP
    // during sync, where packets can be received before connection
    if (receiver.connectionState != ELRS_CONNECTED) {
        return;
    }

    bool currentMspConfirmValue = getCurrentMspConfirm();
    receiveMspData(otaPktPtr->msp_ul.packageIndex, otaPktPtr->msp_ul.payload);
    if (currentMspConfirmValue != getCurrentMspConfirm()) {
        nextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
    }
    if (hasFinishedMspData()) {
        if (mspBuffer[ELRS_MSP_COMMAND_INDEX] == ELRS_MSP_SET_RX_CONFIG && mspBuffer[ELRS_MSP_COMMAND_INDEX + 1] == ELRS_MSP_MODEL_ID) { //mspReceiverComplete
            if (rxExpressLrsSpiConfig()->modelId != mspBuffer[9]) { //UpdateModelMatch
                rxExpressLrsSpiConfigMutable()->modelId = mspBuffer[9];
                receiver.configChanged = true;
                receiver.connectionState = ELRS_DISCONNECT_PENDING;
            }
        } else if (connectionHasModelMatch) {
            processMspPacket(mspBuffer);
        }

        mspReceiverUnlock();
    }
#endif
}

static bool processRFSyncPacket(volatile elrsOtaPacket_t const * const otaPktPtr, const uint32_t timeStampMs)
{
    // Verify the first two of three bytes of the binding ID, which should always match
    if (otaPktPtr->sync.UID3 != receiver.UID[3] || otaPktPtr->sync.UID4 != receiver.UID[4]) {
        return false;
    }

    // The third byte will be XORed with inverse of the ModelId if ModelMatch is on
    // Only require the first 18 bits of the UID to match to establish a connection
    // but the last 6 bits must modelmatch before sending any data to the FC
    if ((otaPktPtr->sync.UID5 & ~ELRS_MODELMATCH_MASK) != (receiver.UID[5] & ~ELRS_MODELMATCH_MASK)) {
        return false;
    }

    receiver.lastSyncPacketMs = timeStampMs;

    // Will change the packet air rate in loop() if this changes
    receiver.nextRateIndex = domainIsTeam24() ? airRateIndexToIndex24(otaPktPtr->sync.rateIndex, receiver.rateIndex) : airRateIndexToIndex900(otaPktPtr->sync.rateIndex, receiver.rateIndex);
    uint8_t switchEncMode = otaPktPtr->sync.switchEncMode;

    // Update switch mode encoding immediately
    if (switchEncMode != rxExpressLrsSpiConfig()->switchMode) {
        rxExpressLrsSpiConfigMutable()->switchMode = switchEncMode;
        receiver.configChanged = true;
    }

    // Update TLM ratio
    uint8_t tlmRateIn = otaPktPtr->sync.newTlmRatio + TLM_RATIO_NO_TLM;
    uint8_t tlmDenom = tlmRatioEnumToValue(tlmRateIn);
    if (currTlmDenom != tlmDenom) {
        currTlmDenom = tlmDenom;
        telemBurstValid = false;
    }

    // modelId = 0xff indicates modelMatch is disabled, the XOR does nothing in that case
    uint8_t modelXor = (~rxExpressLrsSpiConfig()->modelId) & ELRS_MODELMATCH_MASK;
    bool modelMatched = otaPktPtr->sync.UID5 == (receiver.UID[5] ^ modelXor);

    if (receiver.connectionState == ELRS_DISCONNECTED || receiver.nonceRX != otaPktPtr->sync.nonce || fhssGetCurrIndex() != otaPktPtr->sync.fhssIndex || connectionHasModelMatch != modelMatched) {
        fhssSetCurrIndex(otaPktPtr->sync.fhssIndex);
        receiver.nonceRX = otaPktPtr->sync.nonce;

        tentativeConnection(timeStampMs);
        connectionHasModelMatch = modelMatched;

        if (!expressLrsTimerIsRunning()) {
            return true;
        }
    }

    return false;
}

static bool validatePacketCrcStd(volatile elrsOtaPacket_t * const otaPktPtr)
{
    uint16_t const inCRC = ((uint16_t) otaPktPtr->crcHigh << 8) + otaPktPtr->crcLow;
    // For smHybrid the CRC only has the packet type in byte 0
    // For smWide the FHSS slot is added to the CRC in byte 0 on PACKET_TYPE_RCDATAs
    if (otaPktPtr->type == ELRS_RC_DATA_PACKET && rxExpressLrsSpiConfig()->switchMode == SM_WIDE) {
        otaPktPtr->crcHigh = (receiver.nonceRX % receiver.modParams->fhssHopInterval) + 1;
    } else {
        otaPktPtr->crcHigh = 0;
    }
    uint16_t const calculatedCRC = calcCrc14((uint8_t *) otaPktPtr, 7, crcInitializer);
    return inCRC == calculatedCRC;
}

rx_spi_received_e processRFPacket(volatile uint8_t *payload, uint32_t timeStampUs)
{
    volatile elrsOtaPacket_t * const otaPktPtr = (elrsOtaPacket_t * const) dmaBuffer;

    if (!validatePacketCrcStd(otaPktPtr)) {
        return RX_SPI_RECEIVED_NONE;
    }

    phaseLockEprEvent(EPR_EXTERNAL, timeStampUs + PACKET_HANDLING_TO_TOCK_ISR_DELAY_US);

    bool shouldStartTimer = false;
    uint32_t timeStampMs = millis();

    receiver.lastValidPacketMs = timeStampMs;

    switch(otaPktPtr->type) {
    case ELRS_RC_DATA_PACKET:
        // Must be fully connected to process RC packets, prevents processing RC
        // during sync, where packets can be received before connection
        if (receiver.connectionState == ELRS_CONNECTED && connectionHasModelMatch) {
            if (rxExpressLrsSpiConfig()->switchMode == SM_WIDE) {
                wideSwitchIndex = hybridWideNonceToSwitchIndex(receiver.nonceRX);
                if ((currTlmDenom < 8) || wideSwitchIndex == 7) {
                    confirmCurrentTelemetryPayload((otaPktPtr->rc.switches & 0x40) >> 6);
                }
            } else {
                confirmCurrentTelemetryPayload(otaPktPtr->rc.switches & (1 << 6));
            }
            memcpy((uint8_t *) payload, (uint8_t *) dmaBuffer, ELRS_RX_TX_BUFF_SIZE); // stick data handling is done in expressLrsSetRcDataFromPayload
        }
        break;
    case ELRS_MSP_DATA_PACKET:
        processRFMspPacket(otaPktPtr);
        break;
    case ELRS_TLM_PACKET:
        //not implemented
        break;
    case ELRS_SYNC_PACKET:
        shouldStartTimer = processRFSyncPacket(otaPktPtr, timeStampMs) && !receiver.inBindingMode;
        break;
    default:
        return RX_SPI_RECEIVED_NONE;
    }

    // Store the LQ/RSSI/Antenna
    receiver.getRfLinkInfo(&receiver.rssi, &receiver.snr);
    meanAccumulatorAdd(&snrFilter, receiver.snr);
    // Received a packet, that's the definition of LQ
    lqIncrease();

    // Extend sync duration since we've received a packet at this rate
    // but do not extend it indefinitely
    receiver.rfModeCycleMultiplier = ELRS_MODE_CYCLE_MULTIPLIER_SLOW; //RFModeCycleMultiplierSlow

    if (shouldStartTimer) {
        expressLrsTimerResume();
    }

    return RX_SPI_RECEIVED_DATA;
}

static void updateTelemetryBurst(void)
{
    if (telemBurstValid) {
        return;
    }
    telemBurstValid = true;

    uint32_t hz = rateEnumToHz(receiver.modParams->enumRate);
    telemetryBurstMax = TELEM_MIN_LINK_INTERVAL * hz / currTlmDenom / 1000U;

    // Reserve one slot for LINK telemetry
    if (telemetryBurstMax > 1) {
        --telemetryBurstMax;
    } else {
        telemetryBurstMax = 1;
    }

    // Notify the sender to adjust its expected throughput
    updateTelemetryRate(hz, currTlmDenom, telemetryBurstMax);
}

/* If not connected will rotate through the RF modes looking for sync
 * and blink LED
 */
static void cycleRfMode(const uint32_t timeStampMs)
{
    if (receiver.connectionState == ELRS_CONNECTED || receiver.inBindingMode) {
        return;
    }
    // Actually cycle the RF mode if not LOCK_ON_FIRST_CONNECTION
    if (receiver.lockRFmode == false && (timeStampMs - receiver.rfModeCycledAtMs) > (receiver.cycleIntervalMs * receiver.rfModeCycleMultiplier)) {
        receiver.rfModeCycledAtMs = timeStampMs;
        receiver.lastSyncPacketMs = timeStampMs;           // reset this variable
        receiver.rateIndex = (receiver.rateIndex + 1) % ELRS_RATE_MAX;
        setRfLinkRate(receiver.rateIndex); // switch between rates
        receiver.statsUpdatedAtMs = timeStampMs;
        lqReset();
        receiver.startReceiving();

        // Switch to FAST_SYNC if not already in it (won't be if was just connected)
        receiver.rfModeCycleMultiplier = 1;
    } // if time to switch RF mode
}

#ifdef USE_RX_SX1280
static inline void configureReceiverForSX1280(void)
{
    receiver.init = (elrsRxInitFnPtr) sx1280Init;
    receiver.config = (elrsRxConfigFnPtr) sx1280Config;
    receiver.startReceiving = (elrsRxStartReceivingFnPtr) sx1280StartReceiving;
    receiver.rxISR = (elrsRxISRFnPtr) sx1280ISR;
    receiver.rxHandleFromTock = (elrsRxHandleFromTockFnPtr) sx1280HandleFromTock;
    receiver.rxHandleFromTick = (elrsRxBusyTimeoutFnPtr) sx1280HandleFromTick;
    receiver.getRfLinkInfo = (elrsRxgetRfLinkInfoFnPtr) sx1280GetLastPacketStats;
    receiver.handleFreqCorrection = (elrsRxHandleFreqCorrectionFnPtr) sx1280AdjustFrequency;
}
#endif

#ifdef USE_RX_SX127X
static inline void configureReceiverForSX127x(void)
{
    receiver.init = (elrsRxInitFnPtr) sx127xInit;
    receiver.config = (elrsRxConfigFnPtr) sx127xConfig;
    receiver.startReceiving = (elrsRxStartReceivingFnPtr) sx127xStartReceiving;
    receiver.rxISR = (elrsRxISRFnPtr) sx127xISR;
    receiver.getRfLinkInfo = (elrsRxgetRfLinkInfoFnPtr) sx127xGetLastPacketStats;
    receiver.handleFreqCorrection = (elrsRxHandleFreqCorrectionFnPtr) sx127xAdjustFrequency;
}
#endif

//setup
bool expressLrsSpiInit(const struct rxSpiConfig_s *rxConfig, struct rxRuntimeState_s *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    if (!rxSpiExtiConfigured()) {
        return false;
    }

    rxSpiCommonIOInit(rxConfig);
    
    rxRuntimeState->channelCount = ELRS_MAX_CHANNELS;
    
    extiConfig->ioConfig = IOCFG_IPD;
    extiConfig->trigger = BETAFLIGHT_EXTI_TRIGGER_RISING;

    if (rxExpressLrsSpiConfig()->resetIoTag) {
        receiver.resetPin = IOGetByTag(rxExpressLrsSpiConfig()->resetIoTag);
    } else {
        receiver.resetPin = IO_NONE;
    }

    if (rxExpressLrsSpiConfig()->busyIoTag) {
        receiver.busyPin = IOGetByTag(rxExpressLrsSpiConfig()->busyIoTag);
    } else {
        receiver.busyPin = IO_NONE;
    }

    switch (rxExpressLrsSpiConfig()->domain) {
#ifdef USE_RX_SX127X
    case AU433:
    case AU915:
    case EU433:
    case EU868:
    case IN866:
    case FCC915:
        configureReceiverForSX127x();
        bindingRateIndex = ELRS_BINDING_RATE_900;
        break;
#endif
#ifdef USE_RX_SX1280
    case ISM2400:
        FALLTHROUGH;
    case CE2400:
        configureReceiverForSX1280();
        bindingRateIndex = ELRS_BINDING_RATE_24;
        break;
#endif
    default:
        return false;
    }

    if (!receiver.init(receiver.resetPin, receiver.busyPin)) {
        return false;
    }

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

    if (linkQualitySource == LQ_SOURCE_NONE) {
        linkQualitySource = LQ_SOURCE_RX_PROTOCOL_CRSF;
    }

    if (rxExpressLrsSpiConfig()->UID[0] || rxExpressLrsSpiConfig()->UID[1]
        || rxExpressLrsSpiConfig()->UID[2] || rxExpressLrsSpiConfig()->UID[3]
        || rxExpressLrsSpiConfig()->UID[4] || rxExpressLrsSpiConfig()->UID[5]) {
        receiver.inBindingMode = false;
        receiver.UID = rxExpressLrsSpiConfig()->UID;
        crcInitializer = (receiver.UID[4] << 8) | receiver.UID[5];
        crcInitializer ^= ELRS_OTA_VERSION_ID;
    } else {
        receiver.inBindingMode = true;
        receiver.UID = BindingUID;
        crcInitializer = 0;
    }

    expressLrsPhaseLockReset();

    expressLrsInitialiseTimer(RX_EXPRESSLRS_TIMER_INSTANCE, &receiver.timerUpdateCb);
    expressLrsTimerStop();

    generateCrc14Table();
    initializeReceiver();

    initTelemetry();
#ifdef USE_MSP_OVER_TELEMETRY
    setMspDataToReceive(ELRS_MSP_BUFFER_SIZE, mspBuffer);
#endif

    // Timer IRQs must only be enabled after the receiver is configured otherwise race conditions occur.
    expressLrsTimerEnableIRQs();

    return true;
}

static void handleConnectionStateUpdate(const uint32_t timeStampMs)
{
    if ((receiver.connectionState != ELRS_DISCONNECTED) && (receiver.modParams->index != receiver.nextRateIndex)) {  // forced change
        lostConnection();
        receiver.lastSyncPacketMs = timeStampMs;         // reset this variable to stop rf mode switching and add extra time
        receiver.rfModeCycledAtMs = timeStampMs;         // reset this variable to stop rf mode switching and add extra time
        setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
#ifdef USE_RX_RSSI_DBM
        setRssiDbmDirect(-130, RSSI_SOURCE_RX_PROTOCOL);
#endif
#ifdef USE_RX_RSNR
        setRsnrDirect(-30);
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
        setLinkQualityDirect(0);
#endif
    }

    if (receiver.connectionState == ELRS_TENTATIVE && ((timeStampMs - receiver.lastSyncPacketMs) > receiver.rfPerfParams->rxLockTimeoutMs)) {
        lostConnection();
        receiver.rfModeCycledAtMs = timeStampMs;
        receiver.lastSyncPacketMs = timeStampMs;
    }

    cycleRfMode(timeStampMs);

    // Note that will updated from ISR and could thus be ahead of the timestamp
    uint32_t localLastValidPacket = receiver.lastValidPacketMs;

    // Determine the time in ms since the last valid packet was received. As this will be received under interrupt control it may be
    // after the timeStampMs value was taken in which case treat as if it arrived at the same time to avoid a timeout
    int32_t lastPacketDeltaMs = cmp32(timeStampMs, localLastValidPacket);

    if (lastPacketDeltaMs < 0) {
        lastPacketDeltaMs = 0;
    }

    if ((receiver.connectionState == ELRS_DISCONNECT_PENDING) || // check if we lost conn.
        ((receiver.connectionState == ELRS_CONNECTED) && ((uint32_t)lastPacketDeltaMs > receiver.rfPerfParams->disconnectTimeoutMs))) {
        lostConnection();
    }

    if ((receiver.connectionState == ELRS_TENTATIVE) && (abs(pl.offsetDeltaUs) <= 10) && (pl.offsetUs < 100) && (lqGet() > minLqForChaos())) {
        gotConnection(timeStampMs); // detects when we are connected
    }

    if ((receiver.timerState == ELRS_TIM_TENTATIVE) && ((timeStampMs - receiver.gotConnectionMs) > ELRS_CONSIDER_CONNECTION_GOOD_MS) && (abs(pl.offsetDeltaUs) <= 5)) {
        receiver.timerState = ELRS_TIM_LOCKED;
    }

    if ((receiver.connectionState == ELRS_CONNECTED) && (abs(pl.offsetDeltaUs) > 10) && (pl.offsetUs >= 100) && (lqGet() <= minLqForChaos())) {
        lostConnection(); // SPI: resync when we're in chaos territory
    }
}

static void handleConfigUpdate(const uint32_t timeStampMs)
{
    if ((timeStampMs - receiver.configCheckedAtMs) > ELRS_CONFIG_CHECK_MS) {
        receiver.configCheckedAtMs = timeStampMs;
        if (receiver.configChanged) {
            saveConfigAndNotify();
            receiver.initializeReceiverPending = true;
            receiver.configChanged = false;
        }
    }
}

static void handleLinkStatsUpdate(const uint32_t timeStampMs)
{
    if ((timeStampMs - receiver.statsUpdatedAtMs) > ELRS_LINK_STATS_CHECK_MS) {

        receiver.statsUpdatedAtMs = timeStampMs;

        if (receiver.connectionState == ELRS_CONNECTED) {
            receiver.rssiFiltered = simpleLPFilterUpdate(&rssiFilter, receiver.rssi);
            uint16_t rssiScaled = scaleRange(constrain(receiver.rssiFiltered, receiver.rfPerfParams->sensitivity, -50), receiver.rfPerfParams->sensitivity, -50, 0, 1023);
            setRssi(rssiScaled, RSSI_SOURCE_RX_PROTOCOL);
#ifdef USE_RX_RSSI_DBM
            setRssiDbm(receiver.rssiFiltered, RSSI_SOURCE_RX_PROTOCOL);
#endif
#ifdef USE_RX_RSNR
            receiver.rsnrFiltered = simpleLPFilterUpdate(&rsnrFilter, receiver.snr/4);
            setRsnr(receiver.rsnrFiltered);
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
            setLinkQualityDirect(receiver.uplinkLQ);
#endif
#ifdef USE_RX_LINK_UPLINK_POWER
            rxSetUplinkTxPwrMw(txPowerIndexToValue(txPower));
#endif
        } else {
            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
#ifdef USE_RX_RSSI_DBM
            setRssiDbmDirect(-130, RSSI_SOURCE_RX_PROTOCOL);
#endif
#ifdef USE_RX_RSNR
            setRsnrDirect(-30);
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
            setLinkQualityDirect(0);
#endif
        }
    }
}

void expressLrsHandleTelemetryUpdate(void)
{
    if (receiver.connectionState != ELRS_CONNECTED || (receiver.modParams->tlmInterval == TLM_RATIO_NO_TLM)) {
        return;
    }

    uint8_t *nextPayload = 0;
    uint8_t nextPlayloadSize = 0;
    if (!isTelemetrySenderActive() && getNextTelemetryPayload(&nextPlayloadSize, &nextPayload)) {
        setTelemetryDataToTransmit(nextPlayloadSize, nextPayload);
    }
    updateTelemetryBurst();
}

void expressLrsSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    if (rcData && payload) {
        volatile elrsOtaPacket_t * const otaPktPtr = (elrsOtaPacket_t * const) payload;
        rxExpressLrsSpiConfig()->switchMode == SM_WIDE ? unpackChannelDataHybridWide(rcData, otaPktPtr) : unpackChannelDataHybridSwitch8(rcData, otaPktPtr);
    }
}

static void enterBindingMode(void)
{
    // Set UID to special binding values
    receiver.UID = BindingUID;
    crcInitializer = 0;
    receiver.inBindingMode = true;
    
    setRfLinkRate(bindingRateIndex);
    receiver.startReceiving();
}

void expressLrsDoTelem(void)
{
    expressLrsHandleTelemetryUpdate();
    expressLrsSendTelemResp();
    
    if (!domainIsTeam24() && !receiver.didFhss && !expressLrsTelemRespReq() && lqPeriodIsSet()) {
        // TODO No need to handle this on SX1280, but will on SX127x
        // TODO this needs to be DMA aswell, SX127x unlikely to work right now
        receiver.handleFreqCorrection(receiver.freqOffset, receiver.currentFreq); //corrects for RX freq offset
    }
}

rx_spi_received_e expressLrsDataReceived(uint8_t *payloadBuffer)
{
    payload = payloadBuffer;

    rx_spi_received_e rfPacketReturnStatus = RX_SPI_RECEIVED_NONE;

    if (!receiver.started && (systemState & SYSTEM_STATE_READY)) {
        receiver.started = true;
        receiver.startReceiving(); // delay receiving after initialization to ensure a clean connect
    }

    if (receiver.initializeReceiverPending) {
        initializeReceiver();
        receiver.initializeReceiverPending = false;
    }

    if (rxSpiCheckBindRequested(true)) {
        enterBindingMode();
    }

    const uint32_t timeStampMs = millis();
    handleConnectionStateUpdate(timeStampMs);
    handleConfigUpdate(timeStampMs);
    handleLinkStatsUpdate(timeStampMs);

    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 0, lostConnectionCounter);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 1, receiver.rssiFiltered);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 2, receiver.snr / 4);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 3, receiver.uplinkLQ);

    receiver.inBindingMode ? rxSpiLedBlinkBind() : rxSpiLedBlinkRxLoss(rfPacketStatus);

    if (rfPacketStatus != RX_SPI_RECEIVED_NONE) {
        // A packet has been received since last time
        rfPacketReturnStatus = rfPacketStatus;
        rfPacketStatus = RX_SPI_RECEIVED_NONE;
    }
    return rfPacketReturnStatus;
}

void expressLrsStop(void)
{
    if (receiver.started) {
        lostConnection();
    }
}

void expressLrsISR(bool runAlways)
{
    if (runAlways || !expressLrsTimerIsRunning()) {
        receiver.rxISR();
    }
}
#endif /* USE_RX_EXPRESSLRS */
