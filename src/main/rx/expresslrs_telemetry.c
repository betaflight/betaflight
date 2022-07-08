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
 */

#include <string.h>
#include "platform.h"

#ifdef USE_RX_EXPRESSLRS

#include "config/feature.h"
#include "fc/runtime_config.h"

#include "msp/msp_protocol.h"

#include "rx/crsf_protocol.h"
#include "rx/expresslrs_telemetry.h"

#include "telemetry/crsf.h"
#include "telemetry/telemetry.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

static uint8_t tlmBuffer[CRSF_FRAME_SIZE_MAX];

typedef enum {
    CRSF_FRAME_GPS_INDEX = 0,
    CRSF_FRAME_BATTERY_SENSOR_INDEX,
    CRSF_FRAME_ATTITUDE_INDEX,
    CRSF_FRAME_FLIGHT_MODE_INDEX,
    CRSF_FRAME_PAYLOAD_TYPES_COUNT //should be last
} frameTypeIndex_e;

static crsfFrameType_e payloadTypes[] = {
    CRSF_FRAMETYPE_GPS,
    CRSF_FRAMETYPE_BATTERY_SENSOR,
    CRSF_FRAMETYPE_ATTITUDE,
    CRSF_FRAMETYPE_FLIGHT_MODE
};

STATIC_UNIT_TESTED uint8_t tlmSensors = 0;
STATIC_UNIT_TESTED uint8_t currentPayloadIndex;

static uint8_t *data;
static uint8_t length;
static uint8_t bytesPerCall;
static uint8_t currentOffset;
static uint8_t currentPackage;
static bool waitUntilTelemetryConfirm;
static uint16_t waitCount;
static uint16_t maxWaitCount;
static volatile stubbornSenderState_e senderState;

static void telemetrySenderResetState(void)
{
    data = 0;
    currentOffset = 0;
    currentPackage = 0;
    length = 0;
    waitUntilTelemetryConfirm = true;
    waitCount = 0;
    // 80 corresponds to UpdateTelemetryRate(ANY, 2, 1), which is what the TX uses in boost mode
    maxWaitCount = 80;
    senderState = ELRS_SENDER_IDLE;
    bytesPerCall = 1;
}

/***
 * Queues a message to send, will abort the current message if one is currently being transmitted
 ***/
void setTelemetryDataToTransmit(const uint8_t lengthToTransmit, uint8_t* dataToTransmit, const uint8_t bpc)
{
    if (lengthToTransmit / bpc >= ELRS_TELEMETRY_MAX_PACKAGES) {
        return;
    }

    length = lengthToTransmit;
    data = dataToTransmit;
    currentOffset = 0;
    currentPackage = 1;
    waitCount = 0;
    bytesPerCall = bpc;
    senderState = (senderState == ELRS_SENDER_IDLE) ? ELRS_SENDING : ELRS_RESYNC_THEN_SEND;
}

bool isTelemetrySenderActive(void)
{
    return senderState != ELRS_SENDER_IDLE;
}

void getCurrentTelemetryPayload(uint8_t *packageIndex, uint8_t *count, uint8_t **currentData)
{
    switch (senderState) {
    case ELRS_RESYNC:
    case ELRS_RESYNC_THEN_SEND:
        *packageIndex = ELRS_TELEMETRY_MAX_PACKAGES;
        *count = 0;
        *currentData = 0;
        break;
    case ELRS_SENDING:
        *currentData = data + currentOffset;
        *packageIndex = currentPackage;
        if (bytesPerCall > 1) {
            if (currentOffset + bytesPerCall <= length) {
                *count = bytesPerCall;
            } else {
                *count = length - currentOffset;
            }
        } else {
            *count = 1;
        }
        break;
    default:
        *count = 0;
        *currentData = 0;
        *packageIndex = 0;
    }
}

void confirmCurrentTelemetryPayload(const bool telemetryConfirmValue)
{
    stubbornSenderState_e nextSenderState = senderState;

    switch (senderState) {
    case ELRS_SENDING:
        if (telemetryConfirmValue != waitUntilTelemetryConfirm) {
            waitCount++;
            if (waitCount > maxWaitCount) {
                waitUntilTelemetryConfirm = !telemetryConfirmValue;
                nextSenderState = ELRS_RESYNC;
            }
            break;
        }

        currentOffset += bytesPerCall;
        currentPackage++;
        waitUntilTelemetryConfirm = !waitUntilTelemetryConfirm;
        waitCount = 0;

        if (currentOffset >= length) {
            nextSenderState = ELRS_WAIT_UNTIL_NEXT_CONFIRM;
        }

        break;

    case ELRS_RESYNC:
    case ELRS_RESYNC_THEN_SEND:
    case ELRS_WAIT_UNTIL_NEXT_CONFIRM:
        if (telemetryConfirmValue == waitUntilTelemetryConfirm) {
            nextSenderState = (senderState == ELRS_RESYNC_THEN_SEND) ? ELRS_SENDING : ELRS_SENDER_IDLE;
            waitUntilTelemetryConfirm = !telemetryConfirmValue;
        } else if (senderState == ELRS_WAIT_UNTIL_NEXT_CONFIRM) { // switch to resync if tx does not confirm value fast enough
            waitCount++;
            if (waitCount > maxWaitCount) {
                waitUntilTelemetryConfirm = !telemetryConfirmValue;
                nextSenderState = ELRS_RESYNC;
            }
        }

        break;

    case ELRS_SENDER_IDLE:
        break;
    }

    senderState = nextSenderState;
}

#ifdef USE_MSP_OVER_TELEMETRY
static uint8_t *mspData;
static volatile bool finishedData;
static volatile uint8_t mspLength;
static volatile uint8_t mspBytesPerCall;
static volatile uint8_t mspCurrentOffset;
static volatile uint8_t mspCurrentPackage;
static volatile bool mspConfirm;

STATIC_UNIT_TESTED volatile bool mspReplyPending;
STATIC_UNIT_TESTED volatile bool deviceInfoReplyPending;

void mspReceiverResetState(void) {
    mspData = 0;
    mspBytesPerCall = 1;
    mspCurrentOffset = 0;
    mspCurrentPackage = 0;
    mspLength = 0;
    mspConfirm = false;
    mspReplyPending = false;
    deviceInfoReplyPending = false;
}

bool getCurrentMspConfirm(void)
{
    return mspConfirm;
}

void setMspDataToReceive(const uint8_t maxLength, uint8_t* dataToReceive, const uint8_t bpc)
{
    mspLength = maxLength;
    mspData = dataToReceive;
    mspCurrentPackage = 1;
    mspCurrentOffset = 0;
    finishedData = false;
    mspBytesPerCall = bpc;
}

void receiveMspData(const uint8_t packageIndex, const volatile uint8_t* receiveData)
{
    if  (packageIndex == 0 && mspCurrentPackage > 1) {
        finishedData = true;
        mspConfirm = !mspConfirm;
        return;
    }

    if (packageIndex == ELRS_MSP_MAX_PACKAGES) {
        mspConfirm = !mspConfirm;
        mspCurrentPackage = 1;
        mspCurrentOffset = 0;
        finishedData = false;
        return;
    }

    if (finishedData) {
        return;
    }

    if (packageIndex == mspCurrentPackage) {
        for (uint8_t i = 0; i < mspBytesPerCall; i++) {
            mspData[mspCurrentOffset++] = *(receiveData + i);
        }

        mspCurrentPackage++;
        mspConfirm = !mspConfirm;
        return;
    }

    return;
}

bool hasFinishedMspData(void)
{
    return finishedData;
}

void mspReceiverUnlock(void)
{
    if (finishedData) {
        mspCurrentPackage = 1;
        mspCurrentOffset = 0;
        finishedData = false;
    }
}

static uint8_t mspFrameSize = 0;

static void bufferMspResponse(uint8_t *payload, const uint8_t payloadSize)
{
    mspFrameSize = getCrsfMspFrame(tlmBuffer, payload, payloadSize);
}

void processMspPacket(uint8_t *packet)
{
    switch (packet[2]) {
    case CRSF_FRAMETYPE_DEVICE_PING:
        deviceInfoReplyPending = true;
        break;
    case CRSF_FRAMETYPE_MSP_REQ:
        FALLTHROUGH;
    case CRSF_FRAMETYPE_MSP_WRITE:
        if (bufferCrsfMspFrame(&packet[ELRS_MSP_PACKET_OFFSET], CRSF_FRAME_RX_MSP_FRAME_SIZE)) {
            handleCrsfMspFrameBuffer(&bufferMspResponse);
            mspReplyPending = true;
        }
        break;
    default:
        break;
    }
}
#endif

/*
 * Called when the telemetry ratio or air rate changes, calculate
 * the new threshold for how many times the telemetryConfirmValue
 * can be wrong in a row before giving up and going to RESYNC
 */
void updateTelemetryRate(const uint16_t airRate, const uint8_t tlmRatio, const uint8_t tlmBurst)
{
    // consipicuously unused airRate parameter, the wait count is strictly based on number
    // of packets, not time between the telemetry packets, or a wall clock timeout
    UNUSED(airRate);
    // The expected number of packet periods between telemetry packets
    uint32_t packsBetween = tlmRatio * (1 + tlmBurst) / tlmBurst;
    maxWaitCount = packsBetween * ELRS_TELEMETRY_MAX_MISSED_PACKETS;
}

void initTelemetry(void)
{
    if (!featureIsEnabled(FEATURE_TELEMETRY)) {
        return;
    }

    if (sensors(SENSOR_ACC) && telemetryIsSensorEnabled(SENSOR_PITCH | SENSOR_ROLL | SENSOR_HEADING)) {
        tlmSensors |= BIT(CRSF_FRAME_ATTITUDE_INDEX);
    }
    if ((isBatteryVoltageConfigured() && telemetryIsSensorEnabled(SENSOR_VOLTAGE))
        || (isAmperageConfigured() && telemetryIsSensorEnabled(SENSOR_CURRENT | SENSOR_FUEL))) {
        tlmSensors |= BIT(CRSF_FRAME_BATTERY_SENSOR_INDEX);
    }
    if (telemetryIsSensorEnabled(SENSOR_MODE)) {
        tlmSensors |= BIT(CRSF_FRAME_FLIGHT_MODE_INDEX);
    }
#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS)
       && telemetryIsSensorEnabled(SENSOR_ALTITUDE | SENSOR_LAT_LONG | SENSOR_GROUND_SPEED | SENSOR_HEADING)) {
        tlmSensors |= BIT(CRSF_FRAME_GPS_INDEX);
    }
#endif

    telemetrySenderResetState();
#ifdef USE_MSP_OVER_TELEMETRY
    mspReceiverResetState();
#endif
}

bool getNextTelemetryPayload(uint8_t *nextPayloadSize, uint8_t **payloadData)
{
#ifdef USE_MSP_OVER_TELEMETRY
    if (deviceInfoReplyPending) {
        *nextPayloadSize = getCrsfFrame(tlmBuffer, CRSF_FRAMETYPE_DEVICE_INFO);
        *payloadData = tlmBuffer;
        deviceInfoReplyPending = false;
        return true;
    } else if (mspReplyPending) {
        *nextPayloadSize = mspFrameSize;
        *payloadData = tlmBuffer;
        mspReplyPending = false;
        return true;
    } else
#endif
    if (tlmSensors & BIT(currentPayloadIndex)) {
        *nextPayloadSize = getCrsfFrame(tlmBuffer, payloadTypes[currentPayloadIndex]);
        *payloadData = tlmBuffer;
        currentPayloadIndex = (currentPayloadIndex + 1) % CRSF_FRAME_PAYLOAD_TYPES_COUNT;
        return true;
    } else {
        currentPayloadIndex = (currentPayloadIndex + 1) % CRSF_FRAME_PAYLOAD_TYPES_COUNT;
        *nextPayloadSize = 0;
        *payloadData = 0;
        return false;
    }
}

#endif
