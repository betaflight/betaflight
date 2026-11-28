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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#ifdef USE_SERIALRX_MAVLINK

#include "common/utils.h"
#include "common/maths.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/flight_plan_nav.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/mavlink.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "drivers/time.h"
#include "drivers/nvic.h"

#include "build/debug.h"

#include "telemetry/mavlink_lib.h"

#ifdef USE_TELEMETRY_MAVLINK
#include "telemetry/mavlink.h"
#endif

#define MAVLINK_CHANNEL_COUNT 18
#define MAVLINK_BAUD_RATE_INDEX BAUD_460800
#define MAVLINK_RC_SOURCE_TIMEOUT_US 500000
#define MAVLINK_GCS_NEUTRAL_INTERVAL_US 20000
#define MAVLINK_MANUAL_CONTROL_AUX_COUNT 6
#define MAVLINK_MANUAL_CONTROL_AUX1_BIT 2

typedef enum {
    MAVLINK_RC_SOURCE_NONE,
    MAVLINK_RC_SOURCE_OVERRIDE,
    MAVLINK_RC_SOURCE_MANUAL,
    MAVLINK_RC_SOURCE_GCS_NEUTRAL,
} mavlinkRcSource_e;

typedef struct mavlinkRcInput_s {
    uint16_t channels[MAVLINK_CHANNEL_COUNT];
    volatile timeUs_t frameTimeUs;
    volatile bool newFrame;
    volatile uint32_t frameCount;
    uint32_t expiredFrameCount;
} mavlinkRcInput_t;

static uint16_t mavlinkChannelData[MAVLINK_CHANNEL_COUNT];
static mavlinkRcInput_t overrideInput;
static mavlinkRcInput_t manualInput;
static timeUs_t gcsNeutralFrameTimeUs;
STATIC_UNIT_TESTED const rxConfig_t *mavlinkRxConfig;

static serialPort_t *serialPort = NULL;

static mavlink_message_t mavRecvMsg;
static mavlink_status_t mavRecvStatus;
static volatile uint8_t txbuff_free = 100;  // tx buffer space in %, start with empty buffer
static volatile bool txbuff_valid = false;
static mavlinkRxQueue_t mavlinkRxQueue;

static void mavlinkAddMessageToQueue(void)
{
    const uint8_t head = mavlinkRxQueue.head;
    const uint8_t next = (head + 1) % MAVLINK_RX_QUEUE_SIZE;
    if (next != __atomic_load_n(&mavlinkRxQueue.tail, __ATOMIC_ACQUIRE)) {
        mavlinkRxQueue.msgs[head] = mavRecvMsg;
        __atomic_store_n(&mavlinkRxQueue.head, next, __ATOMIC_RELEASE);
    }
}

bool mavlinkGetNextQueueMessage(mavlink_message_t *msg)
{
    const uint8_t tail = mavlinkRxQueue.tail;
    if (tail == __atomic_load_n(&mavlinkRxQueue.head, __ATOMIC_ACQUIRE)) {
        return false;
    }
    *msg = mavlinkRxQueue.msgs[tail];
    __atomic_store_n(&mavlinkRxQueue.tail, (tail + 1) % MAVLINK_RX_QUEUE_SIZE, __ATOMIC_RELEASE);
    return true;
}

void mavlinkRxHandleMessage(const mavlink_rc_channels_override_t *msg)
{
    const uint16_t channels[MAVLINK_CHANNEL_COUNT] = {
        msg->chan1_raw,
        msg->chan2_raw,
        msg->chan3_raw,
        msg->chan4_raw,
        msg->chan5_raw,
        msg->chan6_raw,
        msg->chan7_raw,
        msg->chan8_raw,
        msg->chan9_raw,
        msg->chan10_raw,
        msg->chan11_raw,
        msg->chan12_raw,
        msg->chan13_raw,
        msg->chan14_raw,
        msg->chan15_raw,
        msg->chan16_raw,
        msg->chan17_raw,
        msg->chan18_raw
    };

    // 0, UINT16_MAX and UINT16_MAX - 1 mean ignore or release
    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; i++) {
        if (channels[i] != 0 && channels[i] < UINT16_MAX - 1) {
            overrideInput.channels[i] = channels[i];
        }
    }

    overrideInput.frameTimeUs = micros();
    overrideInput.frameCount++;
    overrideInput.newFrame = true;
}

#ifdef USE_TELEMETRY_MAVLINK
static uint16_t mavlinkStickToUs(int16_t value, uint16_t neutral)
{
    if (value == INT16_MAX) {
        return neutral;
    }
    return neutral + constrain(value, -1000, 1000) / 2;
}
#endif

static void mavlinkSetLogicalChannel(uint16_t *channels, uint8_t logical, uint16_t value)
{
    const uint8_t raw = logical < RX_MAPPABLE_CHANNEL_COUNT ? mavlinkRxConfig->rcmap[logical] : logical;
    channels[raw] = value;
}

void mavlinkRxHandleManualControl(const mavlink_message_t *msg)
{
#ifdef USE_TELEMETRY_MAVLINK
    if (!mavlinkIsGcsSysid(msg->sysid)) {
        return;
    }
    mavlink_manual_control_t manual;
    mavlink_msg_manual_control_decode(msg, &manual);
    if (manual.target != 0 && manual.target != MAVLINK_SYSTEM_ID) {
        return;
    }
    if (!mavlinkRxConfig) {
        return;
    }

    uint16_t channels[MAVLINK_CHANNEL_COUNT];
    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; i++) {
        channels[i] = PWM_RANGE_MIN;
    }
    const uint16_t midrc = mavlinkRxConfig->midrc;
    mavlinkSetLogicalChannel(channels, ROLL, mavlinkStickToUs(manual.y, midrc));
    mavlinkSetLogicalChannel(channels, PITCH, mavlinkStickToUs(manual.x, midrc));
    mavlinkSetLogicalChannel(channels, YAW, mavlinkStickToUs(manual.r, midrc));
    const uint16_t throttle = manual.z == INT16_MAX ? PWM_RANGE_MIN : PWM_RANGE_MIN + constrain(manual.z, 0, 1000);
    mavlinkSetLogicalChannel(channels, THROTTLE, throttle);

    const int16_t aux[MAVLINK_MANUAL_CONTROL_AUX_COUNT] = { manual.aux1, manual.aux2, manual.aux3, manual.aux4, manual.aux5, manual.aux6 };
    for (int i = 0; i < MAVLINK_MANUAL_CONTROL_AUX_COUNT; i++) {
        if (manual.enabled_extensions & (1 << (MAVLINK_MANUAL_CONTROL_AUX1_BIT + i))) {
            mavlinkSetLogicalChannel(channels, AUX1 + i, mavlinkStickToUs(aux[i], midrc));
        }
    }

    memcpy(manualInput.channels, channels, sizeof(channels));
    manualInput.frameTimeUs = micros();
    manualInput.frameCount++;
    manualInput.newFrame = true;
#else
    UNUSED(msg);
#endif
}

#if ENABLE_REMOTE_ARM && defined(USE_TELEMETRY_MAVLINK)
static bool mavlinkGcsNeutralAllowed(void)
{
    if (!mavlinkGcsIsFresh(millis())) {
        return false;
    }
    return !ARMING_FLAG(ARMED) || !wasThrottleRaised() || FLIGHT_MODE(AUTOPILOT_MODE | POS_HOLD_MODE | GPS_RESCUE_MODE)
        || flightPlanNavIsTakingOff();
}
#endif

static bool mavlinkRcInputIsFresh(mavlinkRcInput_t *input, timeUs_t currentTimeUs)
{
    const uint32_t frameCount = input->frameCount;
    if (frameCount == input->expiredFrameCount) {
        return false;
    }
    if (cmpTimeUs(currentTimeUs, input->frameTimeUs) > MAVLINK_RC_SOURCE_TIMEOUT_US) {
        input->expiredFrameCount = frameCount;
        return false;
    }
    return true;
}

static mavlinkRcSource_e mavlinkSelectRcSource(timeUs_t currentTimeUs)
{
    const bool overrideFresh = mavlinkRcInputIsFresh(&overrideInput, currentTimeUs);
    const bool manualFresh = mavlinkRcInputIsFresh(&manualInput, currentTimeUs);
    if (overrideFresh) {
        return MAVLINK_RC_SOURCE_OVERRIDE;
    }
    if (manualFresh) {
        return MAVLINK_RC_SOURCE_MANUAL;
    }
#if ENABLE_REMOTE_ARM && defined(USE_TELEMETRY_MAVLINK)
    if (mavlinkGcsNeutralAllowed()) {
        return MAVLINK_RC_SOURCE_GCS_NEUTRAL;
    }
#endif
    return MAVLINK_RC_SOURCE_NONE;
}

static uint8_t mavlinkDeliverRcInput(mavlinkRcInput_t *input, rxRuntimeState_t *rxRuntimeState)
{
    if (!input->newFrame) {
        return RX_FRAME_PENDING;
    }
    input->newFrame = false;
    memcpy(mavlinkChannelData, input->channels, sizeof(mavlinkChannelData));
    rxRuntimeState->lastRcFrameTimeUs = input->frameTimeUs;
    return RX_FRAME_COMPLETE;
}

static uint8_t mavlinkDeliverGcsNeutral(rxRuntimeState_t *rxRuntimeState, timeUs_t currentTimeUs)
{
    if (gcsNeutralFrameTimeUs != 0 && currentTimeUs - gcsNeutralFrameTimeUs < MAVLINK_GCS_NEUTRAL_INTERVAL_US) {
        return RX_FRAME_PENDING;
    }
    gcsNeutralFrameTimeUs = currentTimeUs;
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < MAVLINK_CHANNEL_COUNT; i++) {
            mavlinkChannelData[i] = PWM_RANGE_MIN;
        }
    }
    mavlinkSetLogicalChannel(mavlinkChannelData, THROTTLE, PWM_RANGE_MIN);
    mavlinkSetLogicalChannel(mavlinkChannelData, ROLL, mavlinkRxConfig->midrc);
    mavlinkSetLogicalChannel(mavlinkChannelData, PITCH, mavlinkRxConfig->midrc);
    mavlinkSetLogicalChannel(mavlinkChannelData, YAW, mavlinkRxConfig->midrc);
    rxRuntimeState->lastRcFrameTimeUs = currentTimeUs;
    return RX_FRAME_COMPLETE;
}

static uint8_t mavlinkFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    const timeUs_t currentTimeUs = micros();
    switch (mavlinkSelectRcSource(currentTimeUs)) {
    case MAVLINK_RC_SOURCE_OVERRIDE:
        return mavlinkDeliverRcInput(&overrideInput, rxRuntimeState);
    case MAVLINK_RC_SOURCE_MANUAL:
        return mavlinkDeliverRcInput(&manualInput, rxRuntimeState);
    case MAVLINK_RC_SOURCE_GCS_NEUTRAL:
        return mavlinkDeliverGcsNeutral(rxRuntimeState, currentTimeUs);
    default:
        return RX_FRAME_PENDING;
    }
}

static float mavlinkReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    return mavlinkChannelData[channel];
}

static bool handleIncoming_RC_CHANNELS_OVERRIDE(void)
{
    mavlink_rc_channels_override_t msg;
    mavlink_msg_rc_channels_override_decode(&mavRecvMsg, &msg);
    mavlinkRxHandleMessage(&msg);
    return true;
}

static void mavlinkParseRxStats(const mavlink_radio_status_t *msg)
{
    const int16_t rssiDbm = -msg->remrssi;
    const uint16_t rssiPercentScaled = scaleRange(rssiDbm, RSSI_DBM_MIN, RSSI_DBM_MAX, 0, RSSI_MAX_VALUE);
    setRssi(rssiPercentScaled, RSSI_SOURCE_RX_PROTOCOL_MAVLINK);

#ifdef USE_RX_RSSI_DBM
    setRssiDbm(rssiDbm, RSSI_SOURCE_RX_PROTOCOL_MAVLINK);
#endif

#ifdef USE_RX_RSNR
    setRsnr(msg->noise);
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
    if (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_MAVLINK) {
        const uint16_t linkqualityValue = scaleRange(msg->rssi, 0, 255, 0, 100);
        setLinkQualityDirect(linkqualityValue);
    }
#endif
}

// Get RADIO_STATUS data
static void handleIncoming_RADIO_STATUS(void)
{
    mavlink_radio_status_t msg;
    mavlink_msg_radio_status_decode(&mavRecvMsg, &msg);
    txbuff_valid = true;
    txbuff_free = msg.txbuf;

    mavlinkParseRxStats(&msg);

    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 1, txbuff_free);  //!< TX Buffer Free [unit:%]
}

STATIC_UNIT_TESTED void mavlinkDataReceive(uint16_t c, void *data)
{
    UNUSED(data);
    uint8_t result = mavlink_parse_char(0, c, &mavRecvMsg, &mavRecvStatus);
    if (result == MAVLINK_FRAMING_OK) {
        switch (mavRecvMsg.msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            // Handle RC CHANNELS data immediately to get maximum RC rate
            handleIncoming_RC_CHANNELS_OVERRIDE();
            break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            mavlinkRxHandleManualControl(&mavRecvMsg);
            break;
        case MAVLINK_MSG_ID_RADIO_STATUS:
            handleIncoming_RADIO_STATUS();
            break;
        default:
            // Add another message into the queue, to handle it in the mavlink telemetry task later
            mavlinkAddMessageToQueue();
        }
    }
}

bool mavlinkRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    mavlinkRxConfig = rxConfig;
    txbuff_valid = false;
    txbuff_free = 100;
    memset(&overrideInput, 0, sizeof(overrideInput));
    memset(&manualInput, 0, sizeof(manualInput));
    gcsNeutralFrameTimeUs = 0;
    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; ++i) {
        mavlinkChannelData[i] = rxConfig->midrc;
        overrideInput.channels[i] = rxConfig->midrc;
    }

    rxRuntimeState->channelData = mavlinkChannelData;
    rxRuntimeState->channelCount = MAVLINK_CHANNEL_COUNT;
    rxRuntimeState->rcReadRawFn = mavlinkReadRawRC;
    rxRuntimeState->rcFrameStatusFn = mavlinkFrameStatus;

    const serialPortIdentifier_e port = rxConfig->rx_uart;
    if (port == SERIAL_PORT_NONE) {
        return false;
    }

    // Share the rate of a MAVLink telemetry provider on this same port, if one
    // is configured; otherwise the port is RX-only and takes the ELRS default.
    baudRate_e baudRateIndex = BAUD_AUTO;
#ifdef USE_TELEMETRY_MAVLINK
    baudRateIndex = telemetryProviderBaud(TELEMETRY_PROTOCOL_MAVLINK, port);
#endif
    if (baudRateIndex == BAUD_AUTO) {
        // default rate for ELRS TX module
        baudRateIndex = MAVLINK_BAUD_RATE_INDEX;
    }

    serialPort = openSerialPort(port,
        FUNCTION_RX_SERIAL,
        mavlinkDataReceive,
        rxRuntimeState,
        baudRates[baudRateIndex],
        MODE_RXTX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0)
    );

#ifdef USE_TELEMETRY_MAVLINK
    telemetrySharedPort = serialPort;
#endif

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL_MAVLINK;
    }
#ifdef USE_RX_LINK_QUALITY_INFO
    if (linkQualitySource == LQ_SOURCE_NONE) {
        linkQualitySource = LQ_SOURCE_RX_PROTOCOL_MAVLINK;
    }
#endif

    return serialPort != NULL;
}

#ifdef USE_TELEMETRY_MAVLINK
bool isValidMavlinkTxBuffer (void) {
    return txbuff_valid;
}

bool shouldSendMavlinkTelemetry(void) {
    const uint8_t mavlink_min_txbuff =  telemetryConfig()->mavlink_min_txbuff;
    bool shouldSendTelemetry = false;

    if (txbuff_valid) {
        shouldSendTelemetry = txbuff_free > mavlink_min_txbuff;
    }
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 0, shouldSendTelemetry ? 1 : 0);  //!< Telemetry Due

    return shouldSendTelemetry;
}
#endif

#endif

