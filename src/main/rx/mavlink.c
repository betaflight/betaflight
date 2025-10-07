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

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/mavlink.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "drivers/time.h"

#include "build/debug.h"

// mavlink library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#define MAVLINK_CHANNEL_COUNT 18
#define MAVLINK_BAUD_RATE_INDEX BAUD_460800
static uint16_t mavlinkChannelData[MAVLINK_CHANNEL_COUNT];
static bool frameReceived;

static serialPort_t *serialPort = NULL;

static mavlink_message_t mavRecvMsg;
static mavlink_status_t mavRecvStatus;
static uint8_t txbuff_free = 100;  // tx buffer space in %, start with empty buffer
static bool txbuff_valid = false;

void mavlinkRxHandleMessage(const mavlink_rc_channels_override_t *msg) {
    const uint16_t *channelsPtr = (uint16_t*)&msg->chan1_raw;
    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; i++) {
        if (channelsPtr[i] != 0 && channelsPtr[i] != UINT16_MAX) {
            mavlinkChannelData[i] = channelsPtr[i];
        }
    }
    frameReceived = true;
}

static uint8_t mavlinkFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    if (frameReceived) {
        frameReceived = false;
        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;
}

static float mavlinkReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    return mavlinkChannelData[channel];
}

static bool handleIncoming_RC_CHANNELS_OVERRIDE(void) {
    mavlink_rc_channels_override_t msg;
    mavlink_msg_rc_channels_override_decode(&mavRecvMsg, &msg);
    mavlinkRxHandleMessage(&msg);
    return true;
}

// Get RADIO_STATUS data
static void handleIncoming_RADIO_STATUS(void)
{
    mavlink_radio_status_t msg;
    mavlink_msg_radio_status_decode(&mavRecvMsg, &msg);
    txbuff_valid = true;
    txbuff_free = msg.txbuf;
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 1, txbuff_free); // Last known TX buffer free space
}

STATIC_UNIT_TESTED void mavlinkDataReceive(uint16_t c, void *data)
{
    rxRuntimeState_t *const rxRuntimeState = (rxRuntimeState_t *const)data;
    uint8_t result = mavlink_parse_char(0, c, &mavRecvMsg, &mavRecvStatus);
    if (result == MAVLINK_FRAMING_OK) {
        switch (mavRecvMsg.msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            handleIncoming_RC_CHANNELS_OVERRIDE();
            rxRuntimeState->lastRcFrameTimeUs = micros();
            break;
        case MAVLINK_MSG_ID_RADIO_STATUS:
            handleIncoming_RADIO_STATUS();
            break;
        }
    }
}

bool mavlinkRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    frameReceived = false;
    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; ++i) {
        mavlinkChannelData[i] = rxConfig->midrc;
    }

    rxRuntimeState->channelData = mavlinkChannelData;
    rxRuntimeState->channelCount = MAVLINK_CHANNEL_COUNT;
    rxRuntimeState->rcReadRawFn = mavlinkReadRawRC;
    rxRuntimeState->rcFrameStatusFn = mavlinkFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    const uint32_t baudRate = baudRates[MAVLINK_BAUD_RATE_INDEX];
    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        mavlinkDataReceive,
        rxRuntimeState,
        baudRate,
        MODE_RXTX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0)
    );
    telemetrySharedPort = serialPort;

    return serialPort != NULL;
}

#endif
