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

#include "rx/rx.h"
#include "rx/mavlink.h"

#define MAVLINK_CHANNEL_COUNT 18
static uint16_t mavlinkChannelData[MAVLINK_CHANNEL_COUNT];
static bool frameReceived;

void mavlinkRxHandleMessage(const mavlink_rc_channels_override_t *msg) {
    if (msg->chan1_raw != 0 && msg->chan1_raw != UINT16_MAX) mavlinkChannelData[0] = msg->chan1_raw;
    if (msg->chan2_raw != 0 && msg->chan2_raw != UINT16_MAX) mavlinkChannelData[1] = msg->chan2_raw;
    if (msg->chan3_raw != 0 && msg->chan3_raw != UINT16_MAX) mavlinkChannelData[2] = msg->chan3_raw;
    if (msg->chan4_raw != 0 && msg->chan4_raw != UINT16_MAX) mavlinkChannelData[3] = msg->chan4_raw;
    if (msg->chan5_raw != 0 && msg->chan5_raw != UINT16_MAX) mavlinkChannelData[4] = msg->chan5_raw;
    if (msg->chan6_raw != 0 && msg->chan6_raw != UINT16_MAX) mavlinkChannelData[5] = msg->chan6_raw;
    if (msg->chan7_raw != 0 && msg->chan7_raw != UINT16_MAX) mavlinkChannelData[6] = msg->chan7_raw;
    if (msg->chan8_raw != 0 && msg->chan8_raw != UINT16_MAX) mavlinkChannelData[7] = msg->chan8_raw;
    if (msg->chan9_raw != 0 && msg->chan9_raw != UINT16_MAX) mavlinkChannelData[8] = msg->chan9_raw;
    if (msg->chan10_raw != 0 && msg->chan10_raw != UINT16_MAX) mavlinkChannelData[9] = msg->chan10_raw;
    if (msg->chan11_raw != 0 && msg->chan11_raw != UINT16_MAX) mavlinkChannelData[10] = msg->chan11_raw;
    if (msg->chan12_raw != 0 && msg->chan12_raw != UINT16_MAX) mavlinkChannelData[11] = msg->chan12_raw;
    if (msg->chan13_raw != 0 && msg->chan13_raw != UINT16_MAX) mavlinkChannelData[12] = msg->chan13_raw;
    if (msg->chan14_raw != 0 && msg->chan14_raw != UINT16_MAX) mavlinkChannelData[13] = msg->chan14_raw;
    if (msg->chan15_raw != 0 && msg->chan15_raw != UINT16_MAX) mavlinkChannelData[14] = msg->chan15_raw;
    if (msg->chan16_raw != 0 && msg->chan16_raw != UINT16_MAX) mavlinkChannelData[15] = msg->chan16_raw;
    if (msg->chan17_raw != 0 && msg->chan17_raw != UINT16_MAX) mavlinkChannelData[16] = msg->chan17_raw;
    if (msg->chan18_raw != 0 && msg->chan18_raw != UINT16_MAX) mavlinkChannelData[17] = msg->chan18_raw;
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

bool mavlinkRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    frameReceived = false;

    rxRuntimeState->channelData = mavlinkChannelData;
    rxRuntimeState->channelCount = MAVLINK_CHANNEL_COUNT;
    rxRuntimeState->rcReadRawFn = mavlinkReadRawRC;
    rxRuntimeState->rcFrameStatusFn = mavlinkFrameStatus;

    for (int i = 0; i < MAVLINK_CHANNEL_COUNT; ++i) {
        mavlinkChannelData[i] = rxConfig->midrc;;
    }

    return true;
}

#endif
