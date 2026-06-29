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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RX_MSP

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/time.h"
#include "pg/rx.h"
#include "rx/rx.h"
#include "rx/msp.h"

static uint16_t mspFrame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static bool rxMspFrameDone = false;
static bool rxMspOverrideFrameDone = false;
static bool rxMspRcFrameEverReceived = false;
static timeMs_t lastRxMspRcFrameMs = 0;
static uint8_t lastRxMspRcFrameChannelCount = 0;

// Substitute MSP RC values into the override path only if a frame containing
// the channel arrived within this window. Without this guard:
//  - empty mspFrame[] (companion never sent, or stopped) would clamp masked
//    channels to rx_min_usec the instant BOXMSPOVERRIDE activated;
//  - a short MSP_SET_RAW_RC frame (e.g. AETR-only) would still leave any
//    masked AUX channel reading from the zero-filled tail of mspFrame[].
// 300 ms gives honest support for ~5 Hz MSP RC with timing margin.
#define RX_MSP_RC_FRAME_FRESH_MS 300

float rxMspReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);
    return mspFrame[chan];
}

/*
 * Called from MSP command handler - mspFcProcessCommand
 */
void rxMspFrameReceive(const uint16_t *frame, int channelCount)
{
    for (int i = 0; i < channelCount; i++) {
        mspFrame[i] = frame[i];
    }

    // Any channels not provided will be reset to zero
    for (int i = channelCount; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        mspFrame[i] = 0;
    }

    rxMspFrameDone = true;
    rxMspOverrideFrameDone = true;
    lastRxMspRcFrameMs = millis();
    lastRxMspRcFrameChannelCount = channelCount;
    rxMspRcFrameEverReceived = true;
}

bool rxMspIsRcChannelFresh(uint8_t chan)
{
    if (!rxMspRcFrameEverReceived) {
        return false;
    }
    if (chan >= lastRxMspRcFrameChannelCount) {
        return false;
    }
    return (millis() - lastRxMspRcFrameMs) <= RX_MSP_RC_FRAME_FRESH_MS;
}

static uint8_t rxMspFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    if (!rxMspFrameDone) {
        return RX_FRAME_PENDING;
    }

    rxMspFrameDone = false;
    rxRuntimeState->lastRcFrameTimeUs = micros();
    return RX_FRAME_COMPLETE;
}

#if defined(USE_RX_MSP_OVERRIDE)
uint8_t rxMspOverrideFrameStatus(void)
{
    if (!rxMspOverrideFrameDone) {
        return RX_FRAME_PENDING;
    }

    rxMspOverrideFrameDone = false;
    return RX_FRAME_COMPLETE;
}
#endif

void rxMspInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxConfig);

    rxRuntimeState->channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT;
    rxRuntimeState->rcReadRawFn = rxMspReadRawRC;
    rxRuntimeState->rcFrameStatusFn = rxMspFrameStatus;
}
#endif
