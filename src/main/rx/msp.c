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
#include "pg/rx.h"
#include "rx/rx.h"
#include "rx/msp.h"

static uint16_t mspFrame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static bool rxMspFrameDone = false;
static bool rxMspOverrideFrameDone = false;

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
}

static uint8_t rxMspFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    if (!rxMspFrameDone) {
        return RX_FRAME_PENDING;
    }

    rxMspFrameDone = false;
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
