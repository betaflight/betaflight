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
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERIALRX_SUMD

#include "common/crc.h"
#include "common/utils.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sumd.h"

// driver for SUMD receiver using UART2

// Support for SUMD and SUMD V3
// Tested with 16 channels, SUMD supports up to 16(*), SUMD V3 up to 32 (MZ-32) channels, but limit to MAX_SUPPORTED_RC_CHANNEL_COUNT (currently 8, BF 3.4)
// * According to the original SUMD V1 documentation, SUMD V1 already supports up to 32 Channels?!?

#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAX_CHANNEL 32
#define SUMD_BUFFSIZE (SUMD_MAX_CHANNEL * 2 + 5) // 6 channels + 5 = 17 bytes for 6 channels

#define SUMD_BAUDRATE 115200

static bool sumdFrameDone = false;
static uint16_t sumdChannels[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static uint16_t crc;

static uint8_t sumd[SUMD_BUFFSIZE] = { 0, };
static uint8_t sumdChannelCount;

// Receive ISR callback
static void sumdDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    uint32_t sumdTime;
    static uint32_t sumdTimeLast;
    static uint8_t sumdIndex;

    sumdTime = micros();
    if ((sumdTime - sumdTimeLast) > 4000)
        sumdIndex = 0;
    sumdTimeLast = sumdTime;

    if (sumdIndex == 0) {
        if (c != SUMD_SYNCBYTE)
            return;
        else
        {
            sumdFrameDone = false; // lazy main loop didnt fetch the stuff
            crc = 0;
        }
    }
    if (sumdIndex == 2)
        sumdChannelCount = (uint8_t)c;
    if (sumdIndex < SUMD_BUFFSIZE)
        sumd[sumdIndex] = (uint8_t)c;
    sumdIndex++;
    if (sumdIndex < sumdChannelCount * 2 + 4)
        crc = crc16_ccitt(crc, (uint8_t)c);
    else
        if (sumdIndex == sumdChannelCount * 2 + 5) {
            sumdIndex = 0;
            sumdFrameDone = true;
        }
}

#define SUMD_OFFSET_CHANNEL_1_HIGH 3
#define SUMD_OFFSET_CHANNEL_1_LOW 4
#define SUMD_BYTES_PER_CHANNEL 2


#define SUMDV1_FRAME_STATE_OK 0x01
#define SUMDV3_FRAME_STATE_OK 0x03
#define SUMD_FRAME_STATE_FAILSAFE 0x81

static uint8_t sumdFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    uint8_t frameStatus = RX_FRAME_PENDING;

    if (!sumdFrameDone) {
        return frameStatus;
    }

    sumdFrameDone = false;

    // verify CRC
    if (crc != ((sumd[SUMD_BYTES_PER_CHANNEL * sumdChannelCount + SUMD_OFFSET_CHANNEL_1_HIGH] << 8) |
            (sumd[SUMD_BYTES_PER_CHANNEL * sumdChannelCount + SUMD_OFFSET_CHANNEL_1_LOW])))
        return frameStatus;

    switch (sumd[1]) {
        case SUMD_FRAME_STATE_FAILSAFE:
            frameStatus = RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
            break;
        case SUMDV1_FRAME_STATE_OK:
        case SUMDV3_FRAME_STATE_OK:
            frameStatus = RX_FRAME_COMPLETE;
            break;
        default:
            return frameStatus;
    }

    unsigned channelsToProcess = MIN(sumdChannelCount, MAX_SUPPORTED_RC_CHANNEL_COUNT);

    for (unsigned channelIndex = 0; channelIndex < channelsToProcess; channelIndex++) {
        sumdChannels[channelIndex] = (
            (sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + SUMD_OFFSET_CHANNEL_1_HIGH] << 8) |
            sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + SUMD_OFFSET_CHANNEL_1_LOW]
        );
    }
    return frameStatus;
}

static uint16_t sumdReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return sumdChannels[chan] / 8;
}

bool sumdInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = MIN(SUMD_MAX_CHANNEL, MAX_SUPPORTED_RC_CHANNEL_COUNT);
    rxRuntimeConfig->rxRefreshRate = 11000;

    rxRuntimeConfig->rcReadRawFn = sumdReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = sumdFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig);
#else
    bool portShared = false;
#endif

    serialPort_t *sumdPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        sumdDataReceive,
        NULL,
        SUMD_BAUDRATE,
        portShared ? MODE_RXTX : MODE_RX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
        );

#ifdef USE_TELEMETRY
    if (portShared) {
        telemetrySharedPort = sumdPort;
    }
#endif

    return sumdPort != NULL;
}
#endif
