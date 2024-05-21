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
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_SBUS

#include "build/debug.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sbus.h"
#include "rx/sbus_channels.h"

/*
 * Observations
 *
 * FrSky X8R
 * time between frames: 6ms.
 * time to send frame: 3ms.
*
 * Futaba R6208SB/R6303SB
 * time between frames: 11ms.
 * time to send frame: 3ms.
 */

#define SBUS_BAUDRATE                 100000
#define SBUS_RX_REFRESH_RATE          11000
#define SBUS_TIME_NEEDED_PER_FRAME    3000

#define SBUS_FAST_BAUDRATE              200000
#define SBUS_FAST_RX_REFRESH_RATE       6000

#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)

#define SBUS_FRAME_BEGIN_BYTE 0x0F

#if !defined(SBUS_PORT_OPTIONS)
#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#endif

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

enum {
    DEBUG_SBUS_FRAME_FLAGS = 0,
    DEBUG_SBUS_STATE_FLAGS,
    DEBUG_SBUS_FRAME_TIME,
};

struct sbusFrame_s {
    uint8_t syncByte;
    sbusChannels_t channels;
    /**
     * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     *
     * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
     * and
     * https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
     */
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union sbusFrame_u {
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;

typedef struct sbusFrameData_s {
    sbusFrame_t frame;
    timeUs_t startAtUs;
    uint8_t position;
    bool done;
} sbusFrameData_t;

#ifdef USE_SERIALTX
static serialPort_t *sBusTxPort;
#endif

// Receive ISR callback
static void sbusDataReceive(uint16_t c, void *data)
{
    sbusFrameData_t *sbusFrameData = data;

    const timeUs_t nowUs = microsISR();

    const timeDelta_t sbusFrameTime = cmpTimeUs(nowUs, sbusFrameData->startAtUs);

    if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {
        sbusFrameData->position = 0;
    }

    if (sbusFrameData->position == 0) {
        if (c != SBUS_FRAME_BEGIN_BYTE) {
            return;
        }
        sbusFrameData->startAtUs = nowUs;
    }

    if (sbusFrameData->position < SBUS_FRAME_SIZE) {
        sbusFrameData->frame.bytes[sbusFrameData->position++] = (uint8_t)c;
        if (sbusFrameData->position < SBUS_FRAME_SIZE) {
            sbusFrameData->done = false;
        } else {
            sbusFrameData->done = true;
            DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_TIME, sbusFrameTime);
        }
    }
}

static uint8_t sbusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    sbusFrameData_t *sbusFrameData = rxRuntimeState->frameData;
    if (!sbusFrameData->done) {
        return RX_FRAME_PENDING;
    }
    sbusFrameData->done = false;

    DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_FLAGS, sbusFrameData->frame.frame.channels.flags);

    const uint8_t frameStatus = sbusChannelsDecode(rxRuntimeState, &sbusFrameData->frame.frame.channels);

    if (!(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
        rxRuntimeState->lastRcFrameTimeUs = sbusFrameData->startAtUs;
    }

    return frameStatus;
}

bool sbusInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    static uint16_t sbusChannelData[SBUS_MAX_CHANNEL];
    static sbusFrameData_t sbusFrameData;
    static uint32_t sbusBaudRate;

    rxRuntimeState->channelData = sbusChannelData;
    rxRuntimeState->frameData = &sbusFrameData;
    sbusChannelsInit(rxConfig, rxRuntimeState);

    rxRuntimeState->channelCount = SBUS_MAX_CHANNEL;

    if (rxConfig->sbus_baud_fast) {
        sbusBaudRate  = SBUS_FAST_BAUDRATE;
    } else {
        sbusBaudRate  = SBUS_BAUDRATE;
    }

    rxRuntimeState->rcFrameStatusFn = sbusFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig, rxRuntimeState->serialrxProvider);
#else
    bool portShared = false;
#endif

    // On SBUS, SERIAL_INVERTED is default
    const portOptions_e portOptions = SBUS_PORT_OPTIONS
        | (rxConfig->serialrx_inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED)
        | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0);
    serialPort_t *sBusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        sbusDataReceive,
        &sbusFrameData,
        sbusBaudRate,
        portShared ? MODE_RXTX : MODE_RX,
        portOptions
        );

    if (rxConfig->rssi_src_frame_errors) {
        rssiSource = RSSI_SOURCE_FRAME_ERRORS;
    }

#ifdef USE_TELEMETRY
    if (portShared) {
        telemetrySharedPort = sBusPort;
    }
#endif

    return sBusPort != NULL;
}


#ifdef USE_SERIALTX
bool sbusTxInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_TX_SERIAL);

    if (!portConfig) {
        return false;
    }

    sBusTxPort = openSerialPort(portConfig->identifier,
        FUNCTION_TX_SERIAL,
        NULL,
        NULL,
        SBUS_BAUDRATE,
        MODE_TX,
        SBUS_PORT_OPTIONS | (rxConfig()->serialtx_inverted ? 0 : SERIAL_INVERTED));

    return sBusTxPort != NULL;
}

bool sbusTx(uint16_t *channels, int channelCount)
{
    struct sbusFrame_s sbusFrame;
    uint16_t channelsFull[16];
    int channel;

    if (sBusTxPort == NULL) {
        return false;
    }

    if (channelCount > 16) {
        channelCount = 16;
    }

    for (channel = 0; channel < channelCount; channel++) {
        // Scale to standard SBus range
        channelsFull[channel] = (channels[channel] * (SBUS_DIGITAL_CHANNEL_MAX - SBUS_DIGITAL_CHANNEL_MIN) / 2048) + SBUS_DIGITAL_CHANNEL_MIN;
    }
    for (; channel < 16; channel++) {
        // Midpoint
        channelsFull[channel] = 993;
    }
    memset(&sbusFrame, 0, sizeof(sbusFrame));
    sbusFrame.syncByte = SBUS_FRAME_BEGIN_BYTE;
    sbusFrame.channels.chan0 = channelsFull[0];
    sbusFrame.channels.chan1 = channelsFull[1];
    sbusFrame.channels.chan2 = channelsFull[2];
    sbusFrame.channels.chan3 = channelsFull[3];
    sbusFrame.channels.chan4 = channelsFull[4];
    sbusFrame.channels.chan5 = channelsFull[5];
    sbusFrame.channels.chan6 = channelsFull[6];
    sbusFrame.channels.chan7 = channelsFull[7];
    sbusFrame.channels.chan8 = channelsFull[8];
    sbusFrame.channels.chan9 = channelsFull[9];
    sbusFrame.channels.chan10 = channelsFull[10];
    sbusFrame.channels.chan11 = channelsFull[11];
    sbusFrame.channels.chan12 = channelsFull[12];
    sbusFrame.channels.chan13 = channelsFull[13];
    sbusFrame.channels.chan14 = channelsFull[14];
    sbusFrame.channels.chan15 = channelsFull[15];
    sbusFrame.channels.flags = 0x00;
    sbusFrame.endByte = 0x00;

    serialWriteBuf(sBusTxPort, (uint8_t *)&sbusFrame, sizeof(sbusFrame));

    return true;
}
#endif // USE_SERIALTX

#endif
