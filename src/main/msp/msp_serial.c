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
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "common/streambuf.h"
#include "common/utils.h"

#include "drivers/system.h"

#include "interface/msp.h"
#include "interface/cli.h"

#include "io/serial.h"

#include "msp/msp_serial.h"

static mspPort_t mspPorts[MAX_MSP_PORT_COUNT];

static void resetMspPort(mspPort_t *mspPortToReset, serialPort_t *serialPort)
{
    memset(mspPortToReset, 0, sizeof(mspPort_t));

    mspPortToReset->port = serialPort;
}

void mspSerialAllocatePorts(void)
{
    uint8_t portIndex = 0;
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);
    while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
        mspPort_t *mspPort = &mspPorts[portIndex];
        if (mspPort->port) {
            portIndex++;
            continue;
        }

        serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
        if (serialPort) {
            resetMspPort(mspPort, serialPort);
            portIndex++;
        }

        portConfig = findNextSerialPortConfig(FUNCTION_MSP);
    }
}

void mspSerialReleasePortIfAllocated(serialPort_t *serialPort)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t *candidateMspPort = &mspPorts[portIndex];
        if (candidateMspPort->port == serialPort) {
            closeSerialPort(serialPort);
            memset(candidateMspPort, 0, sizeof(mspPort_t));
        }
    }
}

static bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c)
{
    if (mspPort->c_state == MSP_IDLE) {
        if (c == '$') {
            mspPort->c_state = MSP_HEADER_START;
        } else {
            return false;
        }
    } else if (mspPort->c_state == MSP_HEADER_START) {
        mspPort->c_state = (c == 'M') ? MSP_HEADER_M : MSP_IDLE;
    } else if (mspPort->c_state == MSP_HEADER_M) {
        mspPort->c_state = MSP_IDLE;
        switch (c) {
            case '<': // COMMAND
                mspPort->packetType = MSP_PACKET_COMMAND;
                mspPort->c_state = MSP_HEADER_ARROW;
                break;
            case '>': // REPLY
                mspPort->packetType = MSP_PACKET_REPLY;
                mspPort->c_state = MSP_HEADER_ARROW;
                break;
            default:
                break;
        }
    } else if (mspPort->c_state == MSP_HEADER_ARROW) {
        if (c > MSP_PORT_INBUF_SIZE) {
            mspPort->c_state = MSP_IDLE;
        } else {
            mspPort->dataSize = c;
            mspPort->offset = 0;
            mspPort->checksum = 0;
            mspPort->checksum ^= c;
            mspPort->c_state = MSP_HEADER_SIZE;
        }
    } else if (mspPort->c_state == MSP_HEADER_SIZE) {
        mspPort->cmdMSP = c;
        mspPort->checksum ^= c;
        mspPort->c_state = MSP_HEADER_CMD;
    } else if (mspPort->c_state == MSP_HEADER_CMD && mspPort->offset < mspPort->dataSize) {
        mspPort->checksum ^= c;
        mspPort->inBuf[mspPort->offset++] = c;
    } else if (mspPort->c_state == MSP_HEADER_CMD && mspPort->offset >= mspPort->dataSize) {
        if (mspPort->checksum == c) {
            mspPort->c_state = MSP_COMMAND_RECEIVED;
        } else {
            mspPort->c_state = MSP_IDLE;
        }
    }
    return true;
}

static uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len)
{
    while (len-- > 0) {
        checksum ^= *data++;
    }
    return checksum;
}

#define JUMBO_FRAME_SIZE_LIMIT 255

static int mspSerialEncode(mspPort_t *msp, mspPacket_t *packet)
{
    serialBeginWrite(msp->port);
    const int len = sbufBytesRemaining(&packet->buf);
    const int mspLen = len < JUMBO_FRAME_SIZE_LIMIT ? len : JUMBO_FRAME_SIZE_LIMIT;
    uint8_t hdr[8] = {
        '$',
        'M',
        packet->result == MSP_RESULT_ERROR ? '!' : packet->direction == MSP_DIRECTION_REPLY ? '>' : '<',
        mspLen,
        packet->cmd
    };
    int hdrLen = 5;
#define CHECKSUM_STARTPOS 3  // checksum starts from mspLen field
    if (len >= JUMBO_FRAME_SIZE_LIMIT) {
        hdrLen += 2;
        hdr[5] = len & 0xff;
        hdr[6] = (len >> 8) & 0xff;
    }
    serialWriteBuf(msp->port, hdr, hdrLen);
    uint8_t checksum = mspSerialChecksumBuf(0, hdr + CHECKSUM_STARTPOS, hdrLen - CHECKSUM_STARTPOS);
    if (len > 0) {
        serialWriteBuf(msp->port, sbufPtr(&packet->buf), len);
        checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), len);
    }
    serialWriteBuf(msp->port, &checksum, 1);
    serialEndWrite(msp->port);
    return sizeof(hdr) + len + 1; // header, data, and checksum
}

static mspPostProcessFnPtr mspSerialProcessReceivedCommand(mspPort_t *msp, mspProcessCommandFnPtr mspProcessCommandFn)
{
    static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];

    mspPacket_t reply = {
        .buf = { .ptr = outBuf, .end = ARRAYEND(outBuf), },
        .cmd = -1,
        .result = 0,
        .direction = MSP_DIRECTION_REPLY,
    };
    uint8_t *outBufHead = reply.buf.ptr;

    mspPacket_t command = {
        .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize, },
        .cmd = msp->cmdMSP,
        .result = 0,
        .direction = MSP_DIRECTION_REQUEST,
    };

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    const mspResult_e status = mspProcessCommandFn(&command, &reply, &mspPostProcessFn);

    if (status != MSP_RESULT_NO_REPLY) {
        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
        mspSerialEncode(msp, &reply);
    }

    return mspPostProcessFn;
}

static void mspEvaluateNonMspData(mspPort_t * mspPort, uint8_t receivedChar)
{
#ifdef USE_CLI
    if (receivedChar == '#') {
        mspPort->pendingRequest = MSP_PENDING_CLI;
        return;
    }
#endif

    if (receivedChar == serialConfig()->reboot_character) {
        mspPort->pendingRequest = MSP_PENDING_BOOTLOADER;
        return;
    }
}

static void mspProcessPendingRequest(mspPort_t * mspPort)
{
    // If no request is pending or 100ms guard time has not elapsed - do nothing
    if ((mspPort->pendingRequest == MSP_PENDING_NONE) || (millis() - mspPort->lastActivityMs < 100)) {
        return;
    }

    switch(mspPort->pendingRequest) {
        case MSP_PENDING_BOOTLOADER:
            systemResetToBootloader();
            break;

#ifdef USE_CLI
        case MSP_PENDING_CLI:
            cliEnter(mspPort->port);
            break;
#endif

        default:
            break;
    }
}

static void mspSerialProcessReceivedReply(mspPort_t *msp, mspProcessReplyFnPtr mspProcessReplyFn)
{
    mspPacket_t reply = {
        .buf = {
            .ptr = msp->inBuf,
            .end = msp->inBuf + msp->dataSize,
        },
        .cmd = msp->cmdMSP,
        .result = 0,
    };

    mspProcessReplyFn(&reply);

    msp->c_state = MSP_IDLE;
}

/*
 * Process MSP commands from serial ports configured as MSP ports.
 *
 * Called periodically by the scheduler.
 */
void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData, mspProcessCommandFnPtr mspProcessCommandFn, mspProcessReplyFnPtr mspProcessReplyFn)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        mspPostProcessFnPtr mspPostProcessFn = NULL;

        if (serialRxBytesWaiting(mspPort->port)) {
            // There are bytes incoming - abort pending request
            mspPort->lastActivityMs = millis();
            mspPort->pendingRequest = MSP_PENDING_NONE;

            while (serialRxBytesWaiting(mspPort->port)) {
                const uint8_t c = serialRead(mspPort->port);
                const bool consumed = mspSerialProcessReceivedData(mspPort, c);

                if (!consumed && evaluateNonMspData == MSP_EVALUATE_NON_MSP_DATA) {
                    mspEvaluateNonMspData(mspPort, c);
                }

                if (mspPort->c_state == MSP_COMMAND_RECEIVED) {
                    if (mspPort->packetType == MSP_PACKET_COMMAND) {
                        mspPostProcessFn = mspSerialProcessReceivedCommand(mspPort, mspProcessCommandFn);
                    } else if (mspPort->packetType == MSP_PACKET_REPLY) {
                        mspSerialProcessReceivedReply(mspPort, mspProcessReplyFn);
                    }

                    mspPort->c_state = MSP_IDLE;
                    break; // process one command at a time so as not to block.
                }
            }

            if (mspPostProcessFn) {
                waitForSerialPortToFinishTransmitting(mspPort->port);
                mspPostProcessFn(mspPort->port);
            }
        }
        else {
            mspProcessPendingRequest(mspPort);
        }
    }
}

bool mspSerialWaiting(void)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        if (serialRxBytesWaiting(mspPort->port)) {
            return true;
        }
    }
    return false;
}

void mspSerialInit(void)
{
    memset(mspPorts, 0, sizeof(mspPorts));
    mspSerialAllocatePorts();
}

int mspSerialPush(uint8_t cmd, uint8_t *data, int datalen, mspDirection_e direction)
{
    int ret = 0;

    for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
        if (mspPort->port->identifier == SERIAL_PORT_USB_VCP) {
            continue;
        }

        mspPacket_t push = {
            .buf = { .ptr = data, .end = data + datalen, },
            .cmd = cmd,
            .result = 0,
            .direction = direction,
        };

        ret = mspSerialEncode(mspPort, &push);
    }
    return ret; // return the number of bytes written
}


uint32_t mspSerialTxBytesFree(void)
{
    uint32_t ret = UINT32_MAX;

    for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
        if (mspPort->port->identifier == SERIAL_PORT_USB_VCP) {
            continue;
        }

        const uint32_t bytesFree = serialTxBytesFree(mspPort->port);
        if (bytesFree < ret) {
            ret = bytesFree;
        }
    }

    return ret;
}
