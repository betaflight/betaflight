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

#include "common/streambuf.h"
#include "common/utils.h"

#include "drivers/serial.h"

#include "io/serial.h"

#include "msp/msp.h"
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

        serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
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
        mspPort->c_state = (c == '<') ? MSP_HEADER_ARROW : MSP_IDLE;
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
    const uint8_t hdr[5] = {'$', 'M', packet->result == MSP_RESULT_ERROR ? '!' : '>', mspLen, packet->cmd};
    serialWriteBuf(msp->port, hdr, sizeof(hdr));
    uint8_t checksum = mspSerialChecksumBuf(0, hdr + 3, 2); // checksum starts from len field
    if (len >= JUMBO_FRAME_SIZE_LIMIT) {
        serialWrite(msp->port, len & 0xff);
        checksum ^= len & 0xff;
        serialWrite(msp->port, (len >> 8) & 0xff);
        checksum ^= (len >> 8) & 0xff;
    }
    if (len > 0) {
        serialWriteBuf(msp->port, sbufPtr(&packet->buf), len);
        checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), len);
    }
    serialWrite(msp->port, checksum);
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
    };
    uint8_t *outBufHead = reply.buf.ptr;

    mspPacket_t command = {
        .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize, },
        .cmd = msp->cmdMSP,
        .result = 0,
    };

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    const mspResult_e status = mspProcessCommandFn(&command, &reply, &mspPostProcessFn);

    if (status != MSP_RESULT_NO_REPLY) {
        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
        mspSerialEncode(msp, &reply);
    }

    msp->c_state = MSP_IDLE;
    return mspPostProcessFn;
}

/*
 * Process MSP commands from serial ports configured as MSP ports.
 *
 * Called periodically by the scheduler.
 */
void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData, mspProcessCommandFnPtr mspProcessCommandFn)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }
        mspPostProcessFnPtr mspPostProcessFn = NULL;
        while (serialRxBytesWaiting(mspPort->port)) {

            const uint8_t c = serialRead(mspPort->port);
            const bool consumed = mspSerialProcessReceivedData(mspPort, c);

            if (!consumed && evaluateNonMspData == MSP_EVALUATE_NON_MSP_DATA) {
                serialEvaluateNonMspData(mspPort->port, c);
            }

            if (mspPort->c_state == MSP_COMMAND_RECEIVED) {
                mspPostProcessFn = mspSerialProcessReceivedCommand(mspPort, mspProcessCommandFn);
                break; // process one command at a time so as not to block.
            }
        }
        if (mspPostProcessFn) {
            waitForSerialPortToFinishTransmitting(mspPort->port);
            mspPostProcessFn(mspPort->port);
        }
    }
}

void mspSerialInit(void)
{
    memset(mspPorts, 0, sizeof(mspPorts));
    mspSerialAllocatePorts();
}

int mspSerialPush(uint8_t cmd, const uint8_t *data, int datalen)
{
    static uint8_t pushBuf[30];
    int ret = 0;

    mspPacket_t push = {
        .buf = { .ptr = pushBuf, .end = ARRAYEND(pushBuf), },
        .cmd = cmd,
        .result = 0,
    };

    for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
        if (mspPort->port->identifier == SERIAL_PORT_USB_VCP) {
            continue;
        }

        sbufWriteData(&push.buf, data, datalen);

        sbufSwitchToReader(&push.buf, pushBuf);

        ret = mspSerialEncode(mspPort, &push);
    }
    return ret; // return the number of bytes written
}

uint32_t mspSerialTxBytesFree()
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
