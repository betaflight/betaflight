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
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "cli/cli.h"

#include "common/streambuf.h"
#include "common/utils.h"
#include "common/crc.h"

#include "drivers/system.h"

#include "io/displayport_msp.h"

#include "msp/msp.h"

#include "msp_serial.h"

static mspPort_t mspPorts[MAX_MSP_PORT_COUNT];

static void resetMspPort(mspPort_t *mspPortToReset, serialPort_t *serialPort, bool sharedWithTelemetry)
{
    memset(mspPortToReset, 0, sizeof(mspPort_t));

    mspPortToReset->port = serialPort;
    mspPortToReset->sharedWithTelemetry = sharedWithTelemetry;
    mspPortToReset->descriptor = mspDescriptorAlloc();
}

void mspSerialAllocatePorts(void)
{
    uint8_t portIndex = 0;
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);
    while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
        mspPort_t *mspPort = &mspPorts[portIndex];

        if (mspPort->port) {
            portIndex++;
            continue;
        }

        serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
        if (serialPort) {
            bool sharedWithTelemetry = isSerialPortShared(portConfig, FUNCTION_MSP, TELEMETRY_PORT_FUNCTIONS_MASK);
            resetMspPort(mspPort, serialPort, sharedWithTelemetry);

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

#if defined(USE_TELEMETRY)
void mspSerialReleaseSharedTelemetryPorts(void) {
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t *candidateMspPort = &mspPorts[portIndex];
        if (candidateMspPort->sharedWithTelemetry) {
            closeSerialPort(candidateMspPort->port);
            memset(candidateMspPort, 0, sizeof(mspPort_t));
        }
    }
}
#endif

static bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c)
{
    switch (mspPort->c_state) {
        default:
        case MSP_IDLE:      // Waiting for '$' character
            if (c == '$') {
                mspPort->c_state = MSP_HEADER_START;
            } else {
                return false;
            }
            break;

        case MSP_HEADER_START:  // Waiting for 'M' (MSPv1 / MSPv2_over_v1) or 'X' (MSPv2 native)
            mspPort->offset = 0;
            mspPort->checksum1 = 0;
            mspPort->checksum2 = 0;
            switch (c) {
                case 'M':
                    mspPort->c_state = MSP_HEADER_M;
                    mspPort->mspVersion = MSP_V1;
                    break;
                case 'X':
                    mspPort->c_state = MSP_HEADER_X;
                    mspPort->mspVersion = MSP_V2_NATIVE;
                    break;
                default:
                    mspPort->c_state = MSP_IDLE;
                    break;
            }
            break;

        case MSP_HEADER_M:      // Waiting for '<' / '>'
            mspPort->c_state = MSP_HEADER_V1;
            switch (c) {
                case '<':
                    mspPort->packetType = MSP_PACKET_COMMAND;
                    break;
                case '>':
                    mspPort->packetType = MSP_PACKET_REPLY;
                    break;
                default:
                    mspPort->c_state = MSP_IDLE;
                    break;
            }
            break;

        case MSP_HEADER_X:
            mspPort->c_state = MSP_HEADER_V2_NATIVE;
            switch (c) {
                case '<':
                    mspPort->packetType = MSP_PACKET_COMMAND;
                    break;
                case '>':
                    mspPort->packetType = MSP_PACKET_REPLY;
                    break;
                default:
                    mspPort->c_state = MSP_IDLE;
                    break;
            }
            break;

        case MSP_HEADER_V1:     // Now receive v1 header (size/cmd), this is already checksummable
            mspPort->inBuf[mspPort->offset++] = c;
            mspPort->checksum1 ^= c;
            if (mspPort->offset == sizeof(mspHeaderV1_t)) {
                mspHeaderV1_t * hdr = (mspHeaderV1_t *)&mspPort->inBuf[0];
                // Check incoming buffer size limit
                if (hdr->size > MSP_PORT_INBUF_SIZE) {
                    mspPort->c_state = MSP_IDLE;
                }
                else if (hdr->cmd == MSP_V2_FRAME_ID) {
                    // MSPv1 payload must be big enough to hold V2 header + extra checksum
                    if (hdr->size >= sizeof(mspHeaderV2_t) + 1) {
                        mspPort->mspVersion = MSP_V2_OVER_V1;
                        mspPort->c_state = MSP_HEADER_V2_OVER_V1;
                    } else {
                        mspPort->c_state = MSP_IDLE;
                    }
                } else {
                    mspPort->dataSize = hdr->size;
                    mspPort->cmdMSP = hdr->cmd;
                    mspPort->cmdFlags = 0;
                    mspPort->offset = 0;                // re-use buffer
                    mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;    // If no payload - jump to checksum byte
                }
            }
            break;

        case MSP_PAYLOAD_V1:
            mspPort->inBuf[mspPort->offset++] = c;
            mspPort->checksum1 ^= c;
            if (mspPort->offset == mspPort->dataSize) {
                mspPort->c_state = MSP_CHECKSUM_V1;
            }
            break;

        case MSP_CHECKSUM_V1:
            if (mspPort->checksum1 == c) {
                mspPort->c_state = MSP_COMMAND_RECEIVED;
            } else {
                mspPort->c_state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_V2_OVER_V1:     // V2 header is part of V1 payload - we need to calculate both checksums now
            mspPort->inBuf[mspPort->offset++] = c;
            mspPort->checksum1 ^= c;
            mspPort->checksum2 = crc8_dvb_s2(mspPort->checksum2, c);
            if (mspPort->offset == (sizeof(mspHeaderV2_t) + sizeof(mspHeaderV1_t))) {
                mspHeaderV2_t * hdrv2 = (mspHeaderV2_t *)&mspPort->inBuf[sizeof(mspHeaderV1_t)];
                if (hdrv2->size > MSP_PORT_INBUF_SIZE) {
                    mspPort->c_state = MSP_IDLE;
                } else {
                    mspPort->dataSize = hdrv2->size;
                    mspPort->cmdMSP = hdrv2->cmd;
                    mspPort->cmdFlags = hdrv2->flags;
                    mspPort->offset = 0;                // re-use buffer
                    mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V2_OVER_V1 : MSP_CHECKSUM_V2_OVER_V1;
                }
            }
            break;

        case MSP_PAYLOAD_V2_OVER_V1:
            mspPort->checksum2 = crc8_dvb_s2(mspPort->checksum2, c);
            mspPort->checksum1 ^= c;
            mspPort->inBuf[mspPort->offset++] = c;

            if (mspPort->offset == mspPort->dataSize) {
                mspPort->c_state = MSP_CHECKSUM_V2_OVER_V1;
            }
            break;

        case MSP_CHECKSUM_V2_OVER_V1:
            mspPort->checksum1 ^= c;
            if (mspPort->checksum2 == c) {
                mspPort->c_state = MSP_CHECKSUM_V1; // Checksum 2 correct - verify v1 checksum
            } else {
                mspPort->c_state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_V2_NATIVE:
            mspPort->inBuf[mspPort->offset++] = c;
            mspPort->checksum2 = crc8_dvb_s2(mspPort->checksum2, c);
            if (mspPort->offset == sizeof(mspHeaderV2_t)) {
                mspHeaderV2_t * hdrv2 = (mspHeaderV2_t *)&mspPort->inBuf[0];
                mspPort->dataSize = hdrv2->size;
                mspPort->cmdMSP = hdrv2->cmd;
                mspPort->cmdFlags = hdrv2->flags;
                mspPort->offset = 0;                // re-use buffer
                mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V2_NATIVE : MSP_CHECKSUM_V2_NATIVE;
            }
            break;

        case MSP_PAYLOAD_V2_NATIVE:
            mspPort->checksum2 = crc8_dvb_s2(mspPort->checksum2, c);
            mspPort->inBuf[mspPort->offset++] = c;

            if (mspPort->offset == mspPort->dataSize) {
                mspPort->c_state = MSP_CHECKSUM_V2_NATIVE;
            }
            break;

        case MSP_CHECKSUM_V2_NATIVE:
            if (mspPort->checksum2 == c) {
                mspPort->c_state = MSP_COMMAND_RECEIVED;
            } else {
                mspPort->c_state = MSP_IDLE;
            }
            break;
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
static int mspSerialSendFrame(mspPort_t *msp, const uint8_t * hdr, int hdrLen, const uint8_t * data, int dataLen, const uint8_t * crc, int crcLen)
{
    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    const int totalFrameLength = hdrLen + dataLen + crcLen;
    if (!isSerialTransmitBufferEmpty(msp->port) && ((int)serialTxBytesFree(msp->port) < totalFrameLength)) {
        return 0;
    }

    // Transmit frame
    serialBeginWrite(msp->port);
    serialWriteBuf(msp->port, hdr, hdrLen);
    serialWriteBuf(msp->port, data, dataLen);
    serialWriteBuf(msp->port, crc, crcLen);
    serialEndWrite(msp->port);

    return totalFrameLength;
}

static int mspSerialEncode(mspPort_t *msp, mspPacket_t *packet, mspVersion_e mspVersion)
{
    static const uint8_t mspMagic[MSP_VERSION_COUNT] = MSP_VERSION_MAGIC_INITIALIZER;
    const int dataLen = sbufBytesRemaining(&packet->buf);
    uint8_t hdrBuf[16] = { '$', mspMagic[mspVersion], packet->result == MSP_RESULT_ERROR ? '!' : '>'};
    uint8_t crcBuf[2];
    uint8_t checksum;
    int hdrLen = 3;
    int crcLen = 0;

    #define V1_CHECKSUM_STARTPOS 3
    if (mspVersion == MSP_V1) {
        mspHeaderV1_t * hdrV1 = (mspHeaderV1_t *)&hdrBuf[hdrLen];
        hdrLen += sizeof(mspHeaderV1_t);
        hdrV1->cmd = packet->cmd;

        // Add JUMBO-frame header if necessary
        if (dataLen >= JUMBO_FRAME_SIZE_LIMIT) {
            mspHeaderJUMBO_t * hdrJUMBO = (mspHeaderJUMBO_t *)&hdrBuf[hdrLen];
            hdrLen += sizeof(mspHeaderJUMBO_t);

            hdrV1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdrJUMBO->size = dataLen;
        } else {
            hdrV1->size = dataLen;
        }

        // Pre-calculate CRC
        checksum = mspSerialChecksumBuf(0, hdrBuf + V1_CHECKSUM_STARTPOS, hdrLen - V1_CHECKSUM_STARTPOS);
        checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), dataLen);
        crcBuf[crcLen++] = checksum;
    } else if (mspVersion == MSP_V2_OVER_V1) {
        mspHeaderV1_t * hdrV1 = (mspHeaderV1_t *)&hdrBuf[hdrLen];

        hdrLen += sizeof(mspHeaderV1_t);

        mspHeaderV2_t * hdrV2 = (mspHeaderV2_t *)&hdrBuf[hdrLen];
        hdrLen += sizeof(mspHeaderV2_t);

        const int v1PayloadSize = sizeof(mspHeaderV2_t) + dataLen + 1;  // MSPv2 header + data payload + MSPv2 checksum
        hdrV1->cmd = MSP_V2_FRAME_ID;

        // Add JUMBO-frame header if necessary
        if (v1PayloadSize >= JUMBO_FRAME_SIZE_LIMIT) {
            mspHeaderJUMBO_t * hdrJUMBO = (mspHeaderJUMBO_t *)&hdrBuf[hdrLen];
            hdrLen += sizeof(mspHeaderJUMBO_t);

            hdrV1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdrJUMBO->size = v1PayloadSize;
        } else {
            hdrV1->size = v1PayloadSize;
        }

        // Fill V2 header
        hdrV2->flags = packet->flags;
        hdrV2->cmd = packet->cmd;
        hdrV2->size = dataLen;

        // V2 CRC: only V2 header + data payload
        checksum = crc8_dvb_s2_update(0, (uint8_t *)hdrV2, sizeof(mspHeaderV2_t));
        checksum = crc8_dvb_s2_update(checksum, sbufPtr(&packet->buf), dataLen);
        crcBuf[crcLen++] = checksum;

        // V1 CRC: All headers + data payload + V2 CRC byte
        checksum = mspSerialChecksumBuf(0, hdrBuf + V1_CHECKSUM_STARTPOS, hdrLen - V1_CHECKSUM_STARTPOS);
        checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), dataLen);
        checksum = mspSerialChecksumBuf(checksum, crcBuf, crcLen);
        crcBuf[crcLen++] = checksum;
    } else if (mspVersion == MSP_V2_NATIVE) {
        mspHeaderV2_t * hdrV2 = (mspHeaderV2_t *)&hdrBuf[hdrLen];
        hdrLen += sizeof(mspHeaderV2_t);

        hdrV2->flags = packet->flags;
        hdrV2->cmd = packet->cmd;
        hdrV2->size = dataLen;

        checksum = crc8_dvb_s2_update(0, (uint8_t *)hdrV2, sizeof(mspHeaderV2_t));
        checksum = crc8_dvb_s2_update(checksum, sbufPtr(&packet->buf), dataLen);
        crcBuf[crcLen++] = checksum;
    } else {
        // Shouldn't get here
        return 0;
    }

    // Send the frame
    return mspSerialSendFrame(msp, hdrBuf, hdrLen, sbufPtr(&packet->buf), dataLen, crcBuf, crcLen);
}

static mspPostProcessFnPtr mspSerialProcessReceivedCommand(mspPort_t *msp, mspProcessCommandFnPtr mspProcessCommandFn)
{
    static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];

    mspPacket_t reply = {
        .buf = { .ptr = outBuf, .end = ARRAYEND(outBuf), },
        .cmd = -1,
        .flags = 0,
        .result = 0,
        .direction = MSP_DIRECTION_REPLY,
    };
    uint8_t *outBufHead = reply.buf.ptr;

    mspPacket_t command = {
        .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize, },
        .cmd = msp->cmdMSP,
        .flags = msp->cmdFlags,
        .result = 0,
        .direction = MSP_DIRECTION_REQUEST,
    };

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    const mspResult_e status = mspProcessCommandFn(msp->descriptor, &command, &reply, &mspPostProcessFn);

    if (status != MSP_RESULT_NO_REPLY) {
        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
        mspSerialEncode(msp, &reply, msp->mspVersion);
    }

    return mspPostProcessFn;
}

static void mspEvaluateNonMspData(mspPort_t * mspPort, uint8_t receivedChar)
{
   if (receivedChar == serialConfig()->reboot_character) {
        mspPort->pendingRequest = MSP_PENDING_BOOTLOADER_ROM;
#ifdef USE_CLI
   } else if (receivedChar == '#') {
        mspPort->pendingRequest = MSP_PENDING_CLI;
#endif
#if defined(USE_FLASH_BOOT_LOADER)
   } else if (receivedChar == 'F') {
        mspPort->pendingRequest = MSP_PENDING_BOOTLOADER_FLASH;
#endif
    }
}

static void mspProcessPendingRequest(mspPort_t * mspPort)
{
    // If no request is pending or 100ms guard time has not elapsed - do nothing
    if ((mspPort->pendingRequest == MSP_PENDING_NONE) || (millis() - mspPort->lastActivityMs < 100)) {
        return;
    }

    switch(mspPort->pendingRequest) {
    case MSP_PENDING_BOOTLOADER_ROM:
        systemResetToBootloader(BOOTLOADER_REQUEST_ROM);

        break;
#if defined(USE_FLASH_BOOT_LOADER)
    case MSP_PENDING_BOOTLOADER_FLASH:
        systemResetToBootloader(BOOTLOADER_REQUEST_FLASH);

        break;
#endif
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
        } else {
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

int mspSerialPush(serialPortIdentifier_e port, uint8_t cmd, uint8_t *data, int datalen, mspDirection_e direction)
{
    int ret = 0;

    for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];

        // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
        if (!mspPort->port
#ifndef USE_MSP_PUSH_OVER_VCP
            || mspPort->port->identifier == SERIAL_PORT_USB_VCP
#endif
            || (port != SERIAL_PORT_NONE && mspPort->port->identifier != port)) {
            continue;
        }

        mspPacket_t push = {
            .buf = { .ptr = data, .end = data + datalen, },
            .cmd = cmd,
            .result = 0,
            .direction = direction,
        };

        ret = mspSerialEncode(mspPort, &push, MSP_V1);
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
