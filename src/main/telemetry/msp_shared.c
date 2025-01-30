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

#if defined(USE_MSP_OVER_TELEMETRY)

#include "build/build_config.h"

#include "common/utils.h"
#include "common/crc.h"
#include "common/streambuf.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "telemetry/crsf.h"
#include "telemetry/msp_shared.h"
#include "telemetry/smartport.h"

/*
---------------------------------------------------------------
How MSP frames are sent over CRSF:
CRSF frame types: 0x7A for MSP requests, 0x7B for responses.
CRSF extended header frames are used. i.e., Destination and origin addresses added after CRSF type.
CRSF frame structure:
<sync/address><length><type><destination><origin><status><MSP_body><CRC>
Status byte consists of three parts:
    bits 0-3 represent the sequence number of the MSP frame,
    bit 4 checks if current MSP chunk is the beginning of a new frame (1 if true),
    bits 5-6 represent the version number of MSP protocol (MSPv1 or MSPv2)
    bit 7 represents Error (1 if there was an error)
MSP_body is unmodified MSP frame without header ($ + M|X + <|>|!) and CRC.
MSP might be MSPv1 or MSPv2 or MSPv1_Jumbo.

MSP_body might be sent in chunks.
First (or only) chunk must always set start bit (#4) of status byte.
Each next chunk must have increased sequence number in status byte.
Size of chunk is recovered from size of CRSF frame.
Although last / only CRSF frame might have size bigger than needed for MSP-body.
Extra bytes must be ignored. So, the real size of MSP-body must be parsed from the MSP-body itself.
CRSF frames might be any size until maximum of 64 bytes for a CRSF frame.
So, maximum chunk size is 57 bytes. Although, MSP-body might be sent in shorter chunks.
Although, first chunk must consist full size any type of the MSP frame.

MSP-CRC is not sent over CRSF due to ther is already CRC of CRSF frame.
So, it must be recalculated of needed for MSP-receiver.

MSP frame must be returned to the origin address of the request

---------------------------------------------------------------
*/
#define TELEMETRY_MSP_VERSION    2
#define TELEMETRY_MSP_RES_ERROR (-10)

#define TELEMETRY_REQUEST_SKIPS_AFTER_EEPROMWRITE 5

enum { // constants for status of msp-over-telemetry frame
    MSP_STATUS_SEQUENCE_MASK   = 0x0f, // 0b00001111,   // sequence number mask
    MSP_STATUS_START_MASK      = 0x10, // 0b00010000,   // bit of starting frame (if 1, the frame is a first/single chunk of msp-frame)
    MSP_STATUS_VERSION_MASK    = 0x60, // 0b01100000,   // MSP version mask
    MSP_STATUS_ERROR_MASK      = 0x80, // 0b10000000,   // Error bit (1 if error)
    MSP_STATUS_VERSION_SHIFT   = 5,    // MSP version shift
};

enum { // error codes (they are not sent anywhere)
    TELEMETRY_MSP_VER_MISMATCH,
    TELEMETRY_MSP_CRC_ERROR,
    TELEMETRY_MSP_ERROR,
    TELEMETRY_MSP_REQUEST_IS_TOO_BIG,
};

enum { // minimum length for a frame.
    MIN_LENGTH_CHUNK         = 2, // status + at_least_one_byte
    MIN_LENGTH_REQUEST_V1    = 3, // status + length + ID
    MIN_LENGTH_REQUEST_JUMBO = 5, // status + length=FF + ID + length_lo + length_hi
    MIN_LENGTH_REQUEST_V2    = 6, // status + flag + ID_lo + ID_hi + size_lo + size_hi
};

enum { // byte position(index) in msp-over-telemetry request payload
    // MSPv1
    MSP_INDEX_STATUS        = 0,                           // status byte
    MSP_INDEX_SIZE_V1       = MSP_INDEX_STATUS        + 1, // MSPv1 payload size
    MSP_INDEX_ID_V1         = MSP_INDEX_SIZE_V1       + 1, // MSPv1 ID/command/function byte
    MSP_INDEX_PAYLOAD_V1    = MSP_INDEX_ID_V1         + 1, // MSPv1 Payload start / CRC for zero payload

    // MSPv1_Jumbo
    MSP_INDEX_SIZE_JUMBO_LO = MSP_INDEX_PAYLOAD_V1,        // MSPv1_Jumbo Lo byte of payload size
    MSP_INDEX_SIZE_JUMBO_HI = MSP_INDEX_SIZE_JUMBO_LO + 1, // MSPv1_Jumbo Hi byte of payload size
    MSP_INDEX_PAYLOAD_JUMBO = MSP_INDEX_SIZE_JUMBO_HI + 1, // MSPv1_Jumbo first byte of payload itself

    // MSPv2
    MSP_INDEX_FLAG_V2       = MSP_INDEX_SIZE_V1,           // MSPv2 flags byte
    MSP_INDEX_ID_LO         = MSP_INDEX_ID_V1,             // MSPv2 Lo byte of ID/command/function
    MSP_INDEX_ID_HI         = MSP_INDEX_ID_LO         + 1, // MSPv2 Hi byte of ID/command/function
    MSP_INDEX_SIZE_V2_LO    = MSP_INDEX_ID_HI         + 1, // MSPv2 Lo byte of payload size
    MSP_INDEX_SIZE_V2_HI    = MSP_INDEX_SIZE_V2_LO    + 1, // MSPv2 Hi byte of payload size
    MSP_INDEX_PAYLOAD_V2    = MSP_INDEX_SIZE_V2_HI    + 1, // MSPv2 first byte of payload itself
};

STATIC_UNIT_TESTED uint8_t requestBuffer[MSP_TLM_INBUF_SIZE];
STATIC_UNIT_TESTED uint8_t responseBuffer[MSP_TLM_OUTBUF_SIZE];
STATIC_UNIT_TESTED mspPacket_t requestPacket;
STATIC_UNIT_TESTED mspPacket_t responsePacket;
static uint8_t lastRequestVersion; // MSP version of last request. Temporary solution. It's better to keep it in requestPacket.

static mspDescriptor_t mspSharedDescriptor = -1;

void initSharedMsp(void)
{
    responsePacket.buf.ptr = responseBuffer;
    responsePacket.buf.end = ARRAYEND(responseBuffer);

    mspSharedDescriptor = mspDescriptorAlloc();
}

mspDescriptor_t getMspTelemetryDescriptor(void)
{
    return mspSharedDescriptor;
}

static void processMspPacket(void)
{
    responsePacket.cmd = 0;
    responsePacket.result = 0;
    responsePacket.buf.ptr = responseBuffer;
    responsePacket.buf.end = ARRAYEND(responseBuffer);

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    if (mspFcProcessCommand(mspSharedDescriptor, &requestPacket, &responsePacket, &mspPostProcessFn) == MSP_RESULT_ERROR) {
        sbufWriteU8(&responsePacket.buf, TELEMETRY_MSP_ERROR);
    }
    if (mspPostProcessFn) {
        mspPostProcessFn(NULL);
    }

    sbufSwitchToReader(&responsePacket.buf, responseBuffer);
}

static void sendMspErrorResponse(uint8_t error, int16_t cmd)
{
    responsePacket.cmd = cmd;
    responsePacket.result = 0;
    responsePacket.buf.ptr = responseBuffer;

    sbufWriteU8(&responsePacket.buf, error);
    responsePacket.result = TELEMETRY_MSP_RES_ERROR;
    sbufSwitchToReader(&responsePacket.buf, responseBuffer);
}

// despite its name, the function actually handles telemetry frame payload with MSP in it
// it reads the MSP into requestPacket stucture and handles it after receiving all the chunks.
bool handleMspFrame(uint8_t *const payload, uint8_t const payloadLength, uint8_t *const skipsBeforeResponse)
{
    if (payloadLength < MIN_LENGTH_CHUNK) {
        return false;   // prevent analyzing garbage data
    }

    static uint8_t mspStarted = 0;
    static uint8_t lastSeq = 0;

    sbuf_t sbufInput;

    const uint8_t status = payload[MSP_INDEX_STATUS];
    const uint8_t seqNumber = status & MSP_STATUS_SEQUENCE_MASK;
    lastRequestVersion = (status & MSP_STATUS_VERSION_MASK) >> MSP_STATUS_VERSION_SHIFT;

    if (lastRequestVersion > TELEMETRY_MSP_VERSION) {
        sendMspErrorResponse(TELEMETRY_MSP_VER_MISMATCH, 0);
        return true;
    }

    if (status & MSP_STATUS_START_MASK) { // first packet in sequence
        uint16_t mspPayloadSize;
        if (lastRequestVersion == 1) { // MSPv1
            if (payloadLength < MIN_LENGTH_REQUEST_V1) {
                return false;   // prevent analyzing garbage data
            }

            mspPayloadSize = payload[MSP_INDEX_SIZE_V1];
            requestPacket.cmd = payload[MSP_INDEX_ID_V1];
            if (mspPayloadSize == 0xff) { // jumbo frame
                if (payloadLength < MIN_LENGTH_REQUEST_JUMBO) {
                    return false;   // prevent analyzing garbage data
                }
                mspPayloadSize = *(uint16_t*)&payload[MSP_INDEX_SIZE_JUMBO_LO];
                sbufInit(&sbufInput, payload + MSP_INDEX_PAYLOAD_JUMBO, payload + payloadLength);
            } else {
                sbufInit(&sbufInput, payload + MSP_INDEX_PAYLOAD_V1, payload + payloadLength);
            }
        } else { // MSPv2
            if (payloadLength < MIN_LENGTH_REQUEST_V2) {
                return false;   // prevent analyzing garbage data
            }
            requestPacket.flags = payload[MSP_INDEX_FLAG_V2];
            requestPacket.cmd = *(uint16_t*)&payload[MSP_INDEX_ID_LO];
            mspPayloadSize = *(uint16_t*)&payload[MSP_INDEX_SIZE_V2_LO];
            sbufInit(&sbufInput, payload + MSP_INDEX_PAYLOAD_V2, payload + payloadLength);
        }
        if (mspPayloadSize <= sizeof(requestBuffer)) { // prevent buffer overrun
            requestPacket.result = 0;
            requestPacket.buf.ptr = requestBuffer;
            requestPacket.buf.end = requestBuffer + mspPayloadSize;
            mspStarted = 1;
        } else { // this MSP packet is too big to fit in the buffer.
            sendMspErrorResponse(TELEMETRY_MSP_REQUEST_IS_TOO_BIG, requestPacket.cmd);
            return true;
        }
    } else { // second onward chunk
        if (!mspStarted) { // no start packet yet, throw this one away
            return false;
        } else {
            if (((lastSeq + 1) & MSP_STATUS_SEQUENCE_MASK) != seqNumber) {
                // packet loss detected!
                mspStarted = 0;
                return false;
            }
        }
        sbufInit(&sbufInput, payload + 1, payload + payloadLength);
    }

    lastSeq = seqNumber;

    const int payloadExpecting = sbufBytesRemaining(&requestPacket.buf);
    const int payloadIncoming = sbufBytesRemaining(&sbufInput);

    if (payloadExpecting > payloadIncoming) {
        sbufWriteData(&requestPacket.buf, sbufInput.ptr, payloadIncoming);
        sbufAdvance(&sbufInput, payloadIncoming);
        return false;
    } else { // this is the last/only chunk
        if (payloadExpecting) {
            sbufWriteData(&requestPacket.buf, sbufInput.ptr, payloadExpecting);
            sbufAdvance(&sbufInput, payloadExpecting);
        }
    }

    // Skip a few telemetry requests if command is MSP_EEPROM_WRITE
    if (requestPacket.cmd == MSP_EEPROM_WRITE && skipsBeforeResponse) {
        *skipsBeforeResponse = TELEMETRY_REQUEST_SKIPS_AFTER_EEPROMWRITE;
    }

    mspStarted = 0;
    sbufSwitchToReader(&requestPacket.buf, requestBuffer);
    processMspPacket();
    return true;
}

bool sendMspReply(const uint8_t payloadSizeMax, mspResponseFnPtr responseFn)
{
    static uint8_t seq = 0;

    uint8_t payloadArray[payloadSizeMax];
    sbuf_t payloadBufStruct;
    sbuf_t *payloadBuf = sbufInit(&payloadBufStruct, payloadArray, payloadArray + payloadSizeMax);

    // detect first reply packet
    if (responsePacket.buf.ptr == responseBuffer) {
        // this is the first frame of the response packet. Add proper header and size.
        // header
        uint8_t status = MSP_STATUS_START_MASK | (seq++ & MSP_STATUS_SEQUENCE_MASK) | (lastRequestVersion << MSP_STATUS_VERSION_SHIFT);
        if (responsePacket.result < 0) {
            status |= MSP_STATUS_ERROR_MASK;
        }
        sbufWriteU8(payloadBuf, status);

        const int size = sbufBytesRemaining(&responsePacket.buf);  // size might be bigger than 0xff
        if (lastRequestVersion == 1) { // MSPv1
            if (size >= 0xff) {
                // Sending Jumbo-frame
                sbufWriteU8(payloadBuf, 0xff);
                sbufWriteU8(payloadBuf, responsePacket.cmd);
                sbufWriteU16(payloadBuf, (uint16_t)size);
            } else {
                sbufWriteU8(payloadBuf, size);
                sbufWriteU8(payloadBuf, responsePacket.cmd);
            }
        } else { // MSPv2
            sbufWriteU8 (payloadBuf, responsePacket.flags);  // MSPv2 flags
            sbufWriteU16(payloadBuf, responsePacket.cmd);    // command is 16 bit in MSPv2
            sbufWriteU16(payloadBuf, (uint16_t)size);        // size is 16 bit in MSPv2
        }
    } else {
        sbufWriteU8(payloadBuf, (seq++ & MSP_STATUS_SEQUENCE_MASK) | (lastRequestVersion << MSP_STATUS_VERSION_SHIFT)); // header without 'start' flag
    }

    const int inputRemainder = sbufBytesRemaining(&responsePacket.buf);// size might be bigger than 0xff
    const int chunkRemainder = sbufBytesRemaining(payloadBuf); // free space remainder for current chunk

    if (inputRemainder >= chunkRemainder) {
        // partial send
        sbufWriteData(payloadBuf, responsePacket.buf.ptr, chunkRemainder);
        sbufAdvance(&responsePacket.buf, chunkRemainder);
        responseFn(payloadArray, payloadSizeMax);
        return true;
    }
    // last/only chunk
    sbufWriteData(payloadBuf, responsePacket.buf.ptr, inputRemainder);
    sbufAdvance(&responsePacket.buf, inputRemainder);
    sbufSwitchToReader(&responsePacket.buf, responseBuffer);// for CRC calculation

    responseFn(payloadArray, payloadBuf->ptr - payloadArray);
    return false;
}

#endif
