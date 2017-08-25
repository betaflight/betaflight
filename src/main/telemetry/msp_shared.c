#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#ifdef TELEMETRY

#include "common/utils.h"

#include "fc/fc_msp.h"

#include "msp/msp.h"

#include "rx/msp.h"

#include "telemetry/msp_shared.h"

#define TELEMETRY_MSP_VERSION    1
#define TELEMETRY_MSP_VER_SHIFT  5
#define TELEMETRY_MSP_VER_MASK   (0x7 << TELEMETRY_MSP_VER_SHIFT)
#define TELEMETRY_MSP_ERROR_FLAG (1 << 5)
#define TELEMETRY_MSP_START_FLAG (1 << 4)
#define TELEMETRY_MSP_SEQ_MASK   0x0F
#define TELEMETRY_MSP_RES_ERROR (-10)

enum {
    TELEMETRY_MSP_VER_MISMATCH=0,
    TELEMETRY_MSP_CRC_ERROR=1,
    TELEMETRY_MSP_ERROR=2
};

static void initMspResponse(int16_t cmd, mspPackage_t *package)
{
    mspPacket_t *packet = package->responsePacket;
    packet->buf.ptr = package->responseBuffer;
    packet->buf.end = ARRAYEND(package->responseBuffer);
    packet->cmd = cmd;
    packet->result = 0;
}

static void processMspPacket(mspPackage_t *package)
{
    initMspResponse(0, package);

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    if (mspFcProcessCommand(package->requestPacket, package->responsePacket, &mspPostProcessFn) == MSP_RESULT_ERROR) {
        sbufWriteU8(&package->responsePacket->buf, TELEMETRY_MSP_ERROR);
    }
    if (mspPostProcessFn) {
        mspPostProcessFn(NULL);
    }

    sbufSwitchToReader(&package->responsePacket->buf, package->responseBuffer);
}

void sendMspErrorResponse(mspPackage_t *package, uint8_t error, int16_t cmd)
{
    initMspResponse(cmd, package);
    sbufWriteU8(&package->responsePacket->buf, error);
    package->responsePacket->result = TELEMETRY_MSP_RES_ERROR;
    sbufSwitchToReader(&package->responsePacket->buf, package->responseBuffer);
}

bool handleMspFrame(mspPackage_t *package)
{
    static uint8_t mspStarted = 0;
    static uint8_t lastSeq = 0;
    static uint8_t checksum = 0;

    sbuf_t *frameBuf = &package->requestFrame;
    mspPacket_t *packet = package->requestPacket;
    uint8_t header = sbufReadU8(frameBuf);
    uint8_t seqNumber = header & TELEMETRY_MSP_SEQ_MASK; 
    uint8_t version = (header & TELEMETRY_MSP_VER_MASK) >> TELEMETRY_MSP_VER_SHIFT;
    
    if (version != TELEMETRY_MSP_VERSION) {
        sendMspErrorResponse(package, TELEMETRY_MSP_VER_MISMATCH, 0);
        return true;
    }

    if (header & TELEMETRY_MSP_START_FLAG) {
        // first packet in sequence
        uint8_t mspPayloadSize = sbufReadU8(frameBuf);
        
        packet->cmd = sbufReadU8(frameBuf);
        packet->result = 0;
        packet->buf.ptr = package->requestBuffer;
        packet->buf.end = package->requestBuffer + mspPayloadSize;

        checksum = mspPayloadSize ^ packet->cmd;
        mspStarted = 1;
    } else if (!mspStarted) {
        // no start packet yet, throw this one away
        return false;
    } else if (((lastSeq + 1) & TELEMETRY_MSP_SEQ_MASK) != seqNumber) {
        // packet loss detected!
        mspStarted = 0;
        return false;
    }

    while ((frameBuf->ptr < frameBuf->end) && sbufBytesRemaining(&packet->buf)) {
        checksum ^= *frameBuf->ptr;
        sbufWriteU8(&packet->buf, sbufReadU8(frameBuf));
    }

    if (frameBuf->ptr == frameBuf->end) {
        lastSeq = seqNumber;
        return false;
    }

    if (checksum != *frameBuf->ptr) {
        mspStarted = 0;
        sendMspErrorResponse(package, TELEMETRY_MSP_CRC_ERROR, packet->cmd);
        return true;
    }

    mspStarted = 0;
    sbufSwitchToReader(&packet->buf, package->requestBuffer);
    processMspPacket(package);
    return true;
}

bool sendMspReply(mspPackage_t *package, uint8_t payloadSize, mspResponseFnPtr responseFn)
{
    static uint8_t checksum = 0;
    static uint8_t seq = 0;

    uint8_t packet[payloadSize];
    uint8_t *p = packet;
    uint8_t *end = p + payloadSize;

    sbuf_t *txBuf = &package->responsePacket->buf;

    // detect first reply packet
    if (txBuf->ptr == package->responseBuffer) {

        // header
        uint8_t head = TELEMETRY_MSP_START_FLAG | (seq++ & TELEMETRY_MSP_SEQ_MASK);
        if (package->responsePacket->result < 0) {
            head |= TELEMETRY_MSP_ERROR_FLAG;
        }
        *p++ = head;

        uint8_t size = sbufBytesRemaining(txBuf);
        *p++ = size;

        checksum = size ^ package->responsePacket->cmd;
    }
    else {
        // header
        *p++ = (seq++ & TELEMETRY_MSP_SEQ_MASK);
    }

    while ((p < end) && (sbufBytesRemaining(txBuf) > 0)) {
        *p = sbufReadU8(txBuf);
        checksum ^= *p++; // MSP checksum
    }

    // to be continued...
    if (p == end) {
        responseFn(packet);
        return true;
    }

    // nothing left in txBuf,
    // append the MSP checksum
    *p++ = checksum;

    // pad with zeros
    while (p < end)
        *p++ = 0;
        responseFn(packet);
    return false;
}

#endif