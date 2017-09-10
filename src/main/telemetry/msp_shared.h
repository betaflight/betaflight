#pragma once

#include "msp/msp.h"
#include "rx/crsf.h"
#include "telemetry/smartport.h"

typedef void (*mspResponseFnPtr)(uint8_t *payload);

typedef struct mspPackage_s {
    sbuf_t requestFrame;
    uint8_t *requestBuffer;
    uint8_t *responseBuffer;
    mspPacket_t *requestPacket;
    mspPacket_t *responsePacket;
} mspPackage_t;

typedef union mspRxBuffer_u {
    uint8_t smartPortMspRxBuffer[SMARTPORT_MSP_RX_BUF_SIZE];
    uint8_t crsfMspRxBuffer[CRSF_MSP_RX_BUF_SIZE];
} mspRxBuffer_t;

typedef union mspTxBuffer_u {
    uint8_t smartPortMspTxBuffer[SMARTPORT_MSP_TX_BUF_SIZE];
    uint8_t crsfMspTxBuffer[CRSF_MSP_TX_BUF_SIZE];
} mspTxBuffer_t;

typedef enum mspFrameHandling_e {
    MSP_FRAME_HANDLING_NORMAL,
    MSP_FRAME_HANDLING_FORCED
} mspFrameHandling_t;

void initSharedMsp();
bool handleMspFrame(uint8_t *frameStart, uint8_t *frameEnd, mspFrameHandling_t handling);
bool sendMspReply(uint8_t payloadSize, mspResponseFnPtr responseFn);
