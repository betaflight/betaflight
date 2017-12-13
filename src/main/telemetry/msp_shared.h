#pragma once

#include "common/streambuf.h"
#include "telemetry/crsf.h"
#include "telemetry/smartport.h"

typedef void (*mspResponseFnPtr)(uint8_t *payload);

struct mspPacket_s;
typedef struct mspPackage_s {
    sbuf_t requestFrame;
    uint8_t *requestBuffer;
    uint8_t *responseBuffer;
    struct mspPacket_s *requestPacket;
    struct mspPacket_s *responsePacket;
} mspPackage_t;

typedef union mspRxBuffer_u {
    uint8_t smartPortMspRxBuffer[SMARTPORT_MSP_RX_BUF_SIZE];
    uint8_t crsfMspRxBuffer[CRSF_MSP_RX_BUF_SIZE];
} mspRxBuffer_t;

typedef union mspTxBuffer_u {
    uint8_t smartPortMspTxBuffer[SMARTPORT_MSP_TX_BUF_SIZE];
    uint8_t crsfMspTxBuffer[CRSF_MSP_TX_BUF_SIZE];
} mspTxBuffer_t;

void initSharedMsp(void);
bool handleMspFrame(uint8_t *frameStart, int frameLength);
bool sendMspReply(uint8_t payloadSize, mspResponseFnPtr responseFn);
