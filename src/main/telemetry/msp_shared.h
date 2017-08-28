#pragma once

#include "msp/msp.h"

typedef void (*mspResponseFnPtr)(uint8_t *packet);

typedef struct mspPackage_s {
    sbuf_t requestFrame;
    uint8_t *requestBuffer;
    uint8_t *responseBuffer;
    mspPacket_t *requestPacket;
    mspPacket_t *responsePacket;
} mspPackage_t;

bool handleMspFrame(mspPackage_t *mspPackage);
bool sendMspReply(mspPackage_t *package, uint8_t payloadSize, mspResponseFnPtr responseFn);
