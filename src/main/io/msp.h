
#pragma once

typedef struct mspPacket_s {
    sbuf_t buf;
    int16_t cmd;
    int16_t result;
} mspPacket_t;

extern bool isRebootScheduled;

void mspInit(void);

int mspProcess(mspPacket_t *command, mspPacket_t *reply);
