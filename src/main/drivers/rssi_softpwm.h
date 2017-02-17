#pragma once

#include "drivers/io_types.h" // For ioTag_t

typedef struct rssiSoftPwmConfig_s {
    ioTag_t ioTag;
    uint8_t minFollow;      // 0 = True duty, 1 = Minimum pulsewidth as duty 1%
    uint8_t monitor;        // Fire debug: 0 = off, 1 = on
} rssiSoftPwmConfig_t;

bool rssiSoftPwmInit(rssiSoftPwmConfig_t *pConfigToUse);
void rssiSoftPwmUpdate(uint32_t currentTime);
bool rssiSoftPwmActive(void);
uint16_t rssiSoftPwmRead(void);
