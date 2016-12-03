#pragma once

#include "drivers/io_types.h" // For ioTag_t

bool rssiSoftPwmInit(void);
void rssiSoftPwmUpdate(uint32_t currentTime);
uint16_t rssiSoftPwmRead(void);

typedef struct rssiSoftPwmConfig_s {
    ioTag_t ioTag;
    uint8_t device;
} rssiSoftPwmConfig_t;

// device
#define RXTYPE_FRSKY_X4R    0
#define RXTYPE_FRSKY_TFR4   1

