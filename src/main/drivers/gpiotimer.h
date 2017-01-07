
#pragma once

typedef struct gpioTimerConfig_s {
    ioTag_t ioTag;
    uint8_t polarity; // 0 = active low (falling), 1 = active high (rising)
    uint16_t guardTimeMs;
} gpioTimerConfig_t;

bool gpioTimerInit(gpioTimerConfig_t *gpioTimerConfig);
void gpioTimerRearm(uint32_t currentTimeUs);

uint32_t gpioTimerValueMs;
