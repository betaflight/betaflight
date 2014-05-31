#pragma once

void systemInit(bool overclock);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

uint32_t micros(void);
uint32_t millis(void);

// failure
void failureMode(uint8_t mode);

// bootloader/IAP
void systemReset(bool toBootloader);

// current crystal frequency - 8 or 12MHz
extern uint32_t hse_value;
