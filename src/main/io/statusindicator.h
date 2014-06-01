#pragma once

void blinkLedAndSoundBeeper(uint8_t num, uint8_t wait, uint8_t repeat);

void enableWarningLed(uint32_t currentTime);
void disableWarningLed(void);
void updateWarningLed(uint32_t currentTime);
