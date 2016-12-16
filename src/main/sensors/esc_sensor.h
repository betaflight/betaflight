#pragma once

uint8_t escSensorFrameStatus(void);
bool escSensorInit(void);
bool isEscSensorActive(void);
int16_t getEscSensorVbat(void);
int16_t getEscSensorCurrent(void);
int16_t getEscSensorConsumption(void);

void escSensorProcess(uint32_t currentTime);
