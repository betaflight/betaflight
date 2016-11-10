#pragma once

uint8_t escTelemetryFrameStatus(void);
bool escTelemetryInit(void);
bool isEscTelemetryActive(void);
uint16_t getEscTelemetryVbat(void);
uint16_t getEscTelemetryCurrent(void);
uint16_t getEscTelemetryConsumption(void);
bool escTelemetrySendTrigger(uint8_t index);

void escTelemetryProcess(uint32_t currentTime);
