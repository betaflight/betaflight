#pragma once

#define ESC_TRIGGER_NONE 99

uint8_t escTelemetryFrameStatus(void);
bool escTelemetryInit(void);
bool isEscTelemetryEnabled(void);
uint8_t getEscTelemetryTriggerMotorIndex(void);

void escTelemetryProcess(uint32_t currentTime);
