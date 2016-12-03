#pragma once

#include "common/time.h"

uint8_t escTelemetryFrameStatus(void);
bool escTelemetryInit(void);
bool isEscTelemetryActive(void);
int16_t getEscTelemetryVbat(void);
int16_t getEscTelemetryCurrent(void);
int16_t getEscTelemetryConsumption(void);

void escTelemetryProcess(timeUs_t currentTimeUs);
