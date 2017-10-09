#pragma once

#include <stdint.h>

#define VTX_TRAMP_POWER_COUNT 5
extern const uint16_t trampPowerTable[VTX_TRAMP_POWER_COUNT];
extern const char * const trampPowerNames[VTX_TRAMP_POWER_COUNT+1];

extern uint8_t trampBand;
extern uint8_t trampChannel;
extern uint16_t trampPower;       // Actual transmitting power
extern uint8_t trampPitMode;
extern uint32_t trampCurFreq;
extern uint16_t trampConfiguredPower; // Configured transmitting power
extern int16_t trampTemperature;

bool vtxTrampInit(void);
bool trampCommitChanges(void);
void trampSetPitMode(uint8_t onoff);
void trampSetBandAndChannel(uint8_t band, uint8_t channel);
void trampSetRFPower(uint16_t level);
