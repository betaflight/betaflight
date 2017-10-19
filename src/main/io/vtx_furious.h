#pragma once

#define VTX_FURIOUS_POWER_COUNT 4
extern const uint16_t furiousPowerTable[VTX_FURIOUS_POWER_COUNT];
extern const char * const furiousPowerNames[VTX_FURIOUS_POWER_COUNT+1];

extern uint8_t furiousBand;
extern uint8_t furiousChannel;
extern uint16_t furiousPower;           // Actual transmitting power
extern uint8_t furiousPitMode;
extern uint32_t furiousCurFreq;
extern uint16_t furiousConfiguredPower; // Configured transmitting power
extern int16_t furiousTemperature;
extern uint8_t furiousHardwareVersion;
extern uint8_t furiousFirmwareVersion;

bool vtxFuriousInit();
bool furiousCommitChanges();
void furiousSetPitMode(uint8_t onoff);
void furiousSetBandAndChannel(uint8_t band, uint8_t channel);
void furiousSetRFPower(uint16_t level);
