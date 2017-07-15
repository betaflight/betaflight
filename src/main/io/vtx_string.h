#pragma once

#include <stdint.h>

#include "platform.h"

#if defined(VTX_COMMON)

#define VTX58_BAND_COUNT    9
#define VTX58_CHANNEL_COUNT  8

extern const uint16_t vtx58frequencyTable[VTX58_BAND_COUNT][VTX58_CHANNEL_COUNT];
extern const char * const vtx58BandNames[];
extern const char * const vtx58ChannelNames[];
extern const char vtx58BandLetter[];

bool vtx58_Freq2Bandchan(uint16_t freq, uint8_t *pBand, uint8_t *pChannel);

#endif
