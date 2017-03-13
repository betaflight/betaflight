#pragma once

#include <stdint.h>

#if defined(VTX_COMMON)

extern const uint16_t vtx58FreqTable[5][8];
extern const char * const vtx58BandNames[];
extern const char * const vtx58ChannelNames[];
extern const char vtx58BandLetter[];

bool vtx58_Freq2Bandchan(uint16_t freq, uint8_t *pBand, uint8_t *pChan);

#endif
