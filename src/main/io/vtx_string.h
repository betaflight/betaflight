#pragma once

#include <stdint.h>

#include "platform.h"
#include "drivers/vtx_common.h"


extern const uint16_t vtx58frequencyTable[VTX_SETTINGS_BAND_COUNT][VTX_SETTINGS_CHANNEL_COUNT];
extern const char * const vtx58BandNames[];
extern const char * const vtx58ChannelNames[];
extern const char vtx58BandLetter[];

bool vtx58_Freq2Bandchan(uint16_t freq, uint8_t *pBand, uint8_t *pChannel);
uint16_t vtx58_Bandchan2Freq(uint8_t band, uint8_t channel);
