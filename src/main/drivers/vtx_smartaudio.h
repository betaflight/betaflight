
// For generic API use, but here for now

#pragma once

typedef struct vtxPowerTable_s {
    char    *name;
    int16_t power;
    int16_t value;
} vtxPowerTable_t;

void smartAudioInit(void);
void smartAudioProcess(uint32_t);
void smartAudioSetPowerByIndex(uint8_t);
void smartAudioSetFreq(uint16_t);
void smartAudioSetBandChan(int, int);

#ifdef OSD
#include "io/osd_menutypes.h"
extern OSD_dynaTAB_t smartAudioOsdStatusTable;
extern OSD_dynaTAB_t smartAudioOsdPowerTable;
#endif
