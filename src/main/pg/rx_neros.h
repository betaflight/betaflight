#pragma once

#include "drivers/io_types.h"

#include "pg/pg.h"

#define MAX_PHRASE_LENGTH 16u

typedef struct nelrsConfig_s {
    char bindPhraseLow[MAX_PHRASE_LENGTH + 1];
    char bindPhraseHigh[MAX_PHRASE_LENGTH + 1];
    uint32_t startFrequencyLow;
    uint32_t midFrequencyLow;
    uint32_t endFrequencyLow;
    uint8_t numChannelsLow;
    uint32_t startFrequencyHigh;
    uint32_t midFrequencyHigh;
    uint32_t endFrequencyHigh;
    uint8_t numChannelsHigh;
    bool thermalCamEnabled;
    bool illuminatorEnabled;
    bool cryptoEnable;
} nelrsConfig_t;

PG_DECLARE(nelrsConfig_t, nelrsConfig);
