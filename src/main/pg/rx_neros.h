#pragma once

#include "drivers/io_types.h"

#include "pg/pg.h"

#define MAX_PHRASE_LENGTH 16u

typedef struct nelrsConfig_s {
    char bindPhraseLow[MAX_PHRASE_LENGTH + 1];
    char bindPhraseHigh[MAX_PHRASE_LENGTH + 1];
} nelrsConfig_t;

PG_DECLARE(nelrsConfig_t, nelrsConfig);
