/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include <platform.h>

#include "build_config.h"

#include "common/maths.h"

#include "drivers/system.h"

#include "config/parameter_group.h"
#include "config/config_streamer.h"

static const uint8_t EEPROM_CONF_VERSION = 111;

#if !defined(FLASH_SIZE)
#error "Flash size not defined for target. (specify in KB)"
#endif

#ifndef FLASH_PAGE_SIZE
    #ifdef STM32F303xC
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif

    #ifdef STM32F10X_MD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif
#endif

#if !defined(FLASH_SIZE) && !defined(FLASH_PAGE_COUNT)
    #ifdef STM32F10X_MD
        #define FLASH_PAGE_COUNT 128
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_COUNT 128
    #endif
#endif

#if defined(FLASH_SIZE)
#define FLASH_PAGE_COUNT ((FLASH_SIZE * 0x400) / FLASH_PAGE_SIZE)
#endif

#if !defined(FLASH_PAGE_SIZE)
#error "Flash page size not defined for target."
#endif

#if !defined(FLASH_PAGE_COUNT)
#error "Flash page count not defined for target."
#endif

#if FLASH_SIZE <= 64
#define FLASH_TO_RESERVE_FOR_CONFIG 0x0800
#else
#define FLASH_TO_RESERVE_FOR_CONFIG 0x1000
#endif

extern uint8_t __config_start; // configured via linker script when building binaries.

// Header for the saved copy.
typedef struct {
    uint8_t format;
} PG_PACKED configHeader_t;

// Header for each stored PG.
typedef struct {
    // TODO(michaelh): shrink to uint8_t once masterConfig has been
    // split up.
    uint16_t size;
    uint8_t pgn;
    uint8_t format;
    uint8_t pg[];
} PG_PACKED configRecord_t;

// Footer for the saved copy.
typedef struct {
    uint16_t terminator;
    uint8_t chk;
} PG_PACKED configFooter_t;

// Used to check the compiler packing at build time.
typedef struct {
    uint8_t byte;
    uint32_t word;
} PG_PACKED packingTest_t;



void initEEPROM(void)
{
    // Verify that this architecture packs as expected.
    BUILD_BUG_ON(offsetof(packingTest_t, byte) != 0);
    BUILD_BUG_ON(offsetof(packingTest_t, word) != 1);
    BUILD_BUG_ON(sizeof(packingTest_t) != 5);

    BUILD_BUG_ON(sizeof(configHeader_t) != 1);
    BUILD_BUG_ON(sizeof(configFooter_t) != 3);
    BUILD_BUG_ON(sizeof(configRecord_t) != 4);
}

static uint8_t updateChecksum(uint8_t chk, const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    const uint8_t *pend = p + length;

    for (; p != pend; p++) {
        chk ^= *p;
    }
    return chk;
}

// Find a parameter group by PGN.  Returns NULL on not found.
static const pgRegistry_t *findPGN(const configRecord_t *record)
{
    // To save memory, the array is terminated by a single zero
    // instead of a full pgRegistry_t.  This means that 'base' must be
    // the first entry in the struct.
    BUILD_BUG_ON(offsetof(pgRegistry_t, base) != 0);

    PG_FOREACH(reg) {
        if (reg->pgn == record->pgn && reg->format == record->format) {
            return reg;
        }
    }
    return NULL;
}

// Load a PG into RAM, upgrading and downgrading as needed.
static bool loadPG(const configRecord_t *record)
{
    const pgRegistry_t *reg = findPGN(record);

    if (reg == NULL) {
        return false;
    }

    // Clear the in-memory copy.  Sets any ungraded fields to zero.
    memset(reg->base, 0, reg->size);
    memcpy(reg->base, record->pg, MIN(reg->size, record->size - sizeof(*record)));
    return true;
}

// Scan the EEPROM config.  Optionally also load into memory.  Returns
// true if the config is valid.
bool scanEEPROM(bool andLoad) // FIXME boolean argument.
{
    uint8_t chk = 0;
    const uint8_t *p = &__config_start;
    const configHeader_t *header = (const configHeader_t *)p;

    if (header->format != EEPROM_CONF_VERSION) {
        return false;
    }
    chk = updateChecksum(chk, header, sizeof(*header));
    p += sizeof(*header);

    for (;;) {
        const configRecord_t *record = (const configRecord_t *)p;

        if (record->size == 0) {
            // Found the end.  Stop scanning.
            break;
        }
        if (record->size >= FLASH_TO_RESERVE_FOR_CONFIG
            || record->size < sizeof(*record)) {
            // Too big or too small.
            return false;
        }

        chk = updateChecksum(chk, p, record->size);

        if (andLoad) {
            loadPG(record);
        }
        p += record->size;
    }

    const configFooter_t *footer = (const configFooter_t *)p;
    chk = updateChecksum(chk, footer, sizeof(*footer));
    return chk == 0xFF;
}

bool isEEPROMContentValid(void)
{
    return scanEEPROM(false);
}

void writeConfigToEEPROM(void)
{
    config_streamer_t streamer;
    config_streamer_init(&streamer);

    // write it
    for (int attempt = 0; attempt < 3; attempt++) {
        config_streamer_start(&streamer, (uintptr_t)&__config_start);

        configHeader_t header = {
            .format = EEPROM_CONF_VERSION,
        };

        config_streamer_write(&streamer, &header, sizeof(header));

        PG_FOREACH(reg) {
            configRecord_t record = {
                .size = sizeof(configRecord_t) + reg->size,
                .pgn = reg->pgn,
                .format = reg->format,
            };

            config_streamer_write(&streamer, &record, sizeof(record));
            config_streamer_write(&streamer, reg->base, reg->size);
        }

        configFooter_t footer = {
            .terminator = 0,
            .chk = ~config_streamer_chk(&streamer),
        };

        if (config_streamer_write(&streamer, &footer, sizeof(footer)) == 0) {
            break;
        }
    }

    // Flash write failed - just die now
    if (config_streamer_finish(&streamer) != 0 || !isEEPROMContentValid()) {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
    }
}
