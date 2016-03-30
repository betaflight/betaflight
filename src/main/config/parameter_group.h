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

#pragma once

typedef uint16_t pgn_t;

// parameter group registry flags
typedef enum {
    PGRF_NONE = 0,
    PGRF_CLASSIFICATON_BIT = (1 << 0),
} pgRegistryFlags_e;

// parameter group classification
typedef enum {
    PGC_SYSTEM = 0,
    PGC_PROFILE = 1,
} pgClassification_e;

typedef struct pgRegistry_s {
    // Base of the group in RAM.
    uint8_t *base;
    // The pointer to update after loading the record into ram.
    uint8_t **ptr;
    // Size of the group in RAM.
    uint16_t size;
    // The parameter group number.
    pgn_t pgn;
    // The in-memory format number.  Bump when making incompatible
    // changes to the PG.
    uint8_t format;
    // see pgRegistryFlags_e
    uint8_t flags;
} pgRegistry_t;

#ifdef UNIT_TEST
#define PG_REGISTRY_SECTION
#define PG_REGISTRY_TAIL_SECTION
#else
#define PG_REGISTRY_SECTION __attribute__((section(".pg_registry"), used))
#define PG_REGISTRY_TAIL_SECTION __attribute__((section(".pg_registry_tail"), used))
#endif

#define PG_PACKED __attribute__((packed))

extern const pgRegistry_t __pg_registry[];

// Helper to iterate over the PG register.  Cheaper than a visitor style callback.
#define PG_FOREACH(_name) \
    for (const pgRegistry_t *(_name) = __pg_registry; (_name)->base != NULL; (_name)++)

typedef uint8_t (*pgMatcherFuncPtr)(const pgRegistry_t *candidate, const void *criteria);

const pgRegistry_t* pgFind(pgn_t pgn);
const pgRegistry_t* pgMatcher(pgMatcherFuncPtr matcher, const void *criteria);
void pgLoad(const pgRegistry_t* reg, const void *from, int size);
void pgResetAll(uint8_t profileCount);

