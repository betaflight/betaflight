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

typedef struct pgRegistry_s {
    // Base of the group in RAM.
    void *base;
    // Size of the group in RAM.
    uint16_t size;
    // The parameter group number.
    pgn_t pgn;
    // The in-memory format number.  Bump when making incompatible
    // changes to the PG.
    uint8_t format;
} pgRegistry_t;

#define PG_REGISTRY_SECTION __attribute__((section(".pg_registry"), used))
#define PG_REGISTRY_TAIL_SECTION __attribute__((section(".pg_registry_tail"), used))

#define PG_PACKED __attribute__((packed))

extern const pgRegistry_t __pg_registry[];

// Helper to iterate over the PG register.  Cheaper than a visitor style callback.
#define PG_FOREACH(_name) \
    for (const pgRegistry_t *(_name) = __pg_registry; (_name)->base != NULL; (_name)++)

typedef uint8_t (*pgMatcherFuncPtr)(const pgRegistry_t *candidate, const void *criteria);

const pgRegistry_t* pgFind(pgn_t pgn);
const pgRegistry_t* pgMatcher(pgMatcherFuncPtr func, const void *criteria);
void pgLoad(const pgRegistry_t* reg, const void *from, int size);
void pgResetAll(void);

