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

typedef struct pgRegistry_s {
    // Base of the group in RAM.
    void *base;
    // Size of the group in RAM.
    uint16_t size;
    // The parameter group number.
    uint16_t pgn;
    // The in-memory format number.  Bump when making incompatible
    // changes to the PG.
    uint8_t format;
} pgRegistry_t;

#define PG_REGISTRY_SECTION __attribute__((section(".pg_registry"), used))
#define PG_REGISTRY_TAIL_SECTION __attribute__((section(".pg_registry_tail"), used))

#define PG_PACKED __attribute__((packed))

const pgRegistry_t* pgFind(uint8_t id);
const pgRegistry_t* pgFindForSet(uint8_t id);
bool pgLoad(const pgRegistry_t* block, const void *from, uint8_t size);
