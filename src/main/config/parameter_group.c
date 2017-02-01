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

#include <stddef.h>
#include <string.h>
#include <stdint.h>

#include "platform.h"

#include "parameter_group.h"
#include "common/maths.h"

const pgRegistry_t* pgFind(pgn_t pgn)
{
    PG_FOREACH(reg) {
        if (pgN(reg) == pgn) {
            return reg;
        }
    }
    return NULL;
}

static uint8_t *pgOffset(const pgRegistry_t* reg, uint8_t profileIndex)
{
    const uint16_t regSize = pgSize(reg);

    uint8_t *base = reg->address;
    if (!pgIsSystem(reg)) {
        base += (regSize * profileIndex);
    }
    return base;
}

static void pgResetInstance(const pgRegistry_t *reg, uint8_t *base)
{
    const uint16_t regSize = pgSize(reg);

    memset(base, 0, regSize);
    if (reg->reset.ptr >= (void*)__pg_resetdata_start && reg->reset.ptr < (void*)__pg_resetdata_end) {
        // pointer points to resetdata section, to it is data template
        memcpy(base, reg->reset.ptr, regSize);
    } else if (reg->reset.fn) {
        // reset function, call it
        reg->reset.fn(base, regSize);
    }
}

void pgReset(const pgRegistry_t* reg, int profileIndex)
{
    pgResetInstance(reg, pgOffset(reg, profileIndex));
}

void pgResetCurrent(const pgRegistry_t *reg)
{
    if (pgIsSystem(reg)) {
        pgResetInstance(reg, reg->address);
    } else {
        pgResetInstance(reg, *reg->ptr);
    }
}

bool pgResetCopy(void *copy, pgn_t pgn)
{
    const pgRegistry_t *reg = pgFind(pgn);
    if (reg) {
        pgResetInstance(reg, copy);
        return true;
    }
    return false;
}

void pgLoad(const pgRegistry_t* reg, int profileIndex, const void *from, int size, int version)
{
    pgResetInstance(reg, pgOffset(reg, profileIndex));
    // restore only matching version, keep defaults otherwise
    if (version == pgVersion(reg)) {
        const int take = MIN(size, pgSize(reg));
        memcpy(pgOffset(reg, profileIndex), from, take);
    }
}

int pgStore(const pgRegistry_t* reg, void *to, int size, uint8_t profileIndex)
{
    const int take = MIN(size, pgSize(reg));
    memcpy(to, pgOffset(reg, profileIndex), take);
    return take;
}


void pgResetAll(int profileCount)
{
    PG_FOREACH(reg) {
        if (pgIsSystem(reg)) {
            pgReset(reg, 0);
        } else {
            // reset one instance for each profile
            for (int profileIndex = 0; profileIndex < profileCount; profileIndex++) {
                pgReset(reg, profileIndex);
            }
        }
    }
}

void pgActivateProfile(int profileIndex)
{
    PG_FOREACH(reg) {
        if (!pgIsSystem(reg)) {
            uint8_t *ptr = pgOffset(reg, profileIndex);
            *(reg->ptr) = ptr;
        }
    }
}
