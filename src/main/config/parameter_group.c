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

#include "parameter_group.h"
#include "common/maths.h"

const pgRegistry_t* pgFind(pgn_t pgn)
{
    PG_FOREACH(reg) {
        if (reg->pgn == pgn) {
            return reg;
        }
    }
    return NULL;
}

const pgRegistry_t* pgFindForSet(pgn_t pgn)
{
    PG_FOREACH(reg) {
        if (reg->pgn_for_set == pgn) {
            return reg;
        }
    }
    return NULL;
}

void pgLoad(const pgRegistry_t* reg, const void *from, int size)
{
    memset(reg->base, 0, reg->size);
    int take = MIN(size, reg->size);
    memcpy(reg->base, from, take);
}
