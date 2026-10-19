/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_FRAME_WIZARD) || defined(USE_FILTER_AUTOTUNE)

#include "bench_scratch.h"

static uint32_t scratch[(BENCH_SCRATCH_BYTES + 3) / 4];
static benchScratchOwner_e owner;

void *benchScratchClaim(benchScratchOwner_e newOwner)
{
    if (owner != newOwner) {
        memset(scratch, 0, sizeof(scratch));
        owner = newOwner;
    }
    return scratch;
}

bool benchScratchOwnedBy(benchScratchOwner_e o)
{
    return owner == o;
}

#endif
