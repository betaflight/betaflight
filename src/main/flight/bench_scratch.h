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

#pragma once

#include <stdbool.h>
#include <stddef.h>

// One RAM area for the large working buffers of the frame wizard (disarmed only) and the filter autotune (armed
// only): they never run at the same time. A new owner gets it cleared; the old owner's data is gone.
#define BENCH_SCRATCH_BYTES 13184            // the frame wizard tap session (the larger of the two)

typedef enum {
    BENCH_SCRATCH_FREE = 0,
    BENCH_SCRATCH_FRAME_WIZARD,
    BENCH_SCRATCH_FILTER_AUTOTUNE,
} benchScratchOwner_e;

void *benchScratchClaim(benchScratchOwner_e owner);
bool benchScratchOwnedBy(benchScratchOwner_e owner);
