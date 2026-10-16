/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// Shared NVIC-to-PLIC interrupt priority conversion for HPMicro.
#ifndef NVIC_HPMICRO_H
#define NVIC_HPMICRO_H

#include <stdint.h>

#include "common/utils.h"

/**
 * Convert a Betaflight NVIC-encoded priority to an HPM PLIC priority.
 *
 * Betaflight encodes 16 priority ranks in bits 7:4 with lower values more
 * urgent; HPM6300/HPM6700 PLIC priorities range from 1..7 in the opposite
 * order (0 disables the interrupt).  NVIC_PRIORITY_BASE() cannot be used
 * here: with this platform's NVIC_PriorityGroup_2 grouping it returns only
 * the 2-bit preemption field (0..3), collapsing all 16 ranks onto two PLIC
 * levels.  Ranking the full encoded byte preserves the relative order of
 * every NVIC_PRIO_* constant.
 */
static inline uint32_t hpmPlicPriorityFromNvic(uint32_t nvicPriority)
{
    const uint32_t nvicRank = nvicPriority >> 4;

    return MAX(1U, 7U - (nvicRank >> 1));
}

#endif // NVIC_HPMICRO_H
