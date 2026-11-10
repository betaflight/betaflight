/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

/* PFIC compares all four implemented priority bits against ITHRESDR.
 * Give each of BF's four preemption groups one IRQ priority and a separate,
 * strictly higher priority threshold. No IRQ uses zero or a threshold value:
 *
 * BF group       0     1     2     3
 * IRQ priority   0x20  0x60  0xA0  0xE0
 * Mask threshold 0x10  0x50  0x90  0xD0
 *
 * Subpriorities collapse within each group. This preserves preemption order
 * and lets ATOMIC_BLOCK mask a whole group without depending on whether the
 * PFIC's threshold comparison includes equality. Zero remains "unmasked".
 */
#define NVIC_PRIORITY_GROUPING 0x500
#define NVIC_BUILD_PRIORITY(base, sub) ((((base) > 3U ? 3U : (base)) << 6) | 0x20U)
#define NVIC_PRIORITY_BASE(priority) (((priority) & 0xC0U) >> 6)
#define NVIC_PRIORITY_SUB(priority) 0U

static inline uint32_t ch32AtomicPriority(uint32_t basePri)
{
    const uint32_t priority = basePri & 0xF0U;
    if (priority == 0) {
        return 0;
    }
    return (priority & 0xC0U) | 0x10U;
}
