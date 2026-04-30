/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/timer_types.h"
#include "platform/dma.h"

/*
 * Betaflight timer numbers are an internal abstraction. For X32M7 we map them
 * to the available ATIM/GTIMA/GTIMB instances here.
 *
 * The current X32M76x bring-up target exposes a conservative timer pin set:
 * GTIMA4 on PB6-PB9 and GTIMA3 on PC6-PC9. That is enough to close PWM motor
 * output and leaves room for later board-specific expansion.
 */
enum {
    X32_TIMER_ATIM1 = 1,
    X32_TIMER_ATIM2,
    X32_TIMER_GTIMA1,
    X32_TIMER_GTIMA2,
    X32_TIMER_GTIMA3,
    X32_TIMER_GTIMA4,
    X32_TIMER_GTIMA5,
    X32_TIMER_GTIMA6,
    X32_TIMER_GTIMA7,
    X32_TIMER_GTIMB1,
    X32_TIMER_GTIMB2,
    X32_TIMER_GTIMB3,
    X32_TIMER_ATIM3,
    X32_TIMER_ATIM4,
};

#define HARDWARE_TIMER_DEFINITION_COUNT 14
#define FULL_TIMER_CHANNEL_COUNT 8

#define USED_TIMERS ( \
    (1U << X32_TIMER_ATIM1)  | \
    (1U << X32_TIMER_ATIM2)  | \
    (1U << X32_TIMER_GTIMA1) | \
    (1U << X32_TIMER_GTIMA2) | \
    (1U << X32_TIMER_GTIMA3) | \
    (1U << X32_TIMER_GTIMA4) | \
    (1U << X32_TIMER_GTIMA5) | \
    (1U << X32_TIMER_GTIMA6) | \
    (1U << X32_TIMER_GTIMA7) | \
    (1U << X32_TIMER_GTIMB1) | \
    (1U << X32_TIMER_GTIMB2) | \
    (1U << X32_TIMER_GTIMB3) | \
    (1U << X32_TIMER_ATIM3)  | \
    (1U << X32_TIMER_ATIM4))

#define TIMUP_TIMERS USED_TIMERS
