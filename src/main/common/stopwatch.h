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

#include <stdbool.h>
#include <stdint.h>

typedef struct stopwatch_s {
    bool isRunning;
    uint32_t start;
    uint32_t stop;
    uint32_t elapsed;
} stopwatch_t;

void stopwatchInit(stopwatch_t *watch);
void stopwatchReset(stopwatch_t *watch);
uint32_t stopwatchStart(stopwatch_t *watch);
uint32_t stopwatchStop(stopwatch_t *watch);
uint32_t stopwatchGetCycles(stopwatch_t *watch);
uint32_t stopwatchGetMicros(stopwatch_t *watch);
float stopwatchGetMicrosf(stopwatch_t *watch);
