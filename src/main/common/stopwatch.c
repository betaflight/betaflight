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

#include "platform.h"

#include "common/time.h"

#include "drivers/system.h"

#include "stopwatch.h"

void stopwatchInit(stopwatch_t *watch)
{
    watch->start = 0;
    watch->stop = 0;
    stopwatchReset(watch);
}

void stopwatchReset(stopwatch_t *watch)
{
    watch->isRunning = false;
    watch->elapsed = 0;
}

uint32_t stopwatchStart(stopwatch_t *watch)
{
    if (watch->isRunning == false) {
        watch->isRunning = true;
        watch->start = getCycleCounter();
    }
    return watch->start;
}

uint32_t stopwatchStop(stopwatch_t *watch)
{
    if (watch->isRunning == true) {
        watch->stop = getCycleCounter();
        watch->elapsed += cmpTimeCycles(watch->stop, watch->start);
        watch->isRunning = false;
    }
    return watch->stop;
}

uint32_t stopwatchGetCycles(stopwatch_t *watch)
{
    if (watch->isRunning == true) {
        return watch->elapsed + cmpTimeCycles(getCycleCounter(), watch->start);
    } else {
        return watch->elapsed;
    }
}

uint32_t stopwatchGetMicros(stopwatch_t *watch)
{
    return clockCyclesToMicros(stopwatchGetCycles(watch));
}

float stopwatchGetMicrosf(stopwatch_t *watch)
{
    return clockCyclesToMicrosf(stopwatchGetCycles(watch));
}
