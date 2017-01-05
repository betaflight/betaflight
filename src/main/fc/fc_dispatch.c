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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/system.h"
#include "fc/fc_dispatch.h"

#define DISPATCH_QUEUE_SIZE     5

typedef struct {
    dispatchFuncPtr ptr;
    uint32_t delayedUntil;
} dispatchItem_t;

static dispatchItem_t queue[DISPATCH_QUEUE_SIZE];
static int next = 0;

static void dispatchRemove(int index)
{
    if (index == (DISPATCH_QUEUE_SIZE-1)) {
        queue[index].ptr = NULL;
        next = index;
        return;
    }

    for (int i = index; i < DISPATCH_QUEUE_SIZE-1; i++) {
        queue[i].ptr = queue[i+1].ptr;

        if (queue[i].ptr == NULL) {
            next = i;
            break;
        }
        queue[i].delayedUntil = queue[i+1].delayedUntil;
    }
}

void dispatchProcess(void)
{
    if (queue[0].ptr == NULL) {
        return;
    }

    for (int i = 0; i < DISPATCH_QUEUE_SIZE; i++) {
        if (queue[i].ptr == NULL) {
            break;
        }
        if (queue[i].delayedUntil < micros()) {
            (*queue[i].ptr)();
            queue[i].ptr = NULL;
            dispatchRemove(i);
        }
    }
}

void dispatchAdd(dispatchFuncPtr ptr, uint32_t delayUs)
{
    if (next < DISPATCH_QUEUE_SIZE) {
        queue[next].ptr = ptr;
        queue[next].delayedUntil = micros() + delayUs;
        next++;
    }
}
