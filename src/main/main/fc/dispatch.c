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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "fc/dispatch.h"

static dispatchEntry_t *head = NULL;
static bool dispatchEnabled = false;

bool dispatchIsEnabled(void)
{
    return dispatchEnabled;
}

void dispatchEnable(void)
{
    dispatchEnabled = true;
}

void dispatchProcess(uint32_t currentTime)
{
    for (dispatchEntry_t **p = &head; *p; ) {
        if (cmp32(currentTime, (*p)->delayedUntil) < 0)
            break;
        // unlink entry first, so handler can replan self
        dispatchEntry_t *current = *p;
        *p = (*p)->next;
        current->inQue = false;
        (*current->dispatch)(current);
    }
}

void dispatchAdd(dispatchEntry_t *entry, int delayUs)
{
    uint32_t delayedUntil = micros() + delayUs;
    dispatchEntry_t **p = &head;

    if (entry->inQue) {
      return;    // Allready in Queue, abort
    }

    while (*p && cmp32((*p)->delayedUntil, delayedUntil) < 0)
        p = &(*p)->next;

    entry->next = *p;
    entry->delayedUntil = delayedUntil;
    entry->inQue = true;
    *p = entry;
}
