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

#include "common/utils.h"

#include "drivers/system.h"
#include "fc/fc_dispatch.h"

static dispatchTask_t *head = NULL;

void dispatchProcess(uint32_t currentTime)
{
    if (!head || currentTime < head->delayedUntil) {
        return;
    }

    dispatchTask_t *current = head;
    dispatchTask_t *previous = NULL;
    while (current && current->delayedUntil < currentTime) {
        if (current->ptr) {
            (*current->ptr)();
        }

        /* remove item from list */
        if (previous) {
            previous->next = current->next;
        } else {
            head = current->next;
        }     
        current->delayedUntil = 0;
        current = current->next;
    }
}

void dispatchAdd(dispatchTask_t *task)
{
    if (!task || task->delayedUntil) {
        /* invalid or already in the list */
        return;
    }

    task->next = NULL;
    task->delayedUntil = micros() + task->minimumDelayUs;

    if (!head) {
        head = task;
        return;
    }

    if (task->delayedUntil < head->delayedUntil) {
        task->next = head;
        head = task;
        return;
    }

    dispatchTask_t *pos = head;
    while (pos->next && pos->next->delayedUntil < task->delayedUntil) {
        pos = pos->next;
    }
    task->next = pos->next;
    pos->next = task;
}
