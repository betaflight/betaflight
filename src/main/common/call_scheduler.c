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

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"
#include "common/utils.h"
#include "drivers/time.h"
#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "common/call_scheduler.h"

#define MAX_SCHEDULED_CALLS 128

typedef void (*ScheduledFunction)(void);

typedef struct ScheduledCall_s {
    ScheduledFunction function; // Function pointer to be called
    timeUs_t callTimeUs;        // Execution time
    uint32_t id;
} ScheduledCall_t;

ScheduledCall_t scheduledCalls[MAX_SCHEDULED_CALLS];
ScheduledCall_t bufferedCalls[MAX_SCHEDULED_CALLS];
int scheduledCallsCount = 0;
int bufferedCallsCount = 0;
uint32_t totalCalls = 0;
uint32_t refusedCalls = 0;

void sortScheduledCallsLast(void)
{ // sorts the last element, assumes scheduledCalls is already sorted except the element sceduledCallsCount
    if (scheduledCallsCount == 0) {
        // No sorting needed, just increase the count
        scheduledCallsCount++;
        return;
    }

    // Store the last element which needs to be sorted
    ScheduledCall_t newCall = scheduledCalls[scheduledCallsCount];

    int i;
    // Start from the end of the sorted part of the array
    for (i = scheduledCallsCount - 1; (i >= 0) && (scheduledCalls[i].callTimeUs > newCall.callTimeUs); i--) {
        // Shift elements forward to make room for the new element
        scheduledCalls[i + 1] = scheduledCalls[i];
    }

    // Place the new element into the correct position
    scheduledCalls[i + 1] = newCall;
    // Now we increment the count of scheduled calls
    scheduledCallsCount++;
}

uint32_t scheduleCall(ScheduledFunction func, timeUs_t delayUs)
{ // returns 0 if not enough memory for a call, otherwize returns the call ID
    totalCalls ++;

    if (totalCalls == 0) {
        totalCalls = 1; // in case of value overflow
    }

    if (scheduledCallsCount + bufferedCallsCount >= MAX_SCHEDULED_CALLS) {
        refusedCalls ++;
        return 0;
    }

    bufferedCalls[bufferedCallsCount].callTimeUs = micros() + delayUs;
    bufferedCalls[bufferedCallsCount].function = func;
    bufferedCalls[bufferedCallsCount].id = totalCalls;
    bufferedCallsCount ++;
    return totalCalls;
}

void callSchedulerUpdate(timeUs_t currentTimeUs)
{
    int i = 0;
    // Call all functions whose time is due (callTimeUs <= currentTimeUs)
    while (i < scheduledCallsCount && scheduledCalls[i].callTimeUs <= currentTimeUs) {
        scheduledCalls[i].function(); // Call the scheduled function
        i++;  // Move to the next scheduled call
    }

    // If any functions were called, we need to shift the remaining elements
    if (i > 0) {
        // Move the remaining elements forward to fill the gap
        for (int j = 0; j < scheduledCallsCount - i; j++) {
            scheduledCalls[j] = scheduledCalls[j + i];
        }

        // Reduce the count of scheduled calls
        scheduledCallsCount -= i;
    }

    // Merge buffered calls
    for (int i = 0; i < bufferedCallsCount; i++) {
        scheduledCalls[scheduledCallsCount] =  bufferedCalls[i];
        sortScheduledCallsLast();        
    }

    bufferedCallsCount = 0;    

    DEBUG_SET(DEBUG_CALL_SCHEDULER, 0, lrintf(scheduledCallsCount));
    DEBUG_SET(DEBUG_CALL_SCHEDULER, 1, lrintf(refusedCalls));
    DEBUG_SET(DEBUG_CALL_SCHEDULER, 2, lrintf(totalCalls));
}
