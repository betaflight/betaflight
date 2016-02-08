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

#define SRC_MAIN_SCHEDULER_C_

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <platform.h>

#include "scheduler.h"
#include "debug.h"

#include "common/maths.h"

#include "drivers/system.h"
#include "config/config_unittest.h"

cfTaskId_e currentTaskId = TASK_NONE;

#define REALTIME_GUARD_INTERVAL_MIN     10
#define REALTIME_GUARD_INTERVAL_MAX     300
#define REALTIME_GUARD_INTERVAL_MARGIN  25

static uint32_t totalWaitingTasks;
static uint32_t totalWaitingTasksSamples;
static uint32_t realtimeGuardInterval = REALTIME_GUARD_INTERVAL_MAX;

uint32_t currentTime = 0;
uint16_t averageSystemLoadPercent = 0;


void taskSystem(void)
{
    uint8_t taskId;

    /* Calculate system load */
    if (totalWaitingTasksSamples > 0) {
        averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
        totalWaitingTasksSamples = 0;
        totalWaitingTasks = 0;
    }

    /* Calculate guard interval */
    uint32_t maxNonRealtimeTaskTime = 0;
    for (taskId = 0; taskId < TASK_COUNT; taskId++) {
        if (cfTasks[taskId].staticPriority != TASK_PRIORITY_REALTIME) {
            maxNonRealtimeTaskTime = MAX(maxNonRealtimeTaskTime, cfTasks[taskId].averageExecutionTime);
        }
    }

    realtimeGuardInterval = constrain(maxNonRealtimeTaskTime, REALTIME_GUARD_INTERVAL_MIN, REALTIME_GUARD_INTERVAL_MAX) + REALTIME_GUARD_INTERVAL_MARGIN;
#if defined SCHEDULER_DEBUG
    debug[2] = realtimeGuardInterval;
#endif
}

#ifndef SKIP_TASK_STATISTICS
void getTaskInfo(cfTaskId_e taskId, cfTaskInfo_t * taskInfo)
{
    taskInfo->taskName = cfTasks[taskId].taskName;
    taskInfo->isEnabled= cfTasks[taskId].isEnabled;
    taskInfo->desiredPeriod = cfTasks[taskId].desiredPeriod;
    taskInfo->staticPriority = cfTasks[taskId].staticPriority;
    taskInfo->maxExecutionTime = cfTasks[taskId].maxExecutionTime;
    taskInfo->totalExecutionTime = cfTasks[taskId].totalExecutionTime;
    taskInfo->averageExecutionTime = cfTasks[taskId].averageExecutionTime;
}
#endif

void rescheduleTask(cfTaskId_e taskId, uint32_t newPeriodMicros)
{
    if (taskId == TASK_SELF)
        taskId = currentTaskId;

    if (taskId < TASK_COUNT) {
        cfTasks[taskId].desiredPeriod = MAX(100, newPeriodMicros);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    }
}

void setTaskEnabled(cfTaskId_e taskId, bool newEnabledState)
{
    if (taskId == TASK_SELF)
        taskId = currentTaskId;

    if (taskId < TASK_COUNT) {
        cfTasks[taskId].isEnabled = newEnabledState;
    }
}

uint32_t getTaskDeltaTime(cfTaskId_e taskId)
{
    if (taskId == TASK_SELF)
        taskId = currentTaskId;

    if (taskId < TASK_COUNT) {
        return cfTasks[taskId].taskLatestDeltaTime;
    }
    else {
        return 0;
    }
}

void scheduler(void)
{
    uint8_t taskId;
    /* The task to be invoked */
    uint8_t selectedTaskId = TASK_NONE;
    uint8_t selectedTaskDynPrio = 0;
    uint16_t waitingTasks = 0;
    uint32_t timeToNextRealtimeTask = UINT32_MAX;

    SET_SCHEDULER_LOCALS();
    /* Cache currentTime */
    currentTime = micros();

    /* Check for realtime tasks */
    for (taskId = 0; taskId < TASK_COUNT; taskId++) {
        if (cfTasks[taskId].staticPriority == TASK_PRIORITY_REALTIME) {
            uint32_t nextExecuteAt = cfTasks[taskId].lastExecutedAt + cfTasks[taskId].desiredPeriod;
            if ((int32_t)(currentTime - nextExecuteAt) >= 0) {
                timeToNextRealtimeTask = 0;
            }
            else {
                uint32_t newTimeInterval = nextExecuteAt - currentTime;
                timeToNextRealtimeTask = MIN(timeToNextRealtimeTask, newTimeInterval);
            }
        }
    }

    bool outsideRealtimeGuardInterval = (timeToNextRealtimeTask > realtimeGuardInterval);

    /* Update task dynamic priorities */
    for (taskId = 0; taskId < TASK_COUNT; taskId++) {
        if (cfTasks[taskId].isEnabled) {
            /* Task has checkFunc - event driven */
            if (cfTasks[taskId].checkFunc != NULL) {
                /* Increase priority for event driven tasks */
                if (cfTasks[taskId].dynamicPriority > 0) {
                    cfTasks[taskId].taskAgeCycles = 1 + ((currentTime - cfTasks[taskId].lastSignaledAt) / cfTasks[taskId].desiredPeriod);
                    cfTasks[taskId].dynamicPriority = 1 + cfTasks[taskId].staticPriority * cfTasks[taskId].taskAgeCycles;
                    waitingTasks++;
                }
                else if (cfTasks[taskId].checkFunc(currentTime - cfTasks[taskId].lastExecutedAt)) {
                    cfTasks[taskId].lastSignaledAt = currentTime;
                    cfTasks[taskId].taskAgeCycles = 1;
                    cfTasks[taskId].dynamicPriority = 1 + cfTasks[taskId].staticPriority;
                    waitingTasks++;
                }
                else {
                    cfTasks[taskId].taskAgeCycles = 0;
                }
            }
            /* Task is time-driven, dynamicPriority is last execution age measured in desiredPeriods) */
            else {
                // Task age is calculated from last execution
                cfTasks[taskId].taskAgeCycles = ((currentTime - cfTasks[taskId].lastExecutedAt) / cfTasks[taskId].desiredPeriod);
                if (cfTasks[taskId].taskAgeCycles > 0) {
                    cfTasks[taskId].dynamicPriority = 1 + cfTasks[taskId].staticPriority * cfTasks[taskId].taskAgeCycles;
                    waitingTasks++;
                }
            }

            /* limit new priority to avoid overflow of uint8_t */
            cfTasks[taskId].dynamicPriority = MIN(cfTasks[taskId].dynamicPriority, TASK_PRIORITY_MAX);;

            bool taskCanBeChosenForScheduling =
                (outsideRealtimeGuardInterval) ||
                (cfTasks[taskId].taskAgeCycles > 1) ||
                (cfTasks[taskId].staticPriority == TASK_PRIORITY_REALTIME);

            if (taskCanBeChosenForScheduling && (cfTasks[taskId].dynamicPriority > selectedTaskDynPrio)) {
                selectedTaskDynPrio = cfTasks[taskId].dynamicPriority;
                selectedTaskId = taskId;
            }
        }
    }

    totalWaitingTasksSamples += 1;
    totalWaitingTasks += waitingTasks;

    /* Found a task that should be run */
    if (selectedTaskId != TASK_NONE) {
        cfTasks[selectedTaskId].taskLatestDeltaTime = currentTime - cfTasks[selectedTaskId].lastExecutedAt;
        cfTasks[selectedTaskId].lastExecutedAt = currentTime;
        cfTasks[selectedTaskId].dynamicPriority = 0;

        currentTaskId = selectedTaskId;

        uint32_t currentTimeBeforeTaskCall = micros();

        /* Execute task */
        if (cfTasks[selectedTaskId].taskFunc != NULL) {
            cfTasks[selectedTaskId].taskFunc();
        }

        uint32_t taskExecutionTime = micros() - currentTimeBeforeTaskCall;

        cfTasks[selectedTaskId].averageExecutionTime = ((uint32_t)cfTasks[selectedTaskId].averageExecutionTime * 31 + taskExecutionTime) / 32;
#ifndef SKIP_TASK_STATISTICS
        cfTasks[selectedTaskId].totalExecutionTime += taskExecutionTime;   // time consumed by scheduler + task
        cfTasks[selectedTaskId].maxExecutionTime = MAX(cfTasks[selectedTaskId].maxExecutionTime, taskExecutionTime);
#endif
#if defined SCHEDULER_DEBUG
        debug[3] = (micros() - currentTime) - taskExecutionTime;
#endif
    }
    else {
        currentTaskId = TASK_NONE;
#if defined SCHEDULER_DEBUG
        debug[3] = (micros() - currentTime);
#endif
    }
    GET_SCHEDULER_LOCALS();
}
