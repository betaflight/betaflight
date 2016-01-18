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

static cfTask_t *currentTask = NULL;

#define REALTIME_GUARD_INTERVAL_MIN     10
#define REALTIME_GUARD_INTERVAL_MAX     300
#define REALTIME_GUARD_INTERVAL_MARGIN  25

static uint32_t totalWaitingTasks;
static uint32_t totalWaitingTasksSamples;
static uint32_t realtimeGuardInterval = REALTIME_GUARD_INTERVAL_MAX;

uint32_t currentTime = 0;
uint16_t averageSystemLoadPercent = 0;


static int queuePos = 0;
static int queueSize = 0;
static cfTask_t* queueArray[TASK_COUNT];

static void queueInit(void)
{
    queuePos = 0;
    queueSize = 0;
    // put the enabled tasks in the queue in priority order
    const cfTaskPriority_e priorities[] =
        {TASK_PRIORITY_MAX, TASK_PRIORITY_REALTIME, TASK_PRIORITY_HIGH, TASK_PRIORITY_MEDIUM, TASK_PRIORITY_LOW, TASK_PRIORITY_IDLE};
    for (unsigned int ii = 0; ii < sizeof(priorities); ++ii) {
        const cfTaskPriority_e priority = priorities[ii];
        for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
            cfTask_t *task = &cfTasks[taskId];
            if (task->staticPriority == priority && task->isEnabled == true) {
                queueArray[queuePos] = task;
                ++queuePos;
                ++queueSize;
            }
        }
    }
}
static cfTask_t *queueFirst(void)
{
    queuePos = 0;
    return queueSize > 0 ? queueArray[0] : NULL;
}

static cfTask_t *queueNext(void) {
    ++queuePos;
    return queuePos < queueSize ? queueArray[queuePos] : NULL;
}

void taskSystem(void)
{
    /* Calculate system load */
    if (totalWaitingTasksSamples > 0) {
        averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
        totalWaitingTasksSamples = 0;
        totalWaitingTasks = 0;
    }

    /* Calculate guard interval */
    uint32_t maxNonRealtimeTaskTime = 0;
    for (const cfTask_t *task = queueFirst(); task != NULL; task = queueNext()) {
        if (task->staticPriority != TASK_PRIORITY_REALTIME) {
            maxNonRealtimeTaskTime = MAX(maxNonRealtimeTaskTime, task->averageExecutionTime);
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
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        cfTask_t *task = taskId == TASK_SELF ? currentTask : &cfTasks[taskId];
        task->desiredPeriod = MAX(100, newPeriodMicros);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    }
}

void setTaskEnabled(cfTaskId_e taskId, bool newEnabledState)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        cfTask_t *task = taskId == TASK_SELF ? currentTask : &cfTasks[taskId];
        task->isEnabled = newEnabledState;
    }
}

uint32_t getTaskDeltaTime(cfTaskId_e taskId)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        cfTask_t *task = taskId == TASK_SELF ? currentTask : &cfTasks[taskId];
        return task->taskLatestDeltaTime;
    } else {
        return 0;
    }
}

void schedulerInit(void)
{
    queueInit();
}

void scheduler(void)
{
    /* Cache currentTime */
    currentTime = micros();

    /* Check for realtime tasks */
    uint32_t timeToNextRealtimeTask = UINT32_MAX;
    for (const cfTask_t *task = queueFirst(); task != NULL && task->staticPriority >= TASK_PRIORITY_REALTIME; task = queueNext()) {
        const uint32_t nextExecuteAt = task->lastExecutedAt + task->desiredPeriod;
        if ((int32_t)(currentTime - nextExecuteAt) >= 0) {
            timeToNextRealtimeTask = 0;
        }
        else {
            const uint32_t newTimeInterval = nextExecuteAt - currentTime;
            timeToNextRealtimeTask = MIN(timeToNextRealtimeTask, newTimeInterval);
        }
    }

    /* The task to be invoked */
    uint8_t selectedTaskDynPrio = 0;
    cfTask_t *selectedTask = NULL;

    /* Update task dynamic priorities */
    uint16_t waitingTasks = 0;
    for (cfTask_t *task = queueFirst(); task != NULL; task = queueNext()) {
        /* Task has checkFunc - event driven */
        if (task->checkFunc != NULL) {
            /* Increase priority for event driven tasks */
            if (task->dynamicPriority > 0) {
                task->taskAgeCycles = 1 + ((currentTime - task->lastSignaledAt) / task->desiredPeriod);
                task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                waitingTasks++;
            }
            else if (task->checkFunc(currentTime - task->lastExecutedAt)) {
                task->lastSignaledAt = currentTime;
                task->taskAgeCycles = 1;
                task->dynamicPriority = 1 + task->staticPriority;
                waitingTasks++;
            }
            else {
                task->taskAgeCycles = 0;
            }
        }
        /* Task is time-driven, dynamicPriority is last execution age measured in desiredPeriods) */
        else {
            // Task age is calculated from last execution
            task->taskAgeCycles = ((currentTime - task->lastExecutedAt) / task->desiredPeriod);
            if (task->taskAgeCycles > 0) {
                task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                waitingTasks++;
            }
        }

        /* limit new priority to avoid overflow of uint8_t */
        task->dynamicPriority = MIN(task->dynamicPriority, TASK_PRIORITY_MAX);;

        if (task->dynamicPriority > selectedTaskDynPrio) {
            const bool outsideRealtimeGuardInterval = (timeToNextRealtimeTask > realtimeGuardInterval);
            bool taskCanBeChosenForScheduling =
                (outsideRealtimeGuardInterval) ||
                (task->taskAgeCycles > 1) ||
                (task->staticPriority == TASK_PRIORITY_REALTIME);
            if (taskCanBeChosenForScheduling) {
                selectedTaskDynPrio = task->dynamicPriority;
                selectedTask = task;
            }
        }
    }

    totalWaitingTasksSamples += 1;
    totalWaitingTasks += waitingTasks;

    /* Found a task that should be run */
    if (selectedTask != NULL) {
        selectedTask->taskLatestDeltaTime = currentTime - selectedTask->lastExecutedAt;
        selectedTask->lastExecutedAt = currentTime;
        selectedTask->dynamicPriority = 0;

        currentTask = selectedTask;

        uint32_t currentTimeBeforeTaskCall = micros();

        /* Execute task */
        selectedTask->taskFunc();

        uint32_t taskExecutionTime = micros() - currentTimeBeforeTaskCall;

        selectedTask->averageExecutionTime = ((uint32_t)selectedTask->averageExecutionTime * 31 + taskExecutionTime) / 32;
#ifndef SKIP_TASK_STATISTICS
        selectedTask->totalExecutionTime += taskExecutionTime;   // time consumed by scheduler + task
        selectedTask->maxExecutionTime = MAX(selectedTask->maxExecutionTime, taskExecutionTime);
#endif
#if defined SCHEDULER_DEBUG
        debug[3] = (micros() - currentTime) - taskExecutionTime;
#endif
    }
    else {
        currentTask = NULL;
#if defined SCHEDULER_DEBUG
        debug[3] = (micros() - currentTime);
#endif
    }
    GET_SCHEDULER_LOCALS();
}
