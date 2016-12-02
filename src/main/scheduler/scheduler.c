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
#include <stdint.h>
#include <string.h>

#ifdef UNIT_TEST
typedef enum {TEST_IRQ = 0 } IRQn_Type;
#endif
#include "platform.h"

#include "scheduler.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/system.h"

static cfTask_t *currentTask = NULL;

static uint32_t totalWaitingTasks;
static uint32_t totalWaitingTasksSamples;
static timeUs_t realtimeGuardInterval;

uint16_t averageSystemLoadPercent = 0;

#define REALTIME_GUARD_INTERVAL_MIN     10
#define REALTIME_GUARD_INTERVAL_MAX     300
#define REALTIME_GUARD_INTERVAL_MARGIN  25
#define TASK_MOVING_SUM_COUNT           32

static int taskQueuePos = 0;
static int taskQueueSize = 0;
// No need for a linked list for the queue, since items are only inserted at startup
#ifdef UNIT_TEST
STATIC_UNIT_TESTED cfTask_t* taskQueueArray[TASK_COUNT + 2]; // 1 extra space so test code can check for buffer overruns
#else
static cfTask_t* taskQueueArray[TASK_COUNT + 1]; // extra item for NULL pointer at end of queue
#endif
STATIC_UNIT_TESTED void queueClear(void)
{
    memset(taskQueueArray, 0, sizeof(taskQueueArray));
    taskQueuePos = 0;
    taskQueueSize = 0;
}

#ifdef UNIT_TEST
STATIC_UNIT_TESTED int queueSize(void)
{
    return taskQueueSize;
}
#endif

STATIC_UNIT_TESTED bool queueContains(cfTask_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            return true;
        }
    }
    return false;
}

STATIC_UNIT_TESTED bool queueAdd(cfTask_t *task)
{
    if ((taskQueueSize >= TASK_COUNT) || queueContains(task)) {
        return false;
    }
    for (int ii = 0; ii <= taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->staticPriority < task->staticPriority) {
            memmove(&taskQueueArray[ii+1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
            taskQueueArray[ii] = task;
            ++taskQueueSize;
            return true;
        }
    }
    return false;
}

STATIC_UNIT_TESTED bool queueRemove(cfTask_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            memmove(&taskQueueArray[ii], &taskQueueArray[ii+1], sizeof(task) * (taskQueueSize - ii));
            --taskQueueSize;
            return true;
        }
    }
    return false;
}

/*
 * Returns first item queue or NULL if queue empty
 */
STATIC_INLINE_UNIT_TESTED cfTask_t *queueFirst(void)
{
    taskQueuePos = 0;
    return taskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

/*
 * Returns next item in queue or NULL if at end of queue
 */
STATIC_INLINE_UNIT_TESTED cfTask_t *queueNext(void)
{
    return taskQueueArray[++taskQueuePos]; // guaranteed to be NULL at end of queue
}

void taskSystem(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Calculate system load
    if (totalWaitingTasksSamples > 0) {
        averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
        totalWaitingTasksSamples = 0;
        totalWaitingTasks = 0;
    }

    // Calculate guard interval
    timeUs_t maxNonRealtimeTaskTime = 0;
    for (const cfTask_t *task = queueFirst(); task != NULL; task = queueNext()) {
        if (task->staticPriority != TASK_PRIORITY_REALTIME) {
            const uint32_t taskAverageExecutionTime = task->movingSumExecutionTime / TASK_MOVING_SUM_COUNT;
            maxNonRealtimeTaskTime = MAX(maxNonRealtimeTaskTime, taskAverageExecutionTime);
        }
    }

    realtimeGuardInterval = constrain(maxNonRealtimeTaskTime, REALTIME_GUARD_INTERVAL_MIN, REALTIME_GUARD_INTERVAL_MAX) + REALTIME_GUARD_INTERVAL_MARGIN;
#if defined SCHEDULER_DEBUG
    debug[2] = realtimeGuardInterval;
#endif
}

#ifndef SKIP_TASK_STATISTICS
uint32_t checkFuncMaxExecutionTime;
uint32_t checkFuncTotalExecutionTime;
uint32_t checkFuncMovingSumExecutionTime;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo)
{
    checkFuncInfo->maxExecutionTime = checkFuncMaxExecutionTime;
    checkFuncInfo->totalExecutionTime = checkFuncTotalExecutionTime;
    checkFuncInfo->averageExecutionTime = checkFuncMovingSumExecutionTime / TASK_MOVING_SUM_COUNT;
}

void getTaskInfo(cfTaskId_e taskId, cfTaskInfo_t * taskInfo)
{
    taskInfo->taskName = cfTasks[taskId].taskName;
    taskInfo->isEnabled = queueContains(&cfTasks[taskId]);
    taskInfo->desiredPeriod = cfTasks[taskId].desiredPeriod;
    taskInfo->staticPriority = cfTasks[taskId].staticPriority;
    taskInfo->maxExecutionTime = cfTasks[taskId].maxExecutionTime;
    taskInfo->totalExecutionTime = cfTasks[taskId].totalExecutionTime;
    taskInfo->averageExecutionTime = cfTasks[taskId].movingSumExecutionTime / TASK_MOVING_SUM_COUNT;
    taskInfo->latestDeltaTime = cfTasks[taskId].taskLatestDeltaTime;
}
#endif

void rescheduleTask(cfTaskId_e taskId, uint32_t newPeriodMicros)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        cfTask_t *task = taskId == TASK_SELF ? currentTask : &cfTasks[taskId];
        task->desiredPeriod = MAX((uint32_t)100, newPeriodMicros);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    }
}

void setTaskEnabled(cfTaskId_e taskId, bool enabled)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        cfTask_t *task = taskId == TASK_SELF ? currentTask : &cfTasks[taskId];
        if (enabled && task->taskFunc) {
            queueAdd(task);
        } else {
            queueRemove(task);
        }
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
    queueClear();
    queueAdd(&cfTasks[TASK_SYSTEM]);
}

void scheduler(void)
{
    // Cache currentTime
    const timeUs_t currentTimeUs = micros();

    // Check for realtime tasks
    timeUs_t timeToNextRealtimeTask = TIMEUS_MAX;
    for (const cfTask_t *task = queueFirst(); task != NULL && task->staticPriority >= TASK_PRIORITY_REALTIME; task = queueNext()) {
        const timeUs_t nextExecuteAt = task->lastExecutedAt + task->desiredPeriod;
        if ((int32_t)(currentTimeUs - nextExecuteAt) >= 0) {
            timeToNextRealtimeTask = 0;
        } else {
            const timeUs_t newTimeInterval = nextExecuteAt - currentTimeUs;
            timeToNextRealtimeTask = MIN(timeToNextRealtimeTask, newTimeInterval);
        }
    }
    const bool outsideRealtimeGuardInterval = (timeToNextRealtimeTask > realtimeGuardInterval);

    // The task to be invoked
    cfTask_t *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;

    // Update task dynamic priorities
    uint16_t waitingTasks = 0;
    for (cfTask_t *task = queueFirst(); task != NULL; task = queueNext()) {
        // Task has checkFunc - event driven
        if (task->checkFunc != NULL) {
            const timeUs_t currentTimeBeforeCheckFuncCallUs = micros();

            // Increase priority for event driven tasks
            if (task->dynamicPriority > 0) {
                task->taskAgeCycles = 1 + ((currentTimeUs - task->lastSignaledAt) / task->desiredPeriod);
                task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                waitingTasks++;
            } else if (task->checkFunc(currentTimeBeforeCheckFuncCallUs, currentTimeBeforeCheckFuncCallUs - task->lastExecutedAt)) {
#ifndef SKIP_TASK_STATISTICS
                const timeUs_t checkFuncExecutionTime = micros() - currentTimeBeforeCheckFuncCallUs;
                checkFuncMovingSumExecutionTime += checkFuncExecutionTime - checkFuncMovingSumExecutionTime / TASK_MOVING_SUM_COUNT;
                checkFuncTotalExecutionTime += checkFuncExecutionTime;   // time consumed by scheduler + task
                checkFuncMaxExecutionTime = MAX(checkFuncMaxExecutionTime, checkFuncExecutionTime);
#endif
                task->lastSignaledAt = currentTimeBeforeCheckFuncCallUs;
                task->taskAgeCycles = 1;
                task->dynamicPriority = 1 + task->staticPriority;
                waitingTasks++;
            } else {
                task->taskAgeCycles = 0;
            }
        } else {
            // Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
            // Task age is calculated from last execution
            task->taskAgeCycles = ((currentTimeUs - task->lastExecutedAt) / task->desiredPeriod);
            if (task->taskAgeCycles > 0) {
                task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                waitingTasks++;
            }
        }

        if (task->dynamicPriority > selectedTaskDynamicPriority) {
            const bool taskCanBeChosenForScheduling =
                (outsideRealtimeGuardInterval) ||
                (task->taskAgeCycles > 1) ||
                (task->staticPriority == TASK_PRIORITY_REALTIME);
            if (taskCanBeChosenForScheduling) {
                selectedTaskDynamicPriority = task->dynamicPriority;
                selectedTask = task;
            }
        }
    }

    totalWaitingTasksSamples++;
    totalWaitingTasks += waitingTasks;

    currentTask = selectedTask;

    if (selectedTask != NULL) {
        // Found a task that should be run
        selectedTask->taskLatestDeltaTime = currentTimeUs - selectedTask->lastExecutedAt;
        selectedTask->lastExecutedAt = currentTimeUs;
        selectedTask->dynamicPriority = 0;

        // Execute task
        const timeUs_t currentTimeBeforeTaskCall = micros();
        selectedTask->taskFunc(currentTimeBeforeTaskCall);
        const timeUs_t taskExecutionTime = micros() - currentTimeBeforeTaskCall;

        selectedTask->movingSumExecutionTime += taskExecutionTime - selectedTask->movingSumExecutionTime / TASK_MOVING_SUM_COUNT;

#ifndef SKIP_TASK_STATISTICS
        selectedTask->totalExecutionTime += taskExecutionTime;   // time consumed by scheduler + task
        selectedTask->maxExecutionTime = MAX(selectedTask->maxExecutionTime, taskExecutionTime);
#endif
#if defined SCHEDULER_DEBUG
        debug[3] = (micros() - currentTimeUs) - taskExecutionTime;
    } else {
        debug[3] = (micros() - currentTimeUs);
#endif
    }
}
