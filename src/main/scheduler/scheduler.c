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

#define SRC_MAIN_SCHEDULER_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/accgyro/accgyro.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/system.h"

#include "fc/core.h"
#include "fc/tasks.h"

#include "rx/rx.h"
#include "flight/failsafe.h"

#include "scheduler.h"

#include "sensors/gyro_init.h"

// DEBUG_SCHEDULER, timings for:
// 0 - Average time spent executing check function
// 1 - Time spent priortising
// 2 - time spent in scheduler

// DEBUG_SCHEDULER_DETERMINISM, requires USE_LATE_TASK_STATISTICS to be defined
// 0 - Gyro task start cycle time in 10th of a us
// 1 - ID of late task
// 2 - Amount task is late in 10th of a us
// 3 - Gyro lock skew in 10th of a us

// DEBUG_TIMING_ACCURACY, requires USE_LATE_TASK_STATISTICS to be defined
// 0 - % CPU busy
// 1 - Tasks late in last second
// 2 - Total lateness in last second in 10ths us
// 3 - Total tasks run in last second

extern task_t tasks[];

static FAST_DATA_ZERO_INIT task_t *currentTask = NULL;
static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskExecRate;
static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskExecTime;

int32_t schedLoopStartCycles;
static int32_t schedLoopStartMinCycles;
static int32_t schedLoopStartMaxCycles;
static uint32_t schedLoopStartDeltaDownCycles;
static uint32_t schedLoopStartDeltaUpCycles;

int32_t taskGuardCycles;
static int32_t taskGuardMinCycles;
static int32_t taskGuardMaxCycles;
static uint32_t taskGuardDeltaDownCycles;
static uint32_t taskGuardDeltaUpCycles;

FAST_DATA_ZERO_INIT uint16_t averageSystemLoadPercent = 0;

static FAST_DATA_ZERO_INIT int taskQueuePos = 0;
STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT int taskQueueSize = 0;

static FAST_DATA_ZERO_INIT bool gyroEnabled;

static int32_t desiredPeriodCycles;
static uint32_t lastTargetCycles;

static uint8_t skippedRxAttempts = 0;
#ifdef USE_OSD
static uint8_t skippedOSDAttempts = 0;
#endif

#if defined(USE_LATE_TASK_STATISTICS)
static int16_t lateTaskCount = 0;
static uint32_t lateTaskTotal = 0;
static int16_t taskCount = 0;
static uint32_t nextTimingCycles;
#endif

static timeMs_t lastFailsafeCheckMs = 0;

// No need for a linked list for the queue, since items are only inserted at startup

STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT task_t* taskQueueArray[TASK_COUNT + 1]; // extra item for NULL pointer at end of queue

void queueClear(void)
{
    memset(taskQueueArray, 0, sizeof(taskQueueArray));
    taskQueuePos = 0;
    taskQueueSize = 0;
}

bool queueContains(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            return true;
        }
    }
    return false;
}

bool queueAdd(task_t *task)
{
    if ((taskQueueSize >= TASK_COUNT) || queueContains(task)) {
        return false;
    }
    for (int ii = 0; ii <= taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->attribute->staticPriority < task->attribute->staticPriority) {
            memmove(&taskQueueArray[ii+1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
            taskQueueArray[ii] = task;
            ++taskQueueSize;
            return true;
        }
    }
    return false;
}

bool queueRemove(task_t *task)
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
FAST_CODE task_t *queueFirst(void)
{
    taskQueuePos = 0;
    return taskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

/*
 * Returns next item in queue or NULL if at end of queue
 */
FAST_CODE task_t *queueNext(void)
{
    return taskQueueArray[++taskQueuePos]; // guaranteed to be NULL at end of queue
}

static timeUs_t taskTotalExecutionTime = 0;

void taskSystemLoad(timeUs_t currentTimeUs)
{
    static timeUs_t lastExecutedAtUs;
    timeDelta_t deltaTime = cmpTimeUs(currentTimeUs, lastExecutedAtUs);

    // Calculate system load
    if (deltaTime) {
        averageSystemLoadPercent = 100 * taskTotalExecutionTime / deltaTime;
        taskTotalExecutionTime = 0;
        lastExecutedAtUs = currentTimeUs;
    } else {
        schedulerIgnoreTaskExecTime();
    }

#if defined(SIMULATOR_BUILD)
    averageSystemLoadPercent = 0;
#endif
}

timeUs_t checkFuncMaxExecutionTimeUs;
timeUs_t checkFuncTotalExecutionTimeUs;
timeUs_t checkFuncMovingSumExecutionTimeUs;
timeUs_t checkFuncMovingSumDeltaTimeUs;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo)
{
    checkFuncInfo->maxExecutionTimeUs = checkFuncMaxExecutionTimeUs;
    checkFuncInfo->totalExecutionTimeUs = checkFuncTotalExecutionTimeUs;
    checkFuncInfo->averageExecutionTimeUs = checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    checkFuncInfo->averageDeltaTimeUs = checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
}

void getTaskInfo(taskId_e taskId, taskInfo_t * taskInfo)
{
    taskInfo->isEnabled = queueContains(getTask(taskId));
    taskInfo->desiredPeriodUs = getTask(taskId)->attribute->desiredPeriodUs;
    taskInfo->staticPriority = getTask(taskId)->attribute->staticPriority;
    taskInfo->taskName = getTask(taskId)->attribute->taskName;
    taskInfo->subTaskName = getTask(taskId)->attribute->subTaskName;
    taskInfo->maxExecutionTimeUs = getTask(taskId)->maxExecutionTimeUs;
    taskInfo->totalExecutionTimeUs = getTask(taskId)->totalExecutionTimeUs;
    taskInfo->averageExecutionTime10thUs = getTask(taskId)->movingSumExecutionTime10thUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->averageDeltaTime10thUs = getTask(taskId)->movingSumDeltaTime10thUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->latestDeltaTimeUs = getTask(taskId)->taskLatestDeltaTimeUs;
    taskInfo->movingAverageCycleTimeUs = getTask(taskId)->movingAverageCycleTimeUs;
#if defined(USE_LATE_TASK_STATISTICS)
    taskInfo->lateCount = getTask(taskId)->lateCount;
    taskInfo->runCount = getTask(taskId)->runCount;
    taskInfo->execTime = getTask(taskId)->execTime;
#endif
}

void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs)
{
    task_t *task;

    if (taskId == TASK_SELF) {
        task = currentTask;
    } else if (taskId < TASK_COUNT) {
        task = getTask(taskId);
    } else {
        return;
    }
    task->attribute->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging

    // Catch the case where the gyro loop is adjusted
    if (taskId == TASK_GYRO) {
        desiredPeriodCycles = (int32_t)clockMicrosToCycles((uint32_t)getTask(TASK_GYRO)->attribute->desiredPeriodUs);
    }
}

void setTaskEnabled(taskId_e taskId, bool enabled)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        task_t *task = taskId == TASK_SELF ? currentTask : getTask(taskId);
        if (enabled && task->attribute->taskFunc) {
            queueAdd(task);
        } else {
            queueRemove(task);
        }
    }
}

timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        return currentTask->taskLatestDeltaTimeUs;
    } else if (taskId < TASK_COUNT) {
        return getTask(taskId)->taskLatestDeltaTimeUs;
    } else {
        return 0;
    }
}

// Called by tasks executing what are known to be short states
void schedulerIgnoreTaskStateTime(void)
{
    ignoreCurrentTaskExecRate = true;
    ignoreCurrentTaskExecTime = true;
}

// Called by tasks with state machines to only count one state as determining rate
void schedulerIgnoreTaskExecRate(void)
{
    ignoreCurrentTaskExecRate = true;
}

// Called by tasks without state machines executing in what is known to be a shorter time than peak
void schedulerIgnoreTaskExecTime(void)
{
    ignoreCurrentTaskExecTime = true;
}

bool schedulerGetIgnoreTaskExecTime(void)
{
    return ignoreCurrentTaskExecTime;
}

void schedulerResetTaskStatistics(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        currentTask->anticipatedExecutionTime = 0;
        currentTask->movingSumDeltaTime10thUs = 0;
        currentTask->totalExecutionTimeUs = 0;
        currentTask->maxExecutionTimeUs = 0;
    } else if (taskId < TASK_COUNT) {
        getTask(taskId)->anticipatedExecutionTime = 0;
        getTask(taskId)->movingSumDeltaTime10thUs = 0;
        getTask(taskId)->totalExecutionTimeUs = 0;
        getTask(taskId)->maxExecutionTimeUs = 0;
    }
}

void schedulerResetTaskMaxExecutionTime(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        currentTask->maxExecutionTimeUs = 0;
    } else if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        task->maxExecutionTimeUs = 0;
#if defined(USE_LATE_TASK_STATISTICS)
        task->lateCount = 0;
        task->runCount = 0;
#endif
    }
}

void schedulerResetCheckFunctionMaxExecutionTime(void)
{
    checkFuncMaxExecutionTimeUs = 0;
}

void schedulerInit(void)
{
    queueClear();
    queueAdd(getTask(TASK_SYSTEM));

    schedLoopStartMinCycles = clockMicrosToCycles(SCHED_START_LOOP_MIN_US);
    schedLoopStartMaxCycles = clockMicrosToCycles(SCHED_START_LOOP_MAX_US);
    schedLoopStartCycles = schedLoopStartMinCycles;
    schedLoopStartDeltaDownCycles = clockMicrosToCycles(1) / SCHED_START_LOOP_DOWN_STEP;
    schedLoopStartDeltaUpCycles = clockMicrosToCycles(1) / SCHED_START_LOOP_UP_STEP;

    taskGuardMinCycles = clockMicrosToCycles(TASK_GUARD_MARGIN_MIN_US);
    taskGuardMaxCycles = clockMicrosToCycles(TASK_GUARD_MARGIN_MAX_US);
    taskGuardCycles = taskGuardMinCycles;
    taskGuardDeltaDownCycles = clockMicrosToCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
    taskGuardDeltaUpCycles = clockMicrosToCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

    desiredPeriodCycles = (int32_t)clockMicrosToCycles((uint32_t)getTask(TASK_GYRO)->attribute->desiredPeriodUs);

    lastTargetCycles = getCycleCounter();

#if defined(USE_LATE_TASK_STATISTICS)
    nextTimingCycles = lastTargetCycles;
#endif

    for (taskId_e taskId = 0; taskId < TASK_COUNT; taskId++) {
        schedulerResetTaskStatistics(taskId);
    }
}

static timeDelta_t taskNextStateTime;

FAST_CODE void schedulerSetNextStateTime(timeDelta_t nextStateTime)
{
    taskNextStateTime = nextStateTime;
}

FAST_CODE timeDelta_t schedulerGetNextStateTime(void)
{
    return currentTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
}

FAST_CODE timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs)
{
    timeUs_t taskExecutionTimeUs = 0;

    if (selectedTask) {
        currentTask = selectedTask;
        ignoreCurrentTaskExecRate = false;
        ignoreCurrentTaskExecTime = false;
        taskNextStateTime = -1;
        float period = currentTimeUs - selectedTask->lastExecutedAtUs;
        selectedTask->lastExecutedAtUs = currentTimeUs;
        selectedTask->lastDesiredAt += selectedTask->attribute->desiredPeriodUs;
        selectedTask->dynamicPriority = 0;

        // Execute task
        const timeUs_t currentTimeBeforeTaskCallUs = micros();
        selectedTask->attribute->taskFunc(currentTimeBeforeTaskCallUs);
        taskExecutionTimeUs = micros() - currentTimeBeforeTaskCallUs;
        taskTotalExecutionTime += taskExecutionTimeUs;
        selectedTask->movingSumExecutionTime10thUs += (taskExecutionTimeUs * 10) - selectedTask->movingSumExecutionTime10thUs / TASK_STATS_MOVING_SUM_COUNT;
        if (!ignoreCurrentTaskExecRate) {
            // Record task execution rate and max execution time
            selectedTask->taskLatestDeltaTimeUs = cmpTimeUs(currentTimeUs, selectedTask->lastStatsAtUs);
            selectedTask->movingSumDeltaTime10thUs += (selectedTask->taskLatestDeltaTimeUs * 10) - selectedTask->movingSumDeltaTime10thUs / TASK_STATS_MOVING_SUM_COUNT;
            selectedTask->lastStatsAtUs = currentTimeUs;
        }

        // Update estimate of expected task duration
        if (taskNextStateTime != -1) {
            selectedTask->anticipatedExecutionTime = taskNextStateTime << TASK_EXEC_TIME_SHIFT;
        } else if (!ignoreCurrentTaskExecTime) {
            if (taskExecutionTimeUs > (selectedTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT)) {
                selectedTask->anticipatedExecutionTime = taskExecutionTimeUs << TASK_EXEC_TIME_SHIFT;
            } else if (selectedTask->anticipatedExecutionTime > 1) {
                // Slowly decay the max time
                selectedTask->anticipatedExecutionTime--;
            }
        }

        if (!ignoreCurrentTaskExecTime) {
            selectedTask->maxExecutionTimeUs = MAX(selectedTask->maxExecutionTimeUs, taskExecutionTimeUs);
        }

        selectedTask->totalExecutionTimeUs += taskExecutionTimeUs;   // time consumed by scheduler + task
        selectedTask->movingAverageCycleTimeUs += 0.05f * (period - selectedTask->movingAverageCycleTimeUs);
#if defined(USE_LATE_TASK_STATISTICS)
        selectedTask->runCount++;
#endif
    }

    return taskExecutionTimeUs;
}

#if defined(UNIT_TEST)
task_t *unittest_scheduler_selectedTask;
uint8_t unittest_scheduler_selectedTaskDynamicPriority;

static void readSchedulerLocals(task_t *selectedTask, uint8_t selectedTaskDynamicPriority)
{
    unittest_scheduler_selectedTask = selectedTask;
    unittest_scheduler_selectedTaskDynamicPriority = selectedTaskDynamicPriority;
}
#endif

FAST_CODE void scheduler(void)
{
    static uint32_t checkCycles = 0;
    static uint32_t scheduleCount = 0;
#if !defined(UNIT_TEST)
    const timeUs_t schedulerStartTimeUs = micros();
#endif
    timeUs_t currentTimeUs;
    uint32_t nowCycles;
    timeUs_t taskExecutionTimeUs = 0;
    task_t *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;
    uint32_t nextTargetCycles = 0;
    int32_t schedLoopRemainingCycles;

#if defined(UNIT_TEST)
    if (nextTargetCycles == 0) {
        lastTargetCycles = getCycleCounter();
        nextTargetCycles = lastTargetCycles + desiredPeriodCycles;
    }
#endif

    if (gyroEnabled) {
        // Realtime gyro/filtering/PID tasks get complete priority
        task_t *gyroTask = getTask(TASK_GYRO);
        nowCycles = getCycleCounter();
#if defined(UNIT_TEST)
        lastTargetCycles = clockMicrosToCycles(gyroTask->lastExecutedAtUs);
#endif
        nextTargetCycles = lastTargetCycles + desiredPeriodCycles;
        schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

        if (schedLoopRemainingCycles < -desiredPeriodCycles) {
            /* A task has so grossly overrun that at entire gyro cycle has been skipped
             * This is most likely to occur when connected to the configurator via USB as the serial
             * task is non-deterministic
             * Recover as best we can, advancing scheduling by a whole number of cycles
             */
            nextTargetCycles += desiredPeriodCycles * (1 + (schedLoopRemainingCycles / -desiredPeriodCycles));
            schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
        }

        // Tune out the time lost between completing the last task execution and re-entering the scheduler
        if ((schedLoopRemainingCycles < schedLoopStartMinCycles) &&
            (schedLoopStartCycles < schedLoopStartMaxCycles)) {
            schedLoopStartCycles += schedLoopStartDeltaUpCycles;
        }

        // Once close to the timing boundary, poll for it's arrival
        if (schedLoopRemainingCycles < schedLoopStartCycles) {
            if (schedLoopStartCycles > schedLoopStartMinCycles) {
                schedLoopStartCycles -= schedLoopStartDeltaDownCycles;
            }
#if !defined(UNIT_TEST)
            while (schedLoopRemainingCycles > 0) {
                nowCycles = getCycleCounter();
                schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
            }
            DEBUG_SET(DEBUG_SCHEDULER_DETERMINISM, 0, clockCyclesTo10thMicros(cmpTimeCycles(nowCycles, lastTargetCycles)));
#endif
            currentTimeUs = micros();
            taskExecutionTimeUs += schedulerExecuteTask(gyroTask, currentTimeUs);

            if (gyroFilterReady()) {
                taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_FILTER), currentTimeUs);
            }
            if (pidLoopReady()) {
                taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_PID), currentTimeUs);
            }

            // Check for incoming RX data. Don't do this in the checker as that is called repeatedly within
            // a given gyro loop, and ELRS takes a long time to process this and so can only be safely processed
            // before the checkers
            rxFrameCheck(currentTimeUs, cmpTimeUs(currentTimeUs, getTask(TASK_RX)->lastExecutedAtUs));

            // Check for failsafe conditions without reliance on the RX task being well behaved
            if (cmp32(millis(), lastFailsafeCheckMs) > PERIOD_RXDATA_FAILURE) {
                // This is very low cost taking less that 4us every 10ms
                failsafeCheckDataFailurePeriod();
                failsafeUpdateState();
                lastFailsafeCheckMs = millis();
            }

#if defined(USE_LATE_TASK_STATISTICS)
            // % CPU busy
            DEBUG_SET(DEBUG_TIMING_ACCURACY, 0, getAverageSystemLoadPercent());

            if (cmpTimeCycles(nextTimingCycles, nowCycles) < 0) {
                nextTimingCycles += clockMicrosToCycles(1000000);

                // Tasks late in last second
                DEBUG_SET(DEBUG_TIMING_ACCURACY, 1, lateTaskCount);
                // Total lateness in last second in us
                DEBUG_SET(DEBUG_TIMING_ACCURACY, 2, clockCyclesTo10thMicros(lateTaskTotal));
                // Total tasks run in last second
                DEBUG_SET(DEBUG_TIMING_ACCURACY, 3, taskCount);

                lateTaskCount = 0;
                lateTaskTotal = 0;
                taskCount = 0;
            }
#endif
            lastTargetCycles = nextTargetCycles;

            gyroDev_t *gyro = gyroActiveDev();

            // Bring the scheduler into lock with the gyro
            if (gyro->gyroModeSPI != GYRO_EXTI_NO_INT) {
                // Track the actual gyro rate over given number of cycle times and set the expected timebase
                static uint32_t terminalGyroRateCount = 0;
                static int32_t sampleRateStartCycles;

                if (terminalGyroRateCount == 0) {
                    terminalGyroRateCount = gyro->detectedEXTI + GYRO_RATE_COUNT;
                    sampleRateStartCycles = nowCycles;
                }

                if (gyro->detectedEXTI >= terminalGyroRateCount) {
                    // Calculate the number of clock cycles on average between gyro interrupts
                    uint32_t sampleCycles = nowCycles - sampleRateStartCycles;
                    desiredPeriodCycles = sampleCycles / GYRO_RATE_COUNT;
                    sampleRateStartCycles = nowCycles;
                    terminalGyroRateCount += GYRO_RATE_COUNT;
                }

                // Track the actual gyro rate over given number of cycle times and remove skew
                static uint32_t terminalGyroLockCount = 0;
                static int32_t accGyroSkew = 0;

                int32_t gyroSkew = cmpTimeCycles(nextTargetCycles, gyro->gyroSyncEXTI) % desiredPeriodCycles;
                if (gyroSkew > (desiredPeriodCycles / 2)) {
                    gyroSkew -= desiredPeriodCycles;
                }

                accGyroSkew += gyroSkew;

                if (terminalGyroLockCount == 0) {
                    terminalGyroLockCount = gyro->detectedEXTI + GYRO_LOCK_COUNT;
                }

                if (gyro->detectedEXTI >= terminalGyroLockCount) {
                    terminalGyroLockCount += GYRO_LOCK_COUNT;

                    // Move the desired start time of the gyroTask
                    lastTargetCycles -= (accGyroSkew/GYRO_LOCK_COUNT);
                    DEBUG_SET(DEBUG_SCHEDULER_DETERMINISM, 3, clockCyclesTo10thMicros(accGyroSkew/GYRO_LOCK_COUNT));
                    accGyroSkew = 0;
                }
            }
       }
    }

    nowCycles = getCycleCounter();
    schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

    if (!gyroEnabled || (schedLoopRemainingCycles > (int32_t)clockMicrosToCycles(CHECK_GUARD_MARGIN_US))) {
        currentTimeUs = micros();

        // Update task dynamic priorities
        for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
            if (task->attribute->staticPriority != TASK_PRIORITY_REALTIME) {
                // Task has checkFunc - event driven
                if (task->attribute->checkFunc) {
                    // Increase priority for event driven tasks
                    if (task->dynamicPriority > 0) {
                        task->taskAgePeriods = 1 + (cmpTimeUs(currentTimeUs, task->lastSignaledAtUs) / task->attribute->desiredPeriodUs);
                        task->dynamicPriority = 1 + task->attribute->staticPriority * task->taskAgePeriods;
                    } else if (task->attribute->checkFunc(currentTimeUs, cmpTimeUs(currentTimeUs, task->lastExecutedAtUs))) {
                        const uint32_t checkFuncExecutionTimeUs = cmpTimeUs(micros(), currentTimeUs);
                        checkFuncMovingSumExecutionTimeUs += checkFuncExecutionTimeUs - checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                        checkFuncMovingSumDeltaTimeUs += task->taskLatestDeltaTimeUs - checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                        checkFuncTotalExecutionTimeUs += checkFuncExecutionTimeUs;   // time consumed by scheduler + task
                        checkFuncMaxExecutionTimeUs = MAX(checkFuncMaxExecutionTimeUs, checkFuncExecutionTimeUs);
                        task->lastSignaledAtUs = currentTimeUs;
                        task->taskAgePeriods = 1;
                        task->dynamicPriority = 1 + task->attribute->staticPriority;
                    } else {
                        task->taskAgePeriods = 0;
                    }
                } else {
                    // Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
                    // Task age is calculated from last execution
                    task->taskAgePeriods = (cmpTimeUs(currentTimeUs, task->lastExecutedAtUs) / task->attribute->desiredPeriodUs);
                    if (task->taskAgePeriods > 0) {
                        task->dynamicPriority = 1 + task->attribute->staticPriority * task->taskAgePeriods;
                    }
                }

                if (task->dynamicPriority > selectedTaskDynamicPriority) {
                    timeDelta_t taskRequiredTimeUs = task->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
                    int32_t taskRequiredTimeCycles = (int32_t)clockMicrosToCycles((uint32_t)taskRequiredTimeUs);
                    // Allow a little extra time
                    taskRequiredTimeCycles += checkCycles + taskGuardCycles;

                    // If there's no time to run the task, discount it from prioritisation unless aged sufficiently
                    // Don't block the SERIAL task.
                    if ((taskRequiredTimeCycles < schedLoopRemainingCycles) ||
                        ((scheduleCount & SCHED_TASK_DEFER_MASK) == 0) ||
                        ((task - tasks) == TASK_SERIAL)) {
                        selectedTaskDynamicPriority = task->dynamicPriority;
                        selectedTask = task;
                    }
                }
            }

        }

        // The number of cycles taken to run the checkers is quite consistent with some higher spikes, but
        // that doesn't defeat its use
        checkCycles = cmpTimeCycles(getCycleCounter(), nowCycles);

        if (selectedTask) {
            // Recheck the available time as checkCycles is only approximate
            timeDelta_t taskRequiredTimeUs = selectedTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
#if defined(USE_LATE_TASK_STATISTICS)
            selectedTask->execTime = taskRequiredTimeUs;
#endif
            int32_t taskRequiredTimeCycles = (int32_t)clockMicrosToCycles((uint32_t)taskRequiredTimeUs);

            nowCycles = getCycleCounter();
            schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

            // Allow a little extra time
            taskRequiredTimeCycles += taskGuardCycles;

            if (!gyroEnabled || (taskRequiredTimeCycles < schedLoopRemainingCycles)) {
                uint32_t antipatedEndCycles = nowCycles + taskRequiredTimeCycles;
                taskExecutionTimeUs += schedulerExecuteTask(selectedTask, currentTimeUs);
                nowCycles = getCycleCounter();
                int32_t cyclesOverdue = cmpTimeCycles(nowCycles, antipatedEndCycles);

#if defined(USE_LATE_TASK_STATISTICS)
                if (cyclesOverdue > 0) {
                    if ((currentTask - tasks) != TASK_SERIAL) {
                        DEBUG_SET(DEBUG_SCHEDULER_DETERMINISM, 1, currentTask - tasks);
                        DEBUG_SET(DEBUG_SCHEDULER_DETERMINISM, 2, clockCyclesTo10thMicros(cyclesOverdue));
                        currentTask->lateCount++;
                        lateTaskCount++;
                        lateTaskTotal += cyclesOverdue;
                    }
                }
#endif  // USE_LATE_TASK_STATISTICS

                if ((currentTask - tasks) == TASK_RX) {
                    skippedRxAttempts = 0;
                }
#ifdef USE_OSD
                else if ((currentTask - tasks) == TASK_OSD) {
                    skippedOSDAttempts = 0;
                }
#endif

                if ((cyclesOverdue > 0) || (-cyclesOverdue < taskGuardMinCycles)) {
                    if (taskGuardCycles < taskGuardMaxCycles) {
                        taskGuardCycles += taskGuardDeltaUpCycles;
                    }
                } else if (taskGuardCycles > taskGuardMinCycles) {
                    taskGuardCycles -= taskGuardDeltaDownCycles;
                }
#if defined(USE_LATE_TASK_STATISTICS)
                taskCount++;
#endif  // USE_LATE_TASK_STATISTICS
            } else if ((selectedTask->taskAgePeriods > TASK_AGE_EXPEDITE_COUNT) ||
#ifdef USE_OSD
                       (((selectedTask - tasks) == TASK_OSD) && (TASK_AGE_EXPEDITE_OSD != 0) && (++skippedOSDAttempts > TASK_AGE_EXPEDITE_OSD)) ||
#endif
                       (((selectedTask - tasks) == TASK_RX) && (TASK_AGE_EXPEDITE_RX != 0) && (++skippedRxAttempts > TASK_AGE_EXPEDITE_RX))) {
                // If a task has been unable to run, then reduce it's recorded estimated run time to ensure
                // it's ultimate scheduling
                selectedTask->anticipatedExecutionTime *= TASK_AGE_EXPEDITE_SCALE;
            }
        }
    }

#if defined(UNIT_TEST)
    readSchedulerLocals(selectedTask, selectedTaskDynamicPriority);
    UNUSED(taskExecutionTimeUs);
#else
    DEBUG_SET(DEBUG_SCHEDULER, 2, micros() - schedulerStartTimeUs - taskExecutionTimeUs); // time spent in scheduler
#endif

    scheduleCount++;
}

void schedulerEnableGyro(void)
{
    gyroEnabled = true;
}

uint16_t getAverageSystemLoadPercent(void)
{
    return averageSystemLoadPercent;
}

float schedulerGetCycleTimeMultiplier(void)
{
    return (float)clockMicrosToCycles(getTask(TASK_GYRO)->attribute->desiredPeriodUs) / desiredPeriodCycles;
}
