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

#include <stdint.h>

extern "C" {
    #include "platform.h"
    #include "scheduler/scheduler.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

const int TEST_PID_LOOP_TIME = 650;
const int TEST_UPDATE_ACCEL_TIME = 192;
const int TEST_HANDLE_SERIAL_TIME = 30;
const int TEST_UPDATE_BATTERY_TIME = 1;
const int TEST_UPDATE_RX_CHECK_TIME = 34;
const int TEST_UPDATE_RX_MAIN_TIME = 1;
const int TEST_IMU_UPDATE_TIME = 5;
const int TEST_DISPATCH_TIME = 1;

#define TASK_COUNT_UNITTEST (TASK_BATTERY_VOLTAGE + 1)
#define TASK_PERIOD_HZ(hz) (1000000 / (hz))

extern "C" {
    cfTask_t * unittest_scheduler_selectedTask;
    uint8_t unittest_scheduler_selectedTaskDynPrio;
    uint16_t unittest_scheduler_waitingTasks;

    // set up micros() to simulate time
    uint32_t simulatedTime = 0;
    uint32_t micros(void) { return simulatedTime; }

    // set up tasks to take a simulated representative time to execute
    void taskMainPidLoop(timeUs_t) { simulatedTime += TEST_PID_LOOP_TIME; }
    void taskUpdateAccelerometer(timeUs_t) { simulatedTime += TEST_UPDATE_ACCEL_TIME; }
    void taskHandleSerial(timeUs_t) { simulatedTime += TEST_HANDLE_SERIAL_TIME; }
    void taskUpdateBatteryVoltage(timeUs_t) { simulatedTime += TEST_UPDATE_BATTERY_TIME; }
    bool rxUpdateCheck(timeUs_t, timeDelta_t) { simulatedTime += TEST_UPDATE_RX_CHECK_TIME; return false; }
    void taskUpdateRxMain(timeUs_t) { simulatedTime += TEST_UPDATE_RX_MAIN_TIME; }
    void imuUpdateAttitude(timeUs_t) { simulatedTime += TEST_IMU_UPDATE_TIME; }
    void dispatchProcess(timeUs_t) { simulatedTime += TEST_DISPATCH_TIME; }

    extern int taskQueueSize;
    extern cfTask_t* taskQueueArray[];

    extern void queueClear(void);
    extern bool queueContains(cfTask_t *task);
    extern bool queueAdd(cfTask_t *task);
    extern bool queueRemove(cfTask_t *task);
    extern cfTask_t *queueFirst(void);
    extern cfTask_t *queueNext(void);

    cfTask_t cfTasks[TASK_COUNT] = {
        [TASK_SYSTEM] = {
            .taskName = "SYSTEM",
            .taskFunc = taskSystemLoad,
            .desiredPeriod = TASK_PERIOD_HZ(10),
            .staticPriority = TASK_PRIORITY_MEDIUM_HIGH,
        },
        [TASK_GYROPID] = {
            .taskName = "PID",
            .subTaskName = "GYRO",
            .taskFunc = taskMainPidLoop,
            .desiredPeriod = 1000,
            .staticPriority = TASK_PRIORITY_REALTIME,
        },
        [TASK_ACCEL] = {
            .taskName = "ACCEL",
            .taskFunc = taskUpdateAccelerometer,
            .desiredPeriod = 10000,
            .staticPriority = TASK_PRIORITY_MEDIUM,
        },
        [TASK_ATTITUDE] = {
            .taskName = "ATTITUDE",
            .taskFunc = imuUpdateAttitude,
            .desiredPeriod = TASK_PERIOD_HZ(100),
            .staticPriority = TASK_PRIORITY_MEDIUM,
        },
        [TASK_RX] = {
            .taskName = "RX",
            .checkFunc = rxUpdateCheck,
            .taskFunc = taskUpdateRxMain,
            .desiredPeriod = TASK_PERIOD_HZ(50),
            .staticPriority = TASK_PRIORITY_HIGH,
        },
        [TASK_SERIAL] = {
            .taskName = "SERIAL",
            .taskFunc = taskHandleSerial,
            .desiredPeriod = TASK_PERIOD_HZ(100),
            .staticPriority = TASK_PRIORITY_LOW,
        },
        [TASK_DISPATCH] = {
            .taskName = "DISPATCH",
            .taskFunc = dispatchProcess,
            .desiredPeriod = TASK_PERIOD_HZ(1000),
            .staticPriority = TASK_PRIORITY_HIGH,
        },
        [TASK_BATTERY_VOLTAGE] = {
            .taskName = "BATTERY_VOLTAGE",
            .taskFunc = taskUpdateBatteryVoltage,
            .desiredPeriod = TASK_PERIOD_HZ(50),
            .staticPriority = TASK_PRIORITY_MEDIUM,
        }
    };
}

TEST(SchedulerUnittest, TestPriorites)
{
    EXPECT_EQ(TASK_PRIORITY_MEDIUM_HIGH, cfTasks[TASK_SYSTEM].staticPriority);
    EXPECT_EQ(TASK_PRIORITY_REALTIME, cfTasks[TASK_GYROPID].staticPriority);
    EXPECT_EQ(TASK_PRIORITY_MEDIUM, cfTasks[TASK_ACCEL].staticPriority);
    EXPECT_EQ(TASK_PRIORITY_LOW, cfTasks[TASK_SERIAL].staticPriority);
    EXPECT_EQ(TASK_PRIORITY_MEDIUM, cfTasks[TASK_BATTERY_VOLTAGE].staticPriority);
}

TEST(SchedulerUnittest, TestQueueInit)
{
    queueClear();
    EXPECT_EQ(0, taskQueueSize);
    EXPECT_EQ(0, queueFirst());
    EXPECT_EQ(0, queueNext());
    for (int ii = 0; ii <= TASK_COUNT; ++ii) {
        EXPECT_EQ(0, taskQueueArray[ii]);
    }
}

cfTask_t *deadBeefPtr = reinterpret_cast<cfTask_t*>(0xDEADBEEF);

TEST(SchedulerUnittest, TestQueue)
{
    queueClear();
    taskQueueArray[TASK_COUNT + 1] = deadBeefPtr;

    queueAdd(&cfTasks[TASK_SYSTEM]); // TASK_PRIORITY_MEDIUM_HIGH
    EXPECT_EQ(1, taskQueueSize);
    EXPECT_EQ(&cfTasks[TASK_SYSTEM], queueFirst());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);

    queueAdd(&cfTasks[TASK_GYROPID]); // TASK_PRIORITY_REALTIME
    EXPECT_EQ(2, taskQueueSize);
    EXPECT_EQ(&cfTasks[TASK_GYROPID], queueFirst());
    EXPECT_EQ(&cfTasks[TASK_SYSTEM], queueNext());
    EXPECT_EQ(NULL, queueNext());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);

    queueAdd(&cfTasks[TASK_SERIAL]); // TASK_PRIORITY_LOW
    EXPECT_EQ(3, taskQueueSize);
    EXPECT_EQ(&cfTasks[TASK_GYROPID], queueFirst());
    EXPECT_EQ(&cfTasks[TASK_SYSTEM], queueNext());
    EXPECT_EQ(&cfTasks[TASK_SERIAL], queueNext());
    EXPECT_EQ(NULL, queueNext());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);

    queueAdd(&cfTasks[TASK_BATTERY_VOLTAGE]); // TASK_PRIORITY_MEDIUM
    EXPECT_EQ(4, taskQueueSize);
    EXPECT_EQ(&cfTasks[TASK_GYROPID], queueFirst());
    EXPECT_EQ(&cfTasks[TASK_SYSTEM], queueNext());
    EXPECT_EQ(&cfTasks[TASK_BATTERY_VOLTAGE], queueNext());
    EXPECT_EQ(&cfTasks[TASK_SERIAL], queueNext());
    EXPECT_EQ(NULL, queueNext());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);

    queueAdd(&cfTasks[TASK_RX]); // TASK_PRIORITY_HIGH
    EXPECT_EQ(5, taskQueueSize);
    EXPECT_EQ(&cfTasks[TASK_GYROPID], queueFirst());
    EXPECT_EQ(&cfTasks[TASK_RX], queueNext());
    EXPECT_EQ(&cfTasks[TASK_SYSTEM], queueNext());
    EXPECT_EQ(&cfTasks[TASK_BATTERY_VOLTAGE], queueNext());
    EXPECT_EQ(&cfTasks[TASK_SERIAL], queueNext());
    EXPECT_EQ(NULL, queueNext());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);

    queueRemove(&cfTasks[TASK_SYSTEM]); // TASK_PRIORITY_HIGH
    EXPECT_EQ(4, taskQueueSize);
    EXPECT_EQ(&cfTasks[TASK_GYROPID], queueFirst());
    EXPECT_EQ(&cfTasks[TASK_RX], queueNext());
    EXPECT_EQ(&cfTasks[TASK_BATTERY_VOLTAGE], queueNext());
    EXPECT_EQ(&cfTasks[TASK_SERIAL], queueNext());
    EXPECT_EQ(NULL, queueNext());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);
}

TEST(SchedulerUnittest, TestQueueAddAndRemove)
{
    queueClear();
    taskQueueArray[TASK_COUNT + 1] = deadBeefPtr;

    // fill up the queue
    for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
        const bool added = queueAdd(&cfTasks[taskId]);
        EXPECT_EQ(true, added);
        EXPECT_EQ(taskId + 1, taskQueueSize);
        EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);
    }

    // double check end of queue
    EXPECT_EQ(TASK_COUNT, taskQueueSize);
    EXPECT_NE(static_cast<cfTask_t*>(0), taskQueueArray[TASK_COUNT - 1]); // last item was indeed added to queue
    EXPECT_EQ(NULL, taskQueueArray[TASK_COUNT]); // null pointer at end of queue is preserved
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]); // there hasn't been an out by one error

    // and empty it again
    for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
        const bool removed = queueRemove(&cfTasks[taskId]);
        EXPECT_EQ(true, removed);
        EXPECT_EQ(TASK_COUNT - taskId - 1, taskQueueSize);
        EXPECT_EQ(NULL, taskQueueArray[TASK_COUNT - taskId]);
        EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);
    }

    // double check size and end of queue
    EXPECT_EQ(0, taskQueueSize); // queue is indeed empty
    EXPECT_EQ(NULL, taskQueueArray[0]); // there is a null pointer at the end of the queueu
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]); // no accidental overwrites past end of queue
}

TEST(SchedulerUnittest, TestQueueArray)
{
    // test there are no "out by one" errors or buffer overruns when items are added and removed
    queueClear();
    taskQueueArray[TASK_COUNT_UNITTEST + 1] = deadBeefPtr; // note, must set deadBeefPtr after queueClear

    unsigned enqueuedTasks = 0;
    EXPECT_EQ(enqueuedTasks, taskQueueSize);

    for (int taskId = 0; taskId < TASK_COUNT_UNITTEST - 1; ++taskId) {
        if (cfTasks[taskId].taskFunc) {
            setTaskEnabled(static_cast<cfTaskId_e>(taskId), true);
            enqueuedTasks++;
            EXPECT_EQ(enqueuedTasks, taskQueueSize);
            EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT_UNITTEST + 1]);
        }
    }

    EXPECT_NE(static_cast<cfTask_t*>(0), taskQueueArray[enqueuedTasks - 1]);
    const cfTask_t *lastTaskPrev = taskQueueArray[enqueuedTasks - 1];
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks + 1]);
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT_UNITTEST + 1]);

    setTaskEnabled(TASK_SYSTEM, false);
    EXPECT_EQ(enqueuedTasks - 1, taskQueueSize);
    EXPECT_EQ(lastTaskPrev, taskQueueArray[enqueuedTasks - 2]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks - 1]); // NULL at end of queue
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks + 1]);
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT_UNITTEST + 1]);

    taskQueueArray[enqueuedTasks - 1] = 0;
    setTaskEnabled(TASK_SYSTEM, true);
    EXPECT_EQ(enqueuedTasks, taskQueueSize);
    EXPECT_EQ(lastTaskPrev, taskQueueArray[enqueuedTasks - 1]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks + 1]);
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT_UNITTEST + 1]);

    cfTaskInfo_t taskInfo;
    getTaskInfo(static_cast<cfTaskId_e>(enqueuedTasks + 1), &taskInfo);
    EXPECT_EQ(false, taskInfo.isEnabled);
    setTaskEnabled(static_cast<cfTaskId_e>(enqueuedTasks), true);
    EXPECT_EQ(enqueuedTasks, taskQueueSize);
    EXPECT_EQ(lastTaskPrev, taskQueueArray[enqueuedTasks - 1]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks + 1]); // check no buffer overrun
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT_UNITTEST + 1]);

    setTaskEnabled(TASK_SYSTEM, false);
    EXPECT_EQ(enqueuedTasks - 1, taskQueueSize);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks + 1]);
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT_UNITTEST + 1]);

    setTaskEnabled(TASK_ACCEL, false);
    EXPECT_EQ(enqueuedTasks - 2, taskQueueSize);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks - 1]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks + 1]);
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT_UNITTEST + 1]);

    setTaskEnabled(TASK_BATTERY_VOLTAGE, false);
    EXPECT_EQ(enqueuedTasks - 2, taskQueueSize);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks - 2]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks - 1]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks]);
    EXPECT_EQ(NULL, taskQueueArray[enqueuedTasks + 1]);
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT_UNITTEST + 1]);
}

TEST(SchedulerUnittest, TestSchedulerInit)
{
    schedulerInit();
    EXPECT_EQ(1, taskQueueSize);
    EXPECT_EQ(&cfTasks[TASK_SYSTEM], queueFirst());
}

TEST(SchedulerUnittest, TestScheduleEmptyQueue)
{
    queueClear();
    simulatedTime = 4000;
    // run the with an empty queue
    scheduler();
    EXPECT_EQ(NULL, unittest_scheduler_selectedTask);
}

TEST(SchedulerUnittest, TestSingleTask)
{
    schedulerInit();
    // disable all tasks except TASK_GYROPID
    for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
        setTaskEnabled(static_cast<cfTaskId_e>(taskId), false);
    }
    setTaskEnabled(TASK_GYROPID, true);
    cfTasks[TASK_GYROPID].lastExecutedAt = 1000;
    simulatedTime = 4000;
    // run the scheduler and check the task has executed
    scheduler();
    EXPECT_NE(static_cast<cfTask_t*>(0), unittest_scheduler_selectedTask);
    EXPECT_EQ(&cfTasks[TASK_GYROPID], unittest_scheduler_selectedTask);
    EXPECT_EQ(3000, cfTasks[TASK_GYROPID].taskLatestDeltaTime);
    EXPECT_EQ(4000, cfTasks[TASK_GYROPID].lastExecutedAt);
    EXPECT_EQ(TEST_PID_LOOP_TIME, cfTasks[TASK_GYROPID].totalExecutionTime);
    // task has run, so its dynamic priority should have been set to zero
    EXPECT_EQ(0, cfTasks[TASK_GYROPID].dynamicPriority);
}

TEST(SchedulerUnittest, TestTwoTasks)
{
    // disable all tasks except TASK_GYROPID  and TASK_ACCEL
    for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
        setTaskEnabled(static_cast<cfTaskId_e>(taskId), false);
    }
    setTaskEnabled(TASK_ACCEL, true);
    setTaskEnabled(TASK_GYROPID, true);

    // set it up so that TASK_ACCEL ran just before TASK_GYROPID
    static const uint32_t startTime = 4000;
    simulatedTime = startTime;
    cfTasks[TASK_GYROPID].lastExecutedAt = simulatedTime;
    cfTasks[TASK_ACCEL].lastExecutedAt = cfTasks[TASK_GYROPID].lastExecutedAt - TEST_UPDATE_ACCEL_TIME;
    EXPECT_EQ(0, cfTasks[TASK_ACCEL].taskAgeCycles);
    // run the scheduler
    scheduler();
    // no tasks should have run, since neither task's desired time has elapsed
    EXPECT_EQ(static_cast<cfTask_t*>(0), unittest_scheduler_selectedTask);

    // NOTE:
    // TASK_GYROPID desiredPeriod is  1000 microseconds
    // TASK_ACCEL   desiredPeriod is 10000 microseconds
    // 500 microseconds later
    simulatedTime += 500;
    // no tasks should run, since neither task's desired time has elapsed
    scheduler();
    EXPECT_EQ(static_cast<cfTask_t*>(0), unittest_scheduler_selectedTask);
    EXPECT_EQ(0, unittest_scheduler_waitingTasks);

    // 500 microseconds later, TASK_GYROPID desiredPeriod has elapsed
    simulatedTime += 500;
    // TASK_GYROPID should now run
    scheduler();
    EXPECT_EQ(&cfTasks[TASK_GYROPID], unittest_scheduler_selectedTask);
    EXPECT_EQ(1, unittest_scheduler_waitingTasks);
    EXPECT_EQ(5000 + TEST_PID_LOOP_TIME, simulatedTime);

    simulatedTime += 1000 - TEST_PID_LOOP_TIME;
    scheduler();
    // TASK_GYROPID should run again
    EXPECT_EQ(&cfTasks[TASK_GYROPID], unittest_scheduler_selectedTask);

    scheduler();
    EXPECT_EQ(static_cast<cfTask_t*>(0), unittest_scheduler_selectedTask);
    EXPECT_EQ(0, unittest_scheduler_waitingTasks);

    simulatedTime = startTime + 10500; // TASK_GYROPID and TASK_ACCEL desiredPeriods have elapsed
    // of the two TASK_GYROPID should run first
    scheduler();
    EXPECT_EQ(&cfTasks[TASK_GYROPID], unittest_scheduler_selectedTask);
    // and finally TASK_ACCEL should now run
    scheduler();
    EXPECT_EQ(&cfTasks[TASK_ACCEL], unittest_scheduler_selectedTask);
}
