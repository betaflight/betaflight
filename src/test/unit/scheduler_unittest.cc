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
    #include "drivers/accgyro/accgyro.h"
    #include "platform.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/scheduler.h"
    #include "scheduler/scheduler.h"

    PG_REGISTER_WITH_RESET_TEMPLATE(schedulerConfig_t, schedulerConfig, PG_SCHEDULER_CONFIG, 0);

    PG_RESET_TEMPLATE(schedulerConfig_t, schedulerConfig,
        .rxRelaxDeterminism = 25,
        .osdRelaxDeterminism = 25,
    );
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

const int TEST_GYRO_SAMPLE_HZ = 8000;
const int TEST_GYRO_SAMPLE_TIME = 10;
const int TEST_FILTERING_TIME = 40;
const int TEST_PID_LOOP_TIME = 58;
const int TEST_UPDATE_ACCEL_TIME = 32;
const int TEST_UPDATE_ATTITUDE_TIME = 28;
const int TEST_HANDLE_SERIAL_TIME = 30;
const int TEST_UPDATE_BATTERY_TIME = 1;
const int TEST_UPDATE_RX_CHECK_TIME = 34;
const int TEST_UPDATE_RX_MAIN_TIME = 1;
const int TEST_IMU_UPDATE_TIME = 5;
const int TEST_DISPATCH_TIME = 200;
const int TEST_UPDATE_OSD_CHECK_TIME = 5;
const int TEST_UPDATE_OSD_TIME = 30;

#define TASK_COUNT_UNITTEST (TASK_BATTERY_VOLTAGE + 1)
#define TASK_PERIOD_HZ(hz) (1000000 / (hz))

extern "C" {
    task_t * unittest_scheduler_selectedTask;
    uint8_t unittest_scheduler_selectedTaskDynPrio;
    timeDelta_t unittest_scheduler_taskRequiredTimeUs;
    bool taskGyroRan = false;
    bool taskFilterRan = false;
    bool taskPidRan = false;
    bool taskFilterReady = false;
    bool taskPidReady = false;
    uint8_t activePidLoopDenom = 1;

    int16_t debug[1];
    uint8_t debugMode = 0;

    void rxFrameCheck(timeUs_t, timeDelta_t) {}

    // set up micros() to simulate time
    uint32_t simulatedTime = 0;
    uint32_t micros(void) { return simulatedTime; }
    uint32_t millis(void) { return simulatedTime/1000; } // Note simplistic mapping suitable only for short unit tests
    int32_t clockCyclesToMicros(int32_t x) { return x/10;}
    int32_t clockCyclesTo10thMicros(int32_t x) { return x;}
    uint32_t clockMicrosToCycles(uint32_t x) { return x*10;}
    uint32_t getCycleCounter(void) {return simulatedTime * 10;}

    // set up tasks to take a simulated representative time to execute
    bool gyroFilterReady(void) { return taskFilterReady; }
    gyroDev_t gyro {
        .gyroModeSPI = GYRO_EXTI_NO_INT
    };
    gyroDev_t *gyroActiveDev(void) { return &gyro; }
    bool pidLoopReady(void) { return taskPidReady; }
    void failsafeCheckDataFailurePeriod(void) {}
    void failsafeUpdateState(void) {}
    void taskGyroSample(timeUs_t) { simulatedTime += TEST_GYRO_SAMPLE_TIME; taskGyroRan = true; }
    void taskFiltering(timeUs_t) { simulatedTime += TEST_FILTERING_TIME; taskFilterRan = true; }
    void taskMainPidLoop(timeUs_t) { simulatedTime += TEST_PID_LOOP_TIME; taskPidRan = true; }
    void taskUpdateAccelerometer(timeUs_t) { simulatedTime += TEST_UPDATE_ACCEL_TIME; }
    void taskHandleSerial(timeUs_t) { simulatedTime += TEST_HANDLE_SERIAL_TIME; }
    void taskUpdateBatteryVoltage(timeUs_t) { simulatedTime += TEST_UPDATE_BATTERY_TIME; }
    bool rxUpdateCheck(timeUs_t, timeDelta_t) { simulatedTime += TEST_UPDATE_RX_CHECK_TIME; return false; }
    void taskUpdateRxMain(timeUs_t) { simulatedTime += TEST_UPDATE_RX_MAIN_TIME; }
    void imuUpdateAttitude(timeUs_t) { simulatedTime += TEST_IMU_UPDATE_TIME; }
    void dispatchProcess(timeUs_t) { simulatedTime += TEST_DISPATCH_TIME; }
    bool osdUpdateCheck(timeUs_t, timeDelta_t) { simulatedTime += TEST_UPDATE_OSD_CHECK_TIME; return false; }
    void osdUpdate(timeUs_t) { simulatedTime += TEST_UPDATE_OSD_TIME; }

    void resetGyroTaskTestFlags(void) {
        taskGyroRan = false;
        taskFilterRan = false;
        taskPidRan = false;
        taskFilterReady = false;
        taskPidReady = false;
    }

    extern int taskQueueSize;
    extern task_t* taskQueueArray[];

    extern void queueClear(void);
    extern bool queueContains(task_t *task);
    extern bool queueAdd(task_t *task);
    extern bool queueRemove(task_t *task);
    extern task_t *queueFirst(void);
    extern task_t *queueNext(void);

    task_attribute_t task_attributes[TASK_COUNT] = {
        [TASK_SYSTEM] = {
            .taskName = "SYSTEM",
            .taskFunc = taskSystemLoad,
            .desiredPeriodUs = TASK_PERIOD_HZ(10),
            .staticPriority = TASK_PRIORITY_MEDIUM_HIGH,
        },
        [TASK_GYRO] = {
            .taskName = "GYRO",
            .taskFunc = taskGyroSample,
            .desiredPeriodUs = TASK_PERIOD_HZ(TEST_GYRO_SAMPLE_HZ),
            .staticPriority = TASK_PRIORITY_REALTIME,
        },
        [TASK_FILTER] = {
            .taskName = "FILTER",
            .taskFunc = taskFiltering,
            .desiredPeriodUs = TASK_PERIOD_HZ(4000),
            .staticPriority = TASK_PRIORITY_REALTIME,
        },
        [TASK_PID] = {
            .taskName = "PID",
            .taskFunc = taskMainPidLoop,
            .desiredPeriodUs = TASK_PERIOD_HZ(4000),
            .staticPriority = TASK_PRIORITY_REALTIME,
        },
        [TASK_ACCEL] = {
            .taskName = "ACCEL",
            .taskFunc = taskUpdateAccelerometer,
            .desiredPeriodUs = TASK_PERIOD_HZ(1000),
            .staticPriority = TASK_PRIORITY_MEDIUM,
        },
        [TASK_ATTITUDE] = {
            .taskName = "ATTITUDE",
            .taskFunc = imuUpdateAttitude,
            .desiredPeriodUs = TASK_PERIOD_HZ(100),
            .staticPriority = TASK_PRIORITY_MEDIUM,
        },
        [TASK_RX] = {
            .taskName = "RX",
            .checkFunc = rxUpdateCheck,
            .taskFunc = taskUpdateRxMain,
            .desiredPeriodUs = TASK_PERIOD_HZ(50),
            .staticPriority = TASK_PRIORITY_HIGH,
        },
        [TASK_SERIAL] = {
            .taskName = "SERIAL",
            .taskFunc = taskHandleSerial,
            .desiredPeriodUs = TASK_PERIOD_HZ(100),
            .staticPriority = TASK_PRIORITY_LOW,
        },
        [TASK_DISPATCH] = {
            .taskName = "DISPATCH",
            .taskFunc = dispatchProcess,
            .desiredPeriodUs = TASK_PERIOD_HZ(1000),
            .staticPriority = TASK_PRIORITY_HIGH,
        },
        [TASK_BATTERY_VOLTAGE] = {
            .taskName = "BATTERY_VOLTAGE",
            .taskFunc = taskUpdateBatteryVoltage,
            .desiredPeriodUs = TASK_PERIOD_HZ(50),
            .staticPriority = TASK_PRIORITY_MEDIUM,
        },
        [TASK_OSD] = {
            .taskName = "OSD",
            .checkFunc = osdUpdateCheck,
            .taskFunc = osdUpdate,
            .desiredPeriodUs = TASK_PERIOD_HZ(12),
            .staticPriority = TASK_PRIORITY_LOW,
        }
    };

    task_t tasks[TASK_COUNT];

    task_t *getTask(unsigned taskId)
    {
        return &tasks[taskId];
    }
}

TEST(SchedulerUnittest, SetupTasks)
{
    for (int i = 0; i < TASK_COUNT; ++i) {
        tasks[i].attribute = &task_attributes[i];
    }
}


TEST(SchedulerUnittest, TestPriorites)
{
    EXPECT_EQ(TASK_PRIORITY_MEDIUM_HIGH, tasks[TASK_SYSTEM].attribute->staticPriority);
    EXPECT_EQ(TASK_PRIORITY_REALTIME, tasks[TASK_GYRO].attribute->staticPriority);
    EXPECT_EQ(TASK_PRIORITY_MEDIUM, tasks[TASK_ACCEL].attribute->staticPriority);
    EXPECT_EQ(TASK_PRIORITY_LOW, tasks[TASK_SERIAL].attribute->staticPriority);
    EXPECT_EQ(TASK_PRIORITY_MEDIUM, tasks[TASK_BATTERY_VOLTAGE].attribute->staticPriority);
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

task_t *deadBeefPtr = reinterpret_cast<task_t*>(0xDEADBEEF);

TEST(SchedulerUnittest, TestQueue)
{
    queueClear();
    taskQueueArray[TASK_COUNT + 1] = deadBeefPtr;

    queueAdd(&tasks[TASK_SYSTEM]); // TASK_PRIORITY_MEDIUM_HIGH
    EXPECT_EQ(1, taskQueueSize);
    EXPECT_EQ(&tasks[TASK_SYSTEM], queueFirst());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);

    queueAdd(&tasks[TASK_SERIAL]); // TASK_PRIORITY_LOW
    EXPECT_EQ(2, taskQueueSize);
    EXPECT_EQ(&tasks[TASK_SYSTEM], queueFirst());
    EXPECT_EQ(&tasks[TASK_SERIAL], queueNext());
    EXPECT_EQ(NULL, queueNext());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);

    queueAdd(&tasks[TASK_BATTERY_VOLTAGE]); // TASK_PRIORITY_MEDIUM
    EXPECT_EQ(3, taskQueueSize);
    EXPECT_EQ(&tasks[TASK_SYSTEM], queueFirst());
    EXPECT_EQ(&tasks[TASK_BATTERY_VOLTAGE], queueNext());
    EXPECT_EQ(&tasks[TASK_SERIAL], queueNext());
    EXPECT_EQ(NULL, queueNext());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);

    queueAdd(&tasks[TASK_RX]); // TASK_PRIORITY_HIGH
    EXPECT_EQ(4, taskQueueSize);
    EXPECT_EQ(&tasks[TASK_RX], queueFirst());
    EXPECT_EQ(&tasks[TASK_SYSTEM], queueNext());
    EXPECT_EQ(&tasks[TASK_BATTERY_VOLTAGE], queueNext());
    EXPECT_EQ(&tasks[TASK_SERIAL], queueNext());
    EXPECT_EQ(NULL, queueNext());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);

    queueRemove(&tasks[TASK_SYSTEM]); // TASK_PRIORITY_HIGH
    EXPECT_EQ(3, taskQueueSize);
    EXPECT_EQ(&tasks[TASK_RX], queueFirst());
    EXPECT_EQ(&tasks[TASK_BATTERY_VOLTAGE], queueNext());
    EXPECT_EQ(&tasks[TASK_SERIAL], queueNext());
    EXPECT_EQ(NULL, queueNext());
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);
}

TEST(SchedulerUnittest, TestQueueAddAndRemove)
{
    queueClear();
    taskQueueArray[TASK_COUNT + 1] = deadBeefPtr;

    // fill up the queue
    for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
        const bool added = queueAdd(&tasks[taskId]);
        EXPECT_TRUE(added);
        EXPECT_EQ(taskId + 1, taskQueueSize);
        EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]);
    }
    // double check end of queue
    EXPECT_EQ(TASK_COUNT, taskQueueSize);
    EXPECT_NE(static_cast<task_t*>(0), taskQueueArray[TASK_COUNT - 1]); // last item was indeed added to queue
    EXPECT_EQ(NULL, taskQueueArray[TASK_COUNT]); // null pointer at end of queue is preserved
    EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT + 1]); // there hasn't been an out by one error

    // and empty it again
    for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
        const bool removed = queueRemove(&tasks[taskId]);
        EXPECT_TRUE(removed);
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
        if (tasks[taskId].attribute->taskFunc) {
            setTaskEnabled(static_cast<taskId_e>(taskId), true);
            enqueuedTasks++;
            EXPECT_EQ(enqueuedTasks, taskQueueSize);
            EXPECT_EQ(deadBeefPtr, taskQueueArray[TASK_COUNT_UNITTEST + 1]);
        }
    }

    EXPECT_NE(static_cast<task_t*>(0), taskQueueArray[enqueuedTasks - 1]);
    const task_t *lastTaskPrev = taskQueueArray[enqueuedTasks - 1];
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

    taskInfo_t taskInfo;
    getTaskInfo(static_cast<taskId_e>(enqueuedTasks + 1), &taskInfo);
    EXPECT_FALSE(taskInfo.isEnabled);
    setTaskEnabled(static_cast<taskId_e>(enqueuedTasks), true);
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
    EXPECT_EQ(&tasks[TASK_SYSTEM], queueFirst());
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
    // disable all tasks except TASK_ACCEL
    for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
        setTaskEnabled(static_cast<taskId_e>(taskId), false);
    }
    setTaskEnabled(TASK_ACCEL, true);
    tasks[TASK_ACCEL].lastExecutedAtUs = 1000;
    tasks[TASK_ACCEL].lastStatsAtUs = 1000;
    simulatedTime = 2050;
    // run the scheduler and check the task has executed
    scheduler();
    EXPECT_NE(unittest_scheduler_selectedTask, static_cast<task_t*>(0));
    EXPECT_EQ(unittest_scheduler_selectedTask, &tasks[TASK_ACCEL]);
    EXPECT_EQ(1050, tasks[TASK_ACCEL].taskLatestDeltaTimeUs);
    EXPECT_EQ(2050, tasks[TASK_ACCEL].lastExecutedAtUs);
    EXPECT_EQ(TEST_UPDATE_ACCEL_TIME, tasks[TASK_ACCEL].totalExecutionTimeUs);
    // task has run, so its dynamic priority should have been set to zero
    EXPECT_EQ(0, tasks[TASK_GYRO].dynamicPriority);
}

TEST(SchedulerUnittest, TestTwoTasks)
{
    // disable all tasks except TASK_ACCEL and TASK_ATTITUDE
    for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
        setTaskEnabled(static_cast<taskId_e>(taskId), false);
    }
    setTaskEnabled(TASK_ACCEL, true);
    setTaskEnabled(TASK_ATTITUDE, true);

    // set it up so that TASK_ACCEL ran just before TASK_ATTITUDE
    static const uint32_t startTime = 4000;
    simulatedTime = startTime;
    tasks[TASK_ACCEL].lastExecutedAtUs = simulatedTime;
    tasks[TASK_ATTITUDE].lastExecutedAtUs = tasks[TASK_ACCEL].lastExecutedAtUs - TEST_UPDATE_ATTITUDE_TIME;
    EXPECT_EQ(0, tasks[TASK_ATTITUDE].taskAgePeriods);
    // run the scheduler
    scheduler();
    // no tasks should have run, since neither task's desired time has elapsed
    EXPECT_EQ(static_cast<task_t*>(0), unittest_scheduler_selectedTask);

    // NOTE:
    // TASK_ACCEL    desiredPeriodUs is 1000 microseconds
    // TASK_ATTITUDE desiredPeriodUs is 10000 microseconds
    // 500 microseconds later
    simulatedTime += 500;
    // no tasks should run, since neither task's desired time has elapsed
    scheduler();
    EXPECT_EQ(static_cast<task_t*>(0), unittest_scheduler_selectedTask);

    // 500 microseconds later, TASK_ACCEL desiredPeriodUs has elapsed
    simulatedTime += 500;
    // TASK_ACCEL should now run
    scheduler();
    EXPECT_EQ(&tasks[TASK_ACCEL], unittest_scheduler_selectedTask);
    EXPECT_EQ(5000 + TEST_UPDATE_ACCEL_TIME, simulatedTime);

    simulatedTime += 1000 - TEST_UPDATE_ACCEL_TIME;
    scheduler();
    // TASK_ACCEL should run again
    EXPECT_EQ(&tasks[TASK_ACCEL], unittest_scheduler_selectedTask);

    scheduler();
    // No task should have run
    EXPECT_EQ(static_cast<task_t*>(0), unittest_scheduler_selectedTask);

    simulatedTime = startTime + 10500; // TASK_ACCEL and TASK_ATTITUDE desiredPeriodUss have elapsed
    // of the two TASK_ACCEL should run first
    scheduler();
    EXPECT_EQ(&tasks[TASK_ACCEL], unittest_scheduler_selectedTask);
    // and finally TASK_ATTITUDE should now run
    scheduler();
    EXPECT_EQ(&tasks[TASK_ATTITUDE], unittest_scheduler_selectedTask);
}

TEST(SchedulerUnittest, TestPriorityBump)
{
    // disable all tasks except TASK_ACCEL and TASK_ATTITUDE
    for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
        setTaskEnabled(static_cast<taskId_e>(taskId), false);
    }
    setTaskEnabled(TASK_ACCEL, true);
    setTaskEnabled(TASK_DISPATCH, true);

    // Both tasks have an update rate of 1kHz, but TASK_DISPATCH has TASK_PRIORITY_HIGH whereas TASK_ACCEL has TASK_PRIORITY_MEDIUM
    static const uint32_t startTime = 4000;
    simulatedTime = startTime;
    tasks[TASK_ACCEL].lastExecutedAtUs = simulatedTime;
    tasks[TASK_DISPATCH].lastExecutedAtUs = tasks[TASK_ACCEL].lastExecutedAtUs;
    EXPECT_EQ(0, tasks[TASK_DISPATCH].taskAgePeriods);

    // Set expectation for execution time of TEST_DISPATCH_TIME us
    tasks[TASK_DISPATCH].anticipatedExecutionTime = TEST_DISPATCH_TIME << TASK_EXEC_TIME_SHIFT;

    // run the scheduler
    scheduler();
    // no tasks should have run, since neither task's desired time has elapsed
    EXPECT_EQ(static_cast<task_t*>(0), unittest_scheduler_selectedTask);

    // NOTE:
    // TASK_ACCEL    desiredPeriodUs is 1000 microseconds
    // TASK_DISPATCH desiredPeriodUs is 1000 microseconds
    // 500 microseconds later
    simulatedTime += 500;
    // no tasks should run, since neither task's desired time has elapsed
    scheduler();
    EXPECT_EQ(static_cast<task_t*>(0), unittest_scheduler_selectedTask);

    // 500 microseconds later, 1000 desiredPeriodUs has elapsed
    simulatedTime += 500;
    // TASK_ACCEL should now run as there is not enough time to run the higher priority TASK_DISPATCH
    scheduler();
    EXPECT_EQ(&tasks[TASK_ACCEL], unittest_scheduler_selectedTask);
    EXPECT_EQ(5000 + TEST_UPDATE_ACCEL_TIME, simulatedTime);

    simulatedTime += 1000 - TEST_UPDATE_ACCEL_TIME;
    // TASK_ACCEL should now run as there is not enough time to run the higher priority TASK_DISPATCH
    scheduler();
    EXPECT_EQ(&tasks[TASK_ACCEL], unittest_scheduler_selectedTask);
    EXPECT_EQ(6000 + TEST_UPDATE_ACCEL_TIME, simulatedTime);

    simulatedTime += 1000 - TEST_UPDATE_ACCEL_TIME;
    // TASK_ACCEL should now run as there is not enough time to run the higher priority TASK_DISPATCH
    scheduler();
    EXPECT_EQ(&tasks[TASK_ACCEL], unittest_scheduler_selectedTask);
    EXPECT_EQ(7000 + TEST_UPDATE_ACCEL_TIME, simulatedTime);

    simulatedTime += 1000 - TEST_UPDATE_ACCEL_TIME;
    // TASK_ACCEL should now run as there is not enough time to run the higher priority TASK_DISPATCH
    scheduler();
    EXPECT_EQ(&tasks[TASK_ACCEL], unittest_scheduler_selectedTask);
    EXPECT_EQ(8000 + TEST_UPDATE_ACCEL_TIME, simulatedTime);

    simulatedTime += 1000 - TEST_UPDATE_ACCEL_TIME;
    // TASK_ACCEL should now run as there is not enough time to run the higher priority TASK_DISPATCH
    scheduler();
    EXPECT_EQ(&tasks[TASK_ACCEL], unittest_scheduler_selectedTask);
    EXPECT_EQ(9000 + TEST_UPDATE_ACCEL_TIME, simulatedTime);

    // TASK_DISPATCH has aged whilst not being run
    EXPECT_EQ(5, tasks[TASK_DISPATCH].taskAgePeriods);
    simulatedTime += 1000 - TEST_UPDATE_ACCEL_TIME;
    // TASK_TASK_DISPATCH should now run as the scheduler is on its eighth loop. Note that this is affected by prior test count.
    scheduler();
    EXPECT_EQ(&tasks[TASK_DISPATCH], unittest_scheduler_selectedTask);
    EXPECT_EQ(10000 + TEST_DISPATCH_TIME, simulatedTime);
    // TASK_DISPATCH still hasn't been executed
    EXPECT_EQ(6, tasks[TASK_DISPATCH].taskAgePeriods);

    simulatedTime += 1000 - TEST_DISPATCH_TIME;
    // TASK_ACCEL should now run again as there is not enough time to run the higher priority TASK_DISPATCH
    scheduler();
    EXPECT_EQ(&tasks[TASK_ACCEL], unittest_scheduler_selectedTask);
    EXPECT_EQ(11000 + TEST_UPDATE_ACCEL_TIME, simulatedTime);
}

TEST(SchedulerUnittest, TestGyroTask)
{
    static const uint32_t startTime = 4000;

    // enable the gyro
    schedulerEnableGyro();

    // disable all tasks except TASK_GYRO, TASK_FILTER and TASK_PID
    for (int taskId = 0; taskId < TASK_COUNT; ++taskId) {
        setTaskEnabled(static_cast<taskId_e>(taskId), false);
    }
    setTaskEnabled(TASK_GYRO, true);
    setTaskEnabled(TASK_FILTER, true);
    setTaskEnabled(TASK_PID, true);

    // First set it up so TASK_GYRO just ran
    simulatedTime = startTime;
    tasks[TASK_GYRO].lastExecutedAtUs = simulatedTime;
    // reset the flags
    resetGyroTaskTestFlags();

    // run the scheduler
    scheduler();
    // no tasks should have run
    EXPECT_EQ(static_cast<task_t*>(0), unittest_scheduler_selectedTask);
    // also the gyro, filter and PID task indicators should be false
    EXPECT_FALSE(taskGyroRan);
    EXPECT_FALSE(taskFilterRan);
    EXPECT_FALSE(taskPidRan);

    /* Test the gyro task running but not triggering the filtering or PID */
    // set the TASK_GYRO last executed time to be one period earlier
    simulatedTime = startTime;
    tasks[TASK_GYRO].lastExecutedAtUs = simulatedTime - TASK_PERIOD_HZ(TEST_GYRO_SAMPLE_HZ);

    // reset the flags
    resetGyroTaskTestFlags();

    // run the scheduler
    scheduler();

    // the gyro task indicator should be true and the TASK_FILTER and TASK_PID indicators should be false
    EXPECT_TRUE(taskGyroRan);
    EXPECT_FALSE(taskFilterRan);
    EXPECT_FALSE(taskPidRan);
    // expect that no other tasks other than TASK_GYRO should have run
    EXPECT_EQ(static_cast<task_t*>(0), unittest_scheduler_selectedTask);

    /* Test the gyro task running and triggering the filtering task */
    // set the TASK_GYRO last executed time to be one period earlier
    simulatedTime = startTime;
    tasks[TASK_GYRO].lastExecutedAtUs = simulatedTime - TASK_PERIOD_HZ(TEST_GYRO_SAMPLE_HZ);

    // reset the flags
    resetGyroTaskTestFlags();
    taskFilterReady = true;

    // run the scheduler
    scheduler();
    // the gyro and filter task indicators should be true and TASK_PID indicator should be false
    EXPECT_TRUE(taskGyroRan);
    EXPECT_TRUE(taskFilterRan);
    EXPECT_FALSE(taskPidRan);
    // expect that no other tasks other tasks should have run
    EXPECT_EQ(static_cast<task_t*>(0), unittest_scheduler_selectedTask);

    /* Test the gyro task running and triggering the PID task */
    // set the TASK_GYRO last executed time to be one period earlier
    simulatedTime = startTime;
    tasks[TASK_GYRO].lastExecutedAtUs = simulatedTime - TASK_PERIOD_HZ(TEST_GYRO_SAMPLE_HZ);

    // reset the flags
    resetGyroTaskTestFlags();
    taskPidReady = true;

    // run the scheduler
    scheduler();
    // the gyro and PID task indicators should be true and TASK_FILTER indicator should be false
    EXPECT_TRUE(taskGyroRan);
    EXPECT_FALSE(taskFilterRan);
    EXPECT_TRUE(taskPidRan);
    // expect that no other tasks other tasks should have run
    EXPECT_EQ(static_cast<task_t*>(0), unittest_scheduler_selectedTask);
}

