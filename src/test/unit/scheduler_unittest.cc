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
    #include <platform.h>
    #include "scheduler.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"
enum {
    pidLoopCheckerTime = 650,
    updateAccelerometerTime = 192,
    handleSerialTime = 30,
    updateBeeperTime = 1,
    updateBatteryTime = 1,
    updateRxCheckTime = 34,
    updateRxMainTime = 10,
    processGPSTime = 10,
    updateCompassTime = 195,
    updateBaroTime = 201,
    updateSonarTime = 10,
    calculateAltitudeTime = 154,
    updateDisplayTime = 10,
    telemetryTime = 10,
    ledStripTime = 10,
    transponderTime = 10
};

extern "C" {
    uint8_t unittest_scheduler_taskId;
    uint8_t unittest_scheduler_selectedTaskId;
    uint8_t unittest_scheduler_selectedTaskDynPrio;
    uint16_t unittest_scheduler_waitingTasks;
    uint32_t unittest_scheduler_timeToNextRealtimeTask;
    bool unittest_outsideRealtimeGuardInterval;

// set up micros() to simulate time
    uint32_t simulatedTime = 0;
    uint32_t micros(void) {return simulatedTime;}
// set up tasks to take a simulated representative time to execute
    void taskMainPidLoopChecker(void) {simulatedTime+=pidLoopCheckerTime;}
    void taskUpdateAccelerometer(void) {simulatedTime+=updateAccelerometerTime;}
    void taskHandleSerial(void) {simulatedTime+=handleSerialTime;}
    void taskUpdateBeeper(void) {simulatedTime+=updateBeeperTime;}
    void taskUpdateBattery(void) {simulatedTime+=updateBatteryTime;}
    bool taskUpdateRxCheck(uint32_t currentDeltaTime) {UNUSED(currentDeltaTime);simulatedTime+=updateRxCheckTime;return false;}
    void taskUpdateRxMain(void) {simulatedTime+=updateRxMainTime;}
    void taskProcessGPS(void) {simulatedTime+=processGPSTime;}
    void taskUpdateCompass(void) {simulatedTime+=updateCompassTime;}
    void taskUpdateBaro(void) {simulatedTime+=updateBaroTime;}
    void taskUpdateSonar(void) {simulatedTime+=updateSonarTime;}
    void taskCalculateAltitude(void) {simulatedTime+=calculateAltitudeTime;}
    void taskUpdateDisplay(void) {simulatedTime+=updateDisplayTime;}
    void taskTelemetry(void) {simulatedTime+=telemetryTime;}
    void taskLedStrip(void) {simulatedTime+=ledStripTime;}
    void taskTransponder(void) {simulatedTime+=transponderTime;}
}

TEST(SchedulerUnittest, TestPriorites)
{
    // check that the #defines used by scheduler.c and scheduler_unittest.cc are in sync
    EXPECT_EQ(14, TASK_COUNT);
    EXPECT_EQ(TASK_PRIORITY_HIGH, cfTasks[TASK_SYSTEM].staticPriority);
    EXPECT_EQ(TASK_PRIORITY_REALTIME, cfTasks[TASK_GYROPID].staticPriority);
    EXPECT_EQ(TASK_PRIORITY_MEDIUM, cfTasks[TASK_ACCEL].staticPriority);
    EXPECT_EQ(TASK_PRIORITY_LOW, cfTasks[TASK_SERIAL].staticPriority);
    EXPECT_EQ(TASK_PRIORITY_MEDIUM, cfTasks[TASK_BATTERY].staticPriority);
}

TEST(SchedulerUnittest, TestSingleTask)
{
    // disable all tasks except TASK_GYROPID
    for (int taskId=0; taskId < TASK_COUNT; ++taskId) {
        setTaskEnabled(static_cast<cfTaskId_e>(taskId), false);
    }
    setTaskEnabled(TASK_GYROPID, true);
    cfTasks[TASK_GYROPID].lastExecutedAt = 1000;
    simulatedTime = 4000;
    // run the scheduler and check the task has executed
    scheduler();
    EXPECT_NE(TASK_NONE, unittest_scheduler_selectedTaskId);
    EXPECT_EQ(TASK_GYROPID, unittest_scheduler_selectedTaskId);
    EXPECT_EQ(3000, cfTasks[TASK_GYROPID].taskLatestDeltaTime);
    EXPECT_EQ(4000, cfTasks[TASK_GYROPID].lastExecutedAt);
    EXPECT_EQ(pidLoopCheckerTime, cfTasks[TASK_GYROPID].totalExecutionTime);
    // task has run, so its dynamic priority should have been set to zero
    EXPECT_EQ(0, cfTasks[TASK_GYROPID].dynamicPriority);
}

TEST(SchedulerUnittest, TestTwoTasks)
{
    // disable all tasks except TASK_GYROPID  and TASK_ACCEL
    for (int taskId=0; taskId < TASK_COUNT; ++taskId) {
        setTaskEnabled(static_cast<cfTaskId_e>(taskId), false);
    }
    setTaskEnabled(TASK_ACCEL, true);
    setTaskEnabled(TASK_GYROPID, true);

    // set it up so that TASK_ACCEL ran just before TASK_GYROPID
    static const uint32_t startTime = 4000;
    simulatedTime = startTime;
    cfTasks[TASK_GYROPID].lastExecutedAt = simulatedTime;
    cfTasks[TASK_ACCEL].lastExecutedAt = cfTasks[TASK_GYROPID].lastExecutedAt - updateAccelerometerTime;
    EXPECT_EQ(0, cfTasks[TASK_ACCEL].taskAgeCycles);
    // run the scheduler
    scheduler();
    // no tasks should have run, since neither task's desired time has elapsed
    EXPECT_EQ(TASK_NONE, unittest_scheduler_selectedTaskId);

    // NOTE:
    // TASK_GYROPID desiredPeriod is  1000 microseconds
    // TASK_ACCEL   desiredPeriod is 10000 microseconds
    // 500 microseconds later
    simulatedTime += 500;
    // no tasks should run, since neither task's desired time has elapsed
    scheduler();
    EXPECT_EQ(TASK_NONE, unittest_scheduler_selectedTaskId);
    EXPECT_EQ(0, unittest_scheduler_waitingTasks);

    // 500 microseconds later, TASK_GYROPID desiredPeriod has elapsed
    simulatedTime += 500;
    // TASK_GYROPID should now run
    scheduler();
    EXPECT_EQ(TASK_GYROPID, unittest_scheduler_selectedTaskId);
    EXPECT_EQ(1, unittest_scheduler_waitingTasks);
    EXPECT_EQ(5000 + pidLoopCheckerTime, simulatedTime);

    simulatedTime += 1000 - pidLoopCheckerTime;
    scheduler();
    // TASK_GYROPID should run again
    EXPECT_EQ(TASK_GYROPID, unittest_scheduler_selectedTaskId);

    scheduler();
    EXPECT_EQ(TASK_NONE, unittest_scheduler_selectedTaskId);
    EXPECT_EQ(0, unittest_scheduler_waitingTasks);

    simulatedTime = startTime + 10500; // TASK_GYROPID and TASK_ACCEL desiredPeriods have elapsed
    // of the two TASK_GYROPID should run first
    scheduler();
    EXPECT_EQ(TASK_GYROPID, unittest_scheduler_selectedTaskId);
    // and finally TASK_ACCEL should now run
    scheduler();
    EXPECT_EQ(TASK_ACCEL, unittest_scheduler_selectedTaskId);
}
TEST(SchedulerUnittest, TestRealTimeGuardInNoTaskRun)
{
    // disable all tasks except TASK_GYROPID and TASK_SYSTEM
    for (int taskId=0; taskId < TASK_COUNT; ++taskId) {
        setTaskEnabled(static_cast<cfTaskId_e>(taskId), false);
    }
    setTaskEnabled(TASK_GYROPID, true);
    cfTasks[TASK_GYROPID].lastExecutedAt = 200000;
    simulatedTime = 200700;

    setTaskEnabled(TASK_SYSTEM, true);
    cfTasks[TASK_SYSTEM].lastExecutedAt = 100000;

    scheduler();

    EXPECT_EQ(false, unittest_outsideRealtimeGuardInterval);
    EXPECT_EQ(300, unittest_scheduler_timeToNextRealtimeTask);

    // Nothing should be scheduled in guard period
    EXPECT_EQ((uint8_t)TASK_NONE, unittest_scheduler_selectedTaskId);
    EXPECT_EQ(100000, cfTasks[TASK_SYSTEM].lastExecutedAt);

    EXPECT_EQ(200000, cfTasks[TASK_GYROPID].lastExecutedAt);
}

TEST(SchedulerUnittest, TestRealTimeGuardOutTaskRun)
{
    // disable all tasks except TASK_GYROPID and TASK_SYSTEM
    for (int taskId=0; taskId < TASK_COUNT; ++taskId) {
        setTaskEnabled(static_cast<cfTaskId_e>(taskId), false);
    }
    setTaskEnabled(TASK_GYROPID, true);
    cfTasks[TASK_GYROPID].lastExecutedAt = 200000;
    simulatedTime = 200699;

    setTaskEnabled(TASK_SYSTEM, true);
    cfTasks[TASK_SYSTEM].lastExecutedAt = 100000;

    scheduler();

    EXPECT_EQ(true, unittest_outsideRealtimeGuardInterval);
    EXPECT_EQ(301, unittest_scheduler_timeToNextRealtimeTask);

    // System should be scheduled as not in guard period
    EXPECT_EQ((uint8_t)TASK_SYSTEM, unittest_scheduler_selectedTaskId);
    EXPECT_EQ(200699, cfTasks[TASK_SYSTEM].lastExecutedAt);

    EXPECT_EQ(200000, cfTasks[TASK_GYROPID].lastExecutedAt);
}

// STUBS
extern "C" {
}

