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
    ledStripTime = 10
};

extern "C" {
    uint8_t unittest_scheduler_taskId;
    uint8_t unittest_scheduler_selectedTaskId;
    uint8_t unittest_scheduler_selectedTaskDynPrio;
    uint16_t unittest_scheduler_waitingTasks;
    uint32_t unittest_scheduler_timeToNextRealtimeTask;

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


// STUBS
extern "C" {
}

