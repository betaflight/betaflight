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


extern "C" {
    uint8_t unittest_scheduler_taskId;
    uint8_t unittest_scheduler_selectedTaskId;
    uint8_t unittest_scheduler_selectedTaskDynPrio;
    uint16_t unittest_scheduler_waitingTasks;
    uint32_t unittest_scheduler_timeToNextRealtimeTask;

// set up micros() to simulate time
    uint32_t simulatedTime = 0;
    uint32_t micros(void) {return simulatedTime;}
// set up all tasks to take a 10 microseconds to execute
// !!TODO set these to use realistic times
    void taskMainPidLoopChecker(void) {simulatedTime+=10;}
    void taskUpdateAccelerometer(void) {simulatedTime+=10;}
    void taskHandleSerial(void) {simulatedTime+=10;}
    void taskUpdateBeeper(void) {simulatedTime+=10;}
    void taskUpdateBattery(void) {simulatedTime+=10;}
    bool taskUpdateRxCheck(uint32_t currentDeltaTime) {UNUSED(currentDeltaTime);simulatedTime+=10;return false;}
    void taskUpdateRxMain(void) {simulatedTime+=10;}
    void taskProcessGPS(void) {simulatedTime+=10;}
    void taskUpdateCompass(void) {simulatedTime+=10;}
    void taskUpdateBaro(void) {simulatedTime+=10;}
    void taskUpdateSonar(void) {simulatedTime+=10;}
    void taskCalculateAltitude(void) {simulatedTime+=10;}
    void taskUpdateDisplay(void) {simulatedTime+=10;}
    void taskTelemetry(void) {simulatedTime+=10;}
    void taskLedStrip(void) {simulatedTime+=10;}
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
    EXPECT_EQ(10, cfTasks[TASK_GYROPID].totalExecutionTime);
    // task has run, so its dynamic priority should have been set to zero
    EXPECT_EQ(0, cfTasks[TASK_GYROPID].dynamicPriority);
}


// STUBS
extern "C" {
}

