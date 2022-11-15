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

#pragma once

#include "common/time.h"
#include "config/config.h"
#include "pg/scheduler.h"

#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
#define TASK_PERIOD_MS(ms) ((ms) * 1000)
#define TASK_PERIOD_US(us) (us)

#define TASK_STATS_MOVING_SUM_COUNT     8

#define LOAD_PERCENTAGE_ONE             100

#define SCHED_TASK_DEFER_MASK           0x07 // Scheduler loop count is masked with this and when 0 long running tasks are processed

#define SCHED_START_LOOP_MIN_US         1   // Wait at start of scheduler loop if gyroTask is nearly due
#define SCHED_START_LOOP_MAX_US         12
#define SCHED_START_LOOP_DOWN_STEP      50  // Fraction of a us to reduce start loop wait
#define SCHED_START_LOOP_UP_STEP        1   // Fraction of a us to increase start loop wait

#define TASK_GUARD_MARGIN_MIN_US        3   // Add an amount to the estimate of a task duration
#define TASK_GUARD_MARGIN_MAX_US        6
#define TASK_GUARD_MARGIN_DOWN_STEP     50  // Fraction of a us to reduce task guard margin
#define TASK_GUARD_MARGIN_UP_STEP       1   // Fraction of a us to increase task guard margin

#define CHECK_GUARD_MARGIN_US           2   // Add a margin to the amount of time allowed for a check function to run

// Some tasks have occasional peaks in execution time so normal moving average duration estimation doesn't work
// Decay the estimated max task duration by 1/(1 << TASK_EXEC_TIME_SHIFT) on every invocation
#define TASK_EXEC_TIME_SHIFT            7

#define TASK_AGE_EXPEDITE_RX            schedulerConfig()->rxRelaxDeterminism  // Make RX tasks more schedulable if it's failed to be scheduled this many times
#define TASK_AGE_EXPEDITE_OSD           schedulerConfig()->osdRelaxDeterminism  // Make OSD tasks more schedulable if it's failed to be scheduled this many times
#define TASK_AGE_EXPEDITE_COUNT         1   // Make aged tasks more schedulable
#define TASK_AGE_EXPEDITE_SCALE         0.9 // By scaling their expected execution time

// Gyro interrupt counts over which to measure loop time and skew
#define GYRO_RATE_COUNT 25000
#define GYRO_LOCK_COUNT 50

typedef enum {
    TASK_PRIORITY_REALTIME = -1, // Task will be run outside the scheduler logic
    TASK_PRIORITY_LOWEST = 1,
    TASK_PRIORITY_LOW = 2,
    TASK_PRIORITY_MEDIUM = 3,
    TASK_PRIORITY_MEDIUM_HIGH = 4,
    TASK_PRIORITY_HIGH = 5,
    TASK_PRIORITY_MAX = 255
} taskPriority_e;

typedef struct {
    timeUs_t     maxExecutionTimeUs;
    timeUs_t     totalExecutionTimeUs;
    timeUs_t     averageExecutionTimeUs;
    timeUs_t     averageDeltaTimeUs;
} cfCheckFuncInfo_t;

typedef struct {
    const char * taskName;
    const char * subTaskName;
    bool         isEnabled;
    int8_t       staticPriority;
    timeDelta_t  desiredPeriodUs;
    timeDelta_t  latestDeltaTimeUs;
    timeUs_t     maxExecutionTimeUs;
    timeUs_t     totalExecutionTimeUs;
    timeUs_t     averageExecutionTime10thUs;
    timeUs_t     averageDeltaTime10thUs;
    float        movingAverageCycleTimeUs;
#if defined(USE_LATE_TASK_STATISTICS)
    uint32_t     runCount;
    uint32_t     lateCount;
    timeUs_t     execTime;
#endif
} taskInfo_t;

typedef enum {
    /* Actual tasks */
    TASK_SYSTEM = 0,
    TASK_MAIN,
    TASK_GYRO,
    TASK_FILTER,
    TASK_PID,
    TASK_ACCEL,
    TASK_ATTITUDE,
    TASK_RX,
    TASK_SERIAL,
    TASK_DISPATCH,
    TASK_BATTERY_VOLTAGE,
    TASK_BATTERY_CURRENT,
    TASK_BATTERY_ALERTS,
#ifdef USE_BEEPER
    TASK_BEEPER,
#endif
#ifdef USE_GPS
    TASK_GPS,
#endif
#ifdef USE_GPS_RESCUE
    TASK_GPS_RESCUE,
#endif
#ifdef USE_MAG
    TASK_COMPASS,
#endif
#ifdef USE_BARO
    TASK_BARO,
#endif
#ifdef USE_RANGEFINDER
    TASK_RANGEFINDER,
#endif
#if defined(USE_BARO) || defined(USE_GPS)
    TASK_ALTITUDE,
#endif
#ifdef USE_DASHBOARD
    TASK_DASHBOARD,
#endif
#ifdef USE_TELEMETRY
    TASK_TELEMETRY,
#endif
#ifdef USE_LED_STRIP
    TASK_LEDSTRIP,
#endif
#ifdef USE_TRANSPONDER
    TASK_TRANSPONDER,
#endif
#ifdef USE_STACK_CHECK
    TASK_STACK_CHECK,
#endif
#ifdef USE_OSD
    TASK_OSD,
#endif
#ifdef USE_BST
    TASK_BST_MASTER_PROCESS,
#endif
#ifdef USE_ESC_SENSOR
    TASK_ESC_SENSOR,
#endif
#ifdef USE_CMS
    TASK_CMS,
#endif
#ifdef USE_VTX_CONTROL
    TASK_VTXCTRL,
#endif
#ifdef USE_CAMERA_CONTROL
    TASK_CAMCTRL,
#endif
#ifdef USE_RCDEVICE
    TASK_RCDEVICE,
#endif
#ifdef USE_ADC_INTERNAL
    TASK_ADC_INTERNAL,
#endif
#ifdef USE_PINIOBOX
    TASK_PINIOBOX,
#endif
#ifdef USE_CRSF_V3
    TASK_SPEED_NEGOTIATION,
#endif

    /* Count of real tasks */
    TASK_COUNT,

    /* Service task IDs */
    TASK_NONE = TASK_COUNT,
    TASK_SELF
} taskId_e;

typedef struct {
    // Configuration
    const char * taskName;
    const char * subTaskName;
    bool (*checkFunc)(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
    void (*taskFunc)(timeUs_t currentTimeUs);
    timeDelta_t desiredPeriodUs;        // target period of execution
    const int8_t staticPriority;        // dynamicPriority grows in steps of this size
} task_attribute_t;

typedef struct {
    // Task static data
    task_attribute_t *attribute;

    // Scheduling
    uint16_t dynamicPriority;           // measurement of how old task was last executed, used to avoid task starvation
    uint16_t taskAgePeriods;
    timeDelta_t taskLatestDeltaTimeUs;
    timeUs_t lastExecutedAtUs;          // last time of invocation
    timeUs_t lastSignaledAtUs;          // time of invocation event for event-driven tasks
    timeUs_t lastDesiredAt;             // time of last desired execution

    // Statistics
    float    movingAverageCycleTimeUs;
    timeUs_t anticipatedExecutionTime;  // Fixed point expectation of next execution time
    timeUs_t movingSumDeltaTime10thUs;  // moving sum over 64 samples
    timeUs_t movingSumExecutionTime10thUs;
    timeUs_t maxExecutionTimeUs;
    timeUs_t totalExecutionTimeUs;      // total time consumed by task since boot
    timeUs_t lastStatsAtUs;             // time of last stats gathering for rate calculation
#if defined(USE_LATE_TASK_STATISTICS)
    uint32_t runCount;
    uint32_t lateCount;
    timeUs_t execTime;
#endif
} task_t;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo);
void getTaskInfo(taskId_e taskId, taskInfo_t *taskInfo);
void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs);
void setTaskEnabled(taskId_e taskId, bool newEnabledState);
timeDelta_t getTaskDeltaTimeUs(taskId_e taskId);
void schedulerIgnoreTaskStateTime();
void schedulerIgnoreTaskExecRate();
void schedulerIgnoreTaskExecTime();
bool schedulerGetIgnoreTaskExecTime();
void schedulerResetTaskStatistics(taskId_e taskId);
void schedulerResetTaskMaxExecutionTime(taskId_e taskId);
void schedulerResetCheckFunctionMaxExecutionTime(void);
void schedulerSetNextStateTime(timeDelta_t nextStateTime);
timeDelta_t schedulerGetNextStateTime(void);
void schedulerInit(void);
void scheduler(void);
timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs);
void taskSystemLoad(timeUs_t currentTimeUs);
void schedulerEnableGyro(void);
uint16_t getAverageSystemLoadPercent(void);
float schedulerGetCycleTimeMultiplier(void);
