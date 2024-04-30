/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "scheduler/scheduler.h"
#include "scheduler_stubs.h"

PG_REGISTER_WITH_RESET_TEMPLATE(schedulerConfig_t, schedulerConfig, PG_SCHEDULER_CONFIG, 0);

PG_RESET_TEMPLATE(schedulerConfig_t, schedulerConfig,
    .rxRelaxDeterminism = 25,
    .osdRelaxDeterminism = 25,
);

#define TEST_GYRO_SAMPLE_HZ 8000

void taskGyroSample(timeUs_t);
void taskFiltering(timeUs_t);
void taskMainPidLoop(timeUs_t);
void taskUpdateAccelerometer(timeUs_t);
void taskHandleSerial(timeUs_t);
void taskUpdateBatteryVoltage(timeUs_t);
bool rxUpdateCheck(timeUs_t, timeDelta_t);
void taskUpdateRxMain(timeUs_t);
void imuUpdateAttitude(timeUs_t);
void dispatchProcess(timeUs_t);
bool osdUpdateCheck(timeUs_t, timeDelta_t);
void osdUpdate(timeUs_t);

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
