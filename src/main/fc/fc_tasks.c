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

#include <platform.h>

#include "fc/fc_tasks.h"

#include "scheduler/scheduler.h"

// No need for a linked list for the queue, since items are only inserted at startup
#ifdef UNIT_TEST
#define TASK_QUEUE_ARRAY_SIZE (TASK_COUNT + 2) // 1 extra space so test code can check for buffer overruns
#else
#define TASK_QUEUE_ARRAY_SIZE (TASK_COUNT + 1) // extra item for NULL pointer at end of queue
#endif

const uint32_t taskQueueArraySize = TASK_QUEUE_ARRAY_SIZE;
const uint32_t taskCount = TASK_COUNT;
cfTask_t* taskQueueArray[TASK_QUEUE_ARRAY_SIZE];

cfTask_t cfTasks[] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = 1000000 / 10,          // 10 Hz, every 100 ms
        .staticPriority = TASK_PRIORITY_HIGH,
    },

    [TASK_GYROPID] = {
        .taskName = "GYRO/PID",
        .taskFunc = taskMainPidLoopChecker,
        .desiredPeriod = 1000,                  // every 1 ms
        .staticPriority = TASK_PRIORITY_REALTIME,
    },

    [TASK_ACCEL] = {
        .taskName = "ACCEL",
        .taskFunc = taskUpdateAccelerometer,
        .desiredPeriod = 1000,                  // every 1 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
        .desiredPeriod = 1000000 / 100,         // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .staticPriority = TASK_PRIORITY_LOW,
    },

#ifdef BEEPER
    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = taskUpdateBeeper,
        .desiredPeriod = 1000000 / 100,         // 100 Hz, every 10 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

    [TASK_BATTERY] = {
        .taskName = "BATTERY",
        .taskFunc = taskUpdateBattery,
        .desiredPeriod = 1000000 / 50,          // 50 Hz, every 20 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_RX] = {
        .taskName = "RX",
        .checkFunc = taskUpdateRxCheck,
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = 1000000 / 50,          // If event-based scheduling doesn't work, fallback to periodic scheduling
        .staticPriority = TASK_PRIORITY_HIGH,
    },

#ifdef GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = taskProcessGPS,
        .desiredPeriod = 1000000 / 10,          // GPS usually don't go faster than 10Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef MAG
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = taskUpdateCompass,
        .desiredPeriod = 1000000 / 10,          // 10 Hz, every 100 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef BARO
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = 1000000 / 20,          // 20 Hz, every 50 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef SONAR
    [TASK_SONAR] = {
        .taskName = "SONAR",
        .taskFunc = taskUpdateSonar,
        .desiredPeriod = 70000,                 // every 70 ms, approximately 14 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#if defined(BARO) || defined(SONAR)
    [TASK_ALTITUDE] = {
        .taskName = "ALTITUDE",
        .taskFunc = taskCalculateAltitude,
        .desiredPeriod = 1000000 / 40,          // 40 Hz, every 25 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef TRANSPONDER
    [TASK_TRANSPONDER] = {
        .taskName = "TRANSPONDER",
        .taskFunc = taskTransponder,
        .desiredPeriod = 1000000 / 250,         // 250 Hz, every 4 ms
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef DISPLAY
    [TASK_DISPLAY] = {
        .taskName = "DISPLAY",
        .taskFunc = taskUpdateDisplay,
        .desiredPeriod = 1000000 / 10,          // 10 Hz, every 100 ms
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = 1000000 / 250,         // 250 Hz, every 4 ms
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = taskLedStrip,
        .desiredPeriod = 1000000 / 100,         // 100 Hz, every 10 ms
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif
};
