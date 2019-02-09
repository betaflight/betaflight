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

#include "FreeRTOS.h"
#include "task.h"

#define TASK_PERIOD_HZ(hz) (1000 / (hz))
#define TASK_PERIOD_MS(ms) (ms)


typedef enum {
    TASK_PRIORITY_IDLE = 0,     // Disables dynamic scheduling, task is executed only if no other task is active this cycle
    TASK_PRIORITY_LOW = 1,
    TASK_PRIORITY_MEDIUM = 3,
    TASK_PRIORITY_MEDIUM_HIGH = 4,
    TASK_PRIORITY_HIGH = 5,
    TASK_PRIORITY_REALTIME = 6
} cfTaskPriority_e;

typedef enum {
    /* Actual tasks */
    TASK_MAIN = 0,
    TASK_GYROPID,
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
#ifdef STACK_CHECK
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

    /* Count of real tasks */
    TASK_COUNT,

    /* Service task IDs */
    TASK_NONE = TASK_COUNT,
    TASK_SELF
} cfTaskId_e;

typedef struct {
    // Configuration
	TaskHandle_t taskId;
#if defined(USE_TASK_STATISTICS)
    const char * taskName;
    const char * subTaskName;
#endif
    void (*taskFunc)();
    uint16_t desiredPeriod; // target period of execution
    uint16_t stackSize;     // stack size in words
    uint8_t priority;
    TickType_t lastExecutedAt;
} cfTask_t;

extern cfTask_t cfTasks[TASK_COUNT];
extern void tasksLaunch();
extern void fcTasksInit(void);
extern void fcTaskReschedule(cfTaskId_e taskId, uint32_t newPeriod);
extern uint16_t fcTaskGetPeriod(cfTaskId_e taskId);
