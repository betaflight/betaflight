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

#pragma once

typedef enum {
    /* Actual tasks */
    TASK_SYSTEM = 0,
    TASK_CYCLE_TIME,
    TASK_BATTERY,
    TASK_STATUS_LED,
    TASK_HARDWARE_WATCHDOG,
    TASK_MSP_SERVER,
    TASK_DRAW_SCREEN,
    TASK_UPDATE_FC_STATE,
    TASK_TEST,

    /* Count of real tasks */
    TASK_COUNT
} cfTaskId_e;

void taskUpdateCycleTime(void);
void taskMSP(void);
void taskUpdateBattery(void);
void taskStatusLed(void);
void taskHardwareWatchdog(void);
void taskSystem(void);
void taskDrawScreen(void);
void taskUpdateFCState(void);
void taskTest(void);
