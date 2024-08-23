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

#pragma once

#include "io/gps.h"

#include "pg/gps_lap_timer.h"

#define MAX_N_RECORDED_PREVIOUS_LAPS 3

typedef struct gpsLapTimerData_s {
    gpsLocation_t gateLocation;
    uint32_t previousLaps[MAX_N_RECORDED_PREVIOUS_LAPS];
    uint32_t currentLapTime;
    uint32_t bestLapTime;
    uint32_t best3Consec;
    uint32_t distToPointCM;
    uint32_t timeOfLastLap;
    int32_t  dirToPoint;
    uint16_t numberOfSetReadings;
    uint16_t numberOfLapsRecorded;
    bool timerRunning;
} gpsLapTimerData_t;

extern gpsLapTimerData_t gpsLapTimerData;

void gpsLapTimerInit(void);
void gpsLapTimerNewGpsData(void);
void gpsLapTimerStartSetGate(void);
void gpsLapTimerEndSetGate(void);
