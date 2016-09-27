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
    TASK_GYRO,
    TASK_PID,
    TASK_ACCEL,
    TASK_ATTITUDE,
    TASK_RX,
    TASK_SERIAL,
    TASK_BATTERY,
#ifdef BEEPER
    TASK_BEEPER,
#endif
#ifdef GPS
    TASK_GPS,
#endif
#ifdef MAG
    TASK_COMPASS,
#endif
#ifdef BARO
    TASK_BARO,
#endif
#ifdef SONAR
    TASK_SONAR,
#endif
#if defined(BARO) || defined(SONAR)
    TASK_ALTITUDE,
#endif
#ifdef DISPLAY
    TASK_DISPLAY,
#endif
#ifdef TELEMETRY
    TASK_TELEMETRY,
#endif
#ifdef LED_STRIP
    TASK_LEDSTRIP,
#endif
#ifdef TRANSPONDER
    TASK_TRANSPONDER,
#endif

    /* Count of real tasks */
    TASK_COUNT
} cfTaskId_e;

void taskSystem(void);
bool taskGyroCheck(uint32_t currentDeltaTime);
void taskGyro(void);
bool taskPidCheck(uint32_t currentDeltaTime);
void taskPid(void);
void taskUpdateAccelerometer(void);
void taskUpdateAttitude(void);
bool taskUpdateRxCheck(uint32_t currentDeltaTime);
void taskUpdateRxMain(void);
void taskHandleSerial(void);
void taskUpdateBeeper(void);
void taskUpdateBattery(void);
void taskUpdateRxMain(void);
void taskProcessGPS(void);
void taskUpdateCompass(void);
void taskUpdateBaro(void);
void taskUpdateSonar(void);
void taskCalculateAltitude(void);
void taskUpdateDisplay(void);
void taskTelemetry(void);
void taskLedStrip(void);
void taskTransponder(void);
