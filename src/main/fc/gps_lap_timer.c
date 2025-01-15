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

#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"

#include "drivers/time.h"

#include "fc/rc_modes.h"

#include "io/gps.h"

#include "gps_lap_timer.h"

#define MAX_GATE_SET_READINGS 1000

// if 1000 readings of 32-bit values are used, the total (for use in calculating the averate) would need 42 bits max.
static int64_t gateSetLatReadings;
static int64_t gateSetLonReadings;
static bool settingGate = false;
static uint32_t minDistance = UINT32_MAX;
static uint32_t minDistanceTime = 0;
static bool wasInCircle = false;

gpsLapTimerData_t gpsLapTimerData;

void gpsLapTimerInit(void)
{
    gpsLapTimerData.gateLocation.lat = gpsLapTimerConfig()->gateLat;
    gpsLapTimerData.gateLocation.lon  = gpsLapTimerConfig()->gateLon;
    gpsLapTimerData.numberOfSetReadings = 0;
    gpsLapTimerData.bestLapTime = 0;
    gpsLapTimerData.best3Consec = 0;
    for (uint8_t i = 0; i < MAX_N_RECORDED_PREVIOUS_LAPS; i++) {
        gpsLapTimerData.previousLaps[i] = 0;
    }
    gpsLapTimerData.timerRunning = false;
    gpsLapTimerData.timeOfLastLap = 0;
    gpsLapTimerData.numberOfLapsRecorded = 0;
}

void gpsLapTimerStartSetGate(void)
{
    settingGate = true;
    gateSetLatReadings = 0;
    gateSetLonReadings = 0;
    gpsLapTimerData.numberOfSetReadings = 0;
}

static void gpsLapTimerProcessSettingGate(void)
{
    if (gpsLapTimerData.numberOfSetReadings < MAX_GATE_SET_READINGS){
        gateSetLatReadings += gpsSol.llh.lat;
        gateSetLonReadings += gpsSol.llh.lon;
        gpsLapTimerData.numberOfSetReadings++;
    }
    gpsLapTimerData.gateLocation.lat = gateSetLatReadings / gpsLapTimerData.numberOfSetReadings;
    gpsLapTimerData.gateLocation.lon = gateSetLonReadings / gpsLapTimerData.numberOfSetReadings;
}

void gpsLapTimerEndSetGate(void)
{
    settingGate = false;
    gpsLapTimerConfigMutable()->gateLat = gpsLapTimerData.gateLocation.lat;
    gpsLapTimerConfigMutable()->gateLon = gpsLapTimerData.gateLocation.lon;
}

void gpsLapTimerNewGpsData(void)
{
    GPS_distance_cm_bearing(&gpsSol.llh, &gpsLapTimerData.gateLocation, false, &gpsLapTimerData.distToPointCM, &gpsLapTimerData.dirToPoint);

    if (settingGate) {
        gpsLapTimerProcessSettingGate();
        return;
    }

    // Reset all lap timer data if reset mode is active
    if (IS_RC_MODE_ACTIVE(BOXLAPTIMERRESET)) {
        gpsLapTimerInit();
        return;
    }

    // Current lap time is at least the min lap timer or timer not running, so we need to get ready to record a gate crossing
    if (!gpsLapTimerData.timerRunning || gpsSol.time - gpsLapTimerData.timeOfLastLap > (gpsLapTimerConfig()->minimumLapTimeSeconds * 1000)) {
        // Within radius of gate, record the closest point we get before leaving
        if (gpsLapTimerData.distToPointCM < (gpsLapTimerConfig()->gateToleranceM * 100)) {
            // Either just entered the circle or were already in circle but are the closest we've been to the center this lap
            if (!wasInCircle || gpsLapTimerData.distToPointCM < minDistance) {
                // Track the closest we've been to the center
                minDistance = gpsLapTimerData.distToPointCM;
                // Track the time we were the closest to the center, which will be used to determine the actual lap time
                minDistanceTime = gpsSol.time;
            }
            wasInCircle = true;
        } else {
            // Just left the circle, so record the lap time
            if (wasInCircle) {
                // Not the first time through the gate
                if (gpsLapTimerData.timerRunning) {
                    uint32_t lapTime = minDistanceTime - gpsLapTimerData.timeOfLastLap;
                    gpsLapTimerData.numberOfLapsRecorded++;

                    // Update best lap time
                    if (gpsLapTimerData.numberOfLapsRecorded >= 1 && (lapTime < gpsLapTimerData.bestLapTime || gpsLapTimerData.bestLapTime == 0)) {
                        gpsLapTimerData.bestLapTime = lapTime;
                    }

                    // Shift array of previously recorded laps and add latest
                    for (unsigned i = MAX_N_RECORDED_PREVIOUS_LAPS - 1; i > 0; i--) {
                        gpsLapTimerData.previousLaps[i] = gpsLapTimerData.previousLaps[i-1];
                    }
                    gpsLapTimerData.previousLaps[0] = lapTime;

                    // Check if we're able to calculate a best N consec time yet, and add them up just in case
                    uint32_t sumLastNLaps = 0;
                    if (gpsLapTimerData.numberOfLapsRecorded >= MAX_N_RECORDED_PREVIOUS_LAPS) {
                        for (int i = 0; i < MAX_N_RECORDED_PREVIOUS_LAPS; i++) {
                            sumLastNLaps += gpsLapTimerData.previousLaps[i];
                        }
                    }

                    // Check if this is better than the previous best
                    if (sumLastNLaps > 0 && (sumLastNLaps < gpsLapTimerData.best3Consec || gpsLapTimerData.best3Consec == 0)) {
                        gpsLapTimerData.best3Consec = sumLastNLaps;
                    }
                }
                gpsLapTimerData.timeOfLastLap = minDistanceTime;
                gpsLapTimerData.timerRunning = true;
            }
            wasInCircle = false;
        }
    }
}
