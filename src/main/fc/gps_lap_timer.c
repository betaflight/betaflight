/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_GPS_LAP_TIMER

#include "drivers/time.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "fc/gps_lap_timer.h"
#include "fc/rc_modes.h"

PG_REGISTER_WITH_RESET_TEMPLATE(gpsLapTimerConfig_t, gpsLapTimerConfig, PG_GPS_LAP_TIMER, 1);

PG_RESET_TEMPLATE(gpsLapTimerConfig_t, gpsLapTimerConfig,
    .gateLat = 0,
    .gateLon = 0,
    .minimumLapTimeSeconds = 10,
    .gateTolerance = 9,
);

// if 1000 readings of 32-bit values are used, the total (for use in calculating the averate) would need 42 bits max.
static int64_t gateSetLatReadings;
static int64_t gateSetLonReadings;
static bool settingGate = false;
static uint32_t minDistance = UINT32_MAX;
static uint32_t minDistanceTime = 0L;
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
    gpsLapTimerData.timeOfLastLap = 0L;
}

void gpsLapTimerStartSetGate(void)
{
    settingGate = true;
    gateSetLatReadings = 0;
    gateSetLonReadings = 0;
    gpsLapTimerData.numberOfSetReadings = 0;
}

void gpsLapTimerProcessSettingGate(void)
{
    if (gpsLapTimerData.numberOfSetReadings < 1000){
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

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

void lapTimerNewGpsData(void)
{
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &gpsLapTimerData.gateLocation.lat, &gpsLapTimerData.gateLocation.lon, &gpsLapTimerData.distToPoint, &gpsLapTimerData.dirToPoint);

    if (IS_RC_MODE_ACTIVE(BOXLAPTIMERRESET)) {
        gpsLapTimerInit();
        return;
    }

    if (!gpsLapTimerData.timerRunning) {
        gpsLapTimerData.timeOfLastLap = 0L;
    }

    // Current lap time is at least the min lap timer or timer not running, so we need to get ready to record a gate crossing
    if (!gpsLapTimerData.timerRunning || gpsSol.time - gpsLapTimerData.timeOfLastLap > (gpsLapTimerConfig()->minimumLapTimeSeconds * 1000)) {

        // Within radius of gate, record the closest point we get before leaving
        if (gpsLapTimerData.distToPoint < (gpsLapTimerConfig()->gateTolerance * 100)) {
            // Either just entered the circle or were already in circle but are the closest we've been to the center this lap
            if (!wasInCircle || gpsLapTimerData.distToPoint < minDistance) {
                minDistance = gpsLapTimerData.distToPoint;
                minDistanceTime = gpsSol.time;
            }
            wasInCircle = true;
        } else {
            // Just left the circle, so record the time
            if (wasInCircle) {
                // Not the first time through the gate
                if (gpsLapTimerData.timerRunning) {
                    uint32_t lapTime = minDistanceTime - gpsLapTimerData.timeOfLastLap;

                    // Update best N consecutive
                    for (uint8_t i = MAX_N_RECORDED_PREVIOUS_LAPS - 1; i > 0; i--) {
                        gpsLapTimerData.previousLaps[i] = gpsLapTimerData.previousLaps[i-1];
                    }
                    gpsLapTimerData.previousLaps[0] = lapTime;

                    // Check if we're able to calculate a best N consec time yet, and add them up just in case
                    bool areLapsFilled = true;
                    uint32_t sumLastNLaps = 0;
                    for (int i = 0; i < MAX_N_RECORDED_PREVIOUS_LAPS; i++) {
                        if (gpsLapTimerData.previousLaps[i] == 0) {
                            areLapsFilled = false;
                        }
                        sumLastNLaps += gpsLapTimerData.previousLaps[i];
                    }

                    // Check if this is better than the previous best
                    if (areLapsFilled && (sumLastNLaps < gpsLapTimerData.best3Consec || gpsLapTimerData.best3Consec == 0)) {
                        gpsLapTimerData.best3Consec = sumLastNLaps;
                    }

                    // Update best lap time
                    if (gpsLapTimerData.previousLaps[0] != 0 &&
                        (gpsLapTimerData.previousLaps[0] < gpsLapTimerData.bestLapTime || gpsLapTimerData.bestLapTime == 0)) {
                        gpsLapTimerData.bestLapTime = gpsLapTimerData.previousLaps[0];
                    }
                }
                gpsLapTimerData.timeOfLastLap = minDistanceTime;
                gpsLapTimerData.timerRunning = true;
            }
            wasInCircle = false;
        }
    }

    if (settingGate) {
        gpsLapTimerProcessSettingGate();
    }
}

#endif // GPS_LAP_TIMER
