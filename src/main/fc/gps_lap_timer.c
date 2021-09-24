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

#include "gps_lap_timer.h"

PG_REGISTER_WITH_RESET_TEMPLATE(gpsLapTimerConfig_t, gpsLapTimerConfig, PG_GPS_LAP_TIMER, 1);

PG_RESET_TEMPLATE(gpsLapTimerConfig_t, gpsLapTimerConfig,
    .gateLeftLat = 0,
    .gateLeftLon = 0,
    .gateRightLat = 0,
    .gateRightLon = 0,
    .minimumLapTimeSeconds = 10,
);

#define INTTYPE_SIGNED(T) ((T)-1 < (T)0)
#define INTTYPE_MAX(T)                      \
    (((T)1 << (8*sizeof(T)-INTTYPE_SIGNED(T)-1)) - 1 +   \
     ((T)1 << (8*sizeof(T)-INTTYPE_SIGNED(T)-1)))

// if 1000 readings of 32-bit values are used, the total (for use in calculating the averate) would need 42 bits max.
static int64_t gateSetLatReadings = 0;
static int64_t gateSetLonReadings = 0;
static bool settingGate = false;
static gpsLocation_t lastLocation;
static uint16_t timeOfLastLap;
static bool timerRunning;
static uint16_t previousLaps[2];

gpsLapTimerData_t gpsLapTimerData;

void gpsLapTimerInit(void)
{
    timeOfLastLap = 0L;
    gpsLapTimerData.currentLapTime = 0L;
    gpsLapTimerData.lastLapTime = 0L;
    gpsLapTimerData.gateLocationLeft.lat = gpsLapTimerConfig()->gateLeftLat;
    gpsLapTimerData.gateLocationLeft.lon = gpsLapTimerConfig()->gateLeftLon;
    gpsLapTimerData.gateLocationRight.lat = gpsLapTimerConfig()->gateRightLat;
    gpsLapTimerData.gateLocationRight.lon  = gpsLapTimerConfig()->gateRightLon;
    gpsLapTimerData.bestLapTime = INTTYPE_MAX(uint16_t);
    gpsLapTimerData.best3Consec = INTTYPE_MAX(uint16_t);
    gateSetLatReadings = 0;
    gateSetLonReadings = 0;
    gpsLapTimerData.numberOfSetReadings = 0;
    settingGate = false;
    timerRunning = false;
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
}

void gpsLapTimerEndSetGate(gpsLapTimerGateSide_e side)
{
    settingGate = false;

    uint32_t newLat = gateSetLatReadings / gpsLapTimerData.numberOfSetReadings;
    uint32_t newLon = gateSetLonReadings / gpsLapTimerData.numberOfSetReadings;

    if (side == GATE_SIDE_LEFT) {
        gpsLapTimerData.gateLocationLeft.lat = newLat;
        gpsLapTimerData.gateLocationLeft.lon = newLon;
        gpsLapTimerConfigMutable()->gateLeftLat = newLat;
        gpsLapTimerConfigMutable()->gateLeftLon = newLon;
    } else {
        gpsLapTimerConfigMutable()->gateRightLat = newLat;
        gpsLapTimerConfigMutable()->gateRightLon = newLon;
        gpsLapTimerData.gateLocationRight.lat = gpsLapTimerConfig()->gateRightLat;
        gpsLapTimerData.gateLocationRight.lon  = gpsLapTimerConfig()->gateRightLon;
    }
}

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

// These three functions taken from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(gpsLocation_t p, gpsLocation_t q, gpsLocation_t r)
{
    if (q.lon <= MAX(p.lon, r.lon) && q.lon >= MIN(p.lon, r.lon) &&
        q.lat <= MAX(p.lat, r.lat) && q.lat >= MIN(p.lat, r.lat)) {
        return true;
    } else {
        return false;
    }
}
 
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(gpsLocation_t p, gpsLocation_t q, gpsLocation_t r)
{
    int val = (q.lat - p.lat) * (r.lon - q.lon) -
              (q.lon - p.lon) * (r.lat - q.lat);
    if (val == 0) {
        return 0;  // collinear
    } else {
        return (val > 0) ? 1 : 2; // clock or counterclock wise
    }
}

// The main function that returns true if line segment 'gateLeftgateRight'
// and 'lastPoscurPos' intersect.
bool doSegmentsIntersect(gpsLocation_t gateLeft, gpsLocation_t gateRight, gpsLocation_t lastPos, gpsLocation_t curPos)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(gateLeft, gateRight, lastPos);
    int o2 = orientation(gateLeft, gateRight, curPos);
    int o3 = orientation(lastPos, curPos, gateLeft);
    int o4 = orientation(lastPos, curPos, gateRight);
 
    // General case
    if (o1 != o2 && o3 != o4) {
        return true;
    }

    // Special Cases
    // gateLeft, gateRight and lastPos are collinear and lastPos lies on segment gateLeftgateRight
    if (o1 == 0 && onSegment(gateLeft, lastPos, gateRight)) {
        return true;
    }
    // gateLeft, gateRight and curPos are collinear and curPos lies on segment gateLeftgateRight
    if (o2 == 0 && onSegment(gateLeft, curPos, gateRight)) {
        return true;
    }
    // lastPos, curPos and gateLeft are collinear and gateLeft lies on segment lastPoscurPos
    if (o3 == 0 && onSegment(lastPos, gateLeft, curPos)) {
        return true;
    }
     // lastPos, curPos and gateRight are collinear and gateRight lies on segment lastPoscurPos
    if (o4 == 0 && onSegment(lastPos, gateRight, curPos)) {
        return true;
    }
 
    return false; // Doesn't fall in any of the above cases
}

void gpsLapTimerUpdate(void)
{
    uint16_t currentTime = millis();

    if (timerRunning) {
        gpsLapTimerData.currentLapTime = currentTime - timeOfLastLap;
    } else {
        gpsLapTimerData.currentLapTime = 0.0;
    }

    bool crossedTimingGate = doSegmentsIntersect(gpsLapTimerData.gateLocationLeft, gpsLapTimerData.gateLocationRight,
                                                 lastLocation, gpsSol.llh);

    if (crossedTimingGate) {
        if (gpsLapTimerData.currentLapTime > (gpsLapTimerConfig()->minimumLapTimeSeconds * 1000) || !timerRunning) {
            previousLaps[1] = previousLaps[0];
            previousLaps[0] = gpsLapTimerData.lastLapTime;
            gpsLapTimerData.lastLapTime = gpsLapTimerData.currentLapTime;
            timeOfLastLap = currentTime;
            if (timeOfLastLap < gpsLapTimerData.bestLapTime) {
                gpsLapTimerData.bestLapTime = timeOfLastLap;
            }
            if (previousLaps[0] + previousLaps[1] + gpsLapTimerData.lastLapTime < gpsLapTimerData.best3Consec) {
                gpsLapTimerData.best3Consec = previousLaps[0] + previousLaps[1] + gpsLapTimerData.lastLapTime;
            }
        }
        timerRunning = true;
    }

    lastLocation = gpsSol.llh;

    if (settingGate) {
        gpsLapTimerProcessSettingGate();
    }
}

#endif // GPS_LAP_TIMER
