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
 
#include "common/axis.h"

#include "pg/pg.h"

typedef enum {
    RESCUE_SANITY_OFF = 0,
    RESCUE_SANITY_ON,
    RESCUE_SANITY_FS_ONLY
} gpsRescueSanity_e;

typedef struct gpsRescue_s {
    uint16_t angle; //degrees
    uint16_t initialAltitude; //meters
    uint16_t descentDistance; //meters
    uint16_t rescueGroundspeed; // centimeters per second
    uint16_t throttleP, throttleI, throttleD;
    uint16_t yawP;
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint16_t throttleHover;
    uint16_t velP, velI, velD;
    uint8_t minSats;
    gpsRescueSanity_e sanityChecks;
} gpsRescueConfig_t;

PG_DECLARE(gpsRescueConfig_t, gpsRescueConfig);

typedef enum {
    RESCUE_IDLE,
    RESCUE_INITIALIZE,
    RESCUE_ATTAIN_ALT,
    RESCUE_CROSSTRACK,
    RESCUE_LANDING_APPROACH,
    RESCUE_LANDING,
    RESCUE_ABORT,
    RESCUE_COMPLETE
} rescuePhase_e;

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_FLYAWAY,
    RESCUE_CRASH_DETECTED,
    RESCUE_TOO_CLOSE
} rescueFailureState_e;

typedef struct {
    int32_t targetAltitude;
    int32_t targetGroundspeed;
    uint8_t minAngleDeg;
    uint8_t maxAngleDeg;
    bool crosstrack;
} rescueIntent_s;

typedef struct {
    int32_t maxAltitude;
    int32_t currentAltitude;
    uint16_t distanceToHome;
    uint16_t maxDistanceToHome;
    int16_t directionToHome;
    uint16_t groundSpeed;
    uint8_t numSat;
    float zVelocity; // Up/down movement in cm/s
    float zVelocityAvg; // Up/down average in cm/s
    float accMagnitude;
    float accMagnitudeAvg;
} rescueSensorData_s;

typedef struct {
    bool bumpDetection;
    bool convergenceDetection;
} rescueSanityFlags;

typedef struct {
    rescuePhase_e phase;
    rescueFailureState_e failure;
    rescueSensorData_s sensor;
    rescueIntent_s intent;
    bool isFailsafe;
} rescueState_s;

extern int32_t gpsRescueAngle[ANGLE_INDEX_COUNT]; //NOTE: ANGLES ARE IN CENTIDEGREES
extern rescueState_s rescueState;

void updateGPSRescueState(void);
void rescueNewGpsData(void);
void idleTasks(void);
void rescueStop(void);
void rescueStart(void);
void setBearing(int16_t deg);
void performSanityChecks(void);
void sensorUpdate(void);

void rescueAttainPosition(void);

float gpsRescueGetYawRate(void);
float gpsRescueGetThrottle(void);
