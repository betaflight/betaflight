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
#include "io/gps.h"

uint16_t      rescueThrottle;

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
