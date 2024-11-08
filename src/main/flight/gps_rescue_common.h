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
#pragma once

#include <stdbool.h>

#include "common/axis.h"

#include "pg/gps_rescue.h"

typedef struct {
    float distanceToHomeCm;
    float distanceToHomeM;
    uint16_t groundSpeedCmS;
    int16_t directionToHome;
    bool healthy;
    float errorAngle;
    float gpsDataIntervalSeconds;
    float velocityToHomeCmS;
    float alitutudeStepCm;
    float maxPitchStep;
    float absErrorAngle;
    float imuYawCogGain;
} rescueSensorData_s;

typedef struct {
    float maxAltitudeCm;
    float returnAltitudeCm;
    float targetAltitudeCm;
    float disarmThreshold;
    float descentDistanceM;
} rescueIntentCommon_s;

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_FLYAWAY,
    RESCUE_GPSLOST,
    RESCUE_LOWSATS,
    RESCUE_CRASHFLIP_DETECTED,
    RESCUE_STALLED,
    RESCUE_TOO_CLOSE,
    RESCUE_NO_HOME_POINT
} rescueFailureState_e;

extern bool newGPSData;
extern float rescueYaw;

void setReturnAltitude(rescueIntentCommon_s *intentCommon, const rescueSensorData_s *sensorData);
void sensorUpdate(rescueSensorData_s *sensorData);
void disarmOnImpact(const rescueIntentCommon_s *intentCommon, void (*rescueStopFunc)(void));
bool checkGPSRescueIsAvailable(void);
