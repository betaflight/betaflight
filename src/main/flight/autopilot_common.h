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

extern float autopilotAngle[RP_AXIS_COUNT]; // NOTE: ANGLES ARE IN DEGREES

void autopilotInit(void);
void resetAltitudeControl(void);
void setSticksActiveStatus(bool areSticksActive);
void resetPositionControl(const gpsLocation_t *initialTargetLocation, unsigned taskRateHz);
bool positionControl(void);
void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeStep);

bool isBelowLandingAltitude(void);
float getAutopilotThrottle(void);
bool isAutopilotInControl(void);

void updateAutopilot(void);
float autopilotGetYawRate(void);

// Debug state for SITL telemetry
typedef struct autopilotDebugState_s {
    // Minimum altitude gate
    uint8_t vertPhase;              // 0=none, 1=minAlt gate
    float navScale;                 // scaling applied to roll/pitch (0.0-1.0)
    // Glide slope velocity limiting
    float glideSlopeExpectedAlt;    // cm (expected altitude at current distance)
    float glideSlopeDeviation;      // cm (positive=above slope, negative=below)
    float navVelocityScale;         // cruise speed scale factor (0.0-1.0)
    // Velocity controller (last axis updated)
    float velSetpointEast;          // cm/s
    float velSetpointNorth;         // cm/s
    float currentVelEast;           // cm/s
    float currentVelNorth;          // cm/s
    float dragCompEast;             // degrees
    float dragCompNorth;            // degrees
    // Position PID output (earth frame, before body rotation)
    float pidSumEast;               // degrees
    float pidSumNorth;              // degrees
    // Upsampled earth-frame values (after PT3 filter, before body rotation)
    float upsampledEast;            // degrees
    float upsampledNorth;           // degrees
    // Body frame rotation debug
    float yawHeading;               // decidegrees (attitude.values.yaw)
    float pidSumBFRoll;             // degrees (body frame, after rotation + limiting)
    float pidSumBFPitch;            // degrees (body frame, after rotation + limiting)
    // Heading debug
    float desiredBearing;           // decidegrees, bearing to waypoint target
    float interceptBearing;         // degrees, bearing to L1 intercept carrot (0.0-360.0)
    float groundCourse;             // decidegrees, GPS ground course (direction of travel)
    float pitchAlign;               // heading alignment scale on pitch (0.0-1.0)
    // Track intercept controller
    float interceptTurnRate;        // deg/s, measured turn rate from GPS ground course
    float interceptHeadingError;    // degrees, heading error to L1 bearing
    // Velocity buildup
    float buildupAngleLimit;        // degrees, current dynamic angle ceiling
    // Ground track
    float groundTrack;              // degrees (0.1-360.0), bearing from activation point to current position
    // Altitude
    float targetAltCm;              // cm, current target altitude (relative to home)
    // L1 guidance state
    bool l1Active;                  // l1GuidanceIsActive() result
    float l1PathLength;             // cm, current L1 path segment length
    float l1DistToEnd;              // cm, distance along path remaining to endpoint
    // Diagnostic
    float gpsDataIntervalS;         // seconds between GPS updates
    bool sticksActive;              // true if pilot sticks override position control
    bool navOriginValid;            // navOriginIsValid() result
} autopilotDebugState_t;

const autopilotDebugState_t *autopilotGetDebugState(void);
