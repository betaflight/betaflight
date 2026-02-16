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
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

#ifndef USE_WING

#include "build/debug.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/vector.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/gyro.h"

#include "pg/autopilot.h"
#include "pg/flight_plan.h"
#include "autopilot.h"
#include "autopilot_waypoint.h"
#if ENABLE_FLIGHT_PLAN
#include "autopilot_guidance.h"
#endif

#define ALTITUDE_P_SCALE  0.01f
#define ALTITUDE_I_SCALE  0.003f
#define ALTITUDE_D_SCALE  0.01f
#define ALTITUDE_F_SCALE  0.01f
#define POSITION_P_SCALE  0.0012f
#define POSITION_I_SCALE  0.0001f
#define POSITION_D_SCALE  0.0015f
#define POSITION_A_SCALE  0.0008f
#define UPSAMPLING_CUTOFF_HZ 5.0f

static pidCoefficient_t altitudePidCoeffs;
static pidCoefficient_t positionPidCoeffs;

static float altitudeI = 0.0f;
static float throttleOut = 0.0f;

typedef struct efPidAxis_s {
    bool isStopping;
    float previousDistance;
    float previousVelocity;
    float integral;               // Position loop integral
    pt1Filter_t velocityLpf;
    pt1Filter_t accelerationLpf;
    // Velocity controller state
    float velocitySetpoint;       // Desired velocity (cm/s)
    float velocityIntegral;       // Velocity loop integral
    float previousVelocityError;  // For velocity D-term
} efPidAxis_t;

typedef enum {
    // axes are for NED system (North-East-Down); standard aviation coordinate frame
    EAST = 0,   // Y-axis in NED, East
    NORTH       // X-axis in NED, North (swapped with EAST for backward compatibility with array indexing)
} axisEF_e;

typedef struct autopilotState_s {
    gpsLocation_t targetLocation;       // active / current target
    float sanityCheckDistance;
    float upsampleLpfGain;              // for the Body Frame upsample filter for pitch and roll
    float vaLpfCutoff;                  // velocity + acceleration lowpass filter cutoff
    bool sticksActive;
    float maxAngle;
    vector2_t pidSumBF;                 // pid output, updated on each GPS update, rotated to body frame
    pt3Filter_t upsampleLpfBF[RP_AXIS_COUNT];    // upsampling filter
    efPidAxis_t efAxis[EF_AXIS_COUNT];

    // Yaw control
    float desiredYawRate;               // deg/s output for RC command
    float previousYawError;             // For D term
    timeUs_t yawControlStartTime;       // When yaw control became active
    bool yawControlWasActive;           // Previous yaw control state

    // Pilot override detection
    bool pilotOverrideRoll;
    bool pilotOverridePitch;
    bool pilotOverrideYaw;
    bool pilotOverrideThrottle;
} autopilotState_t;

static autopilotState_t ap = {
    .sanityCheckDistance = 1000.0f,
    .upsampleLpfGain = 1.0f,
    .vaLpfCutoff = 1.0f,
    .sticksActive = false,
};

float autopilotAngle[RP_AXIS_COUNT];

static void resetEFAxisFilters(efPidAxis_t* efAxis, const float vaGain)
{
    pt1FilterInit(&efAxis->velocityLpf, vaGain);
    pt1FilterInit(&efAxis->accelerationLpf, vaGain);
}

static void resetEFAxisParams(efPidAxis_t *efAxis, const float vaGain)
{
    // at start only
    resetEFAxisFilters(efAxis, vaGain);
    efAxis->isStopping = true; // Enter starting (deceleration) 'phase'
    efAxis->integral = 0.0f;
    // Velocity controller state
    efAxis->velocitySetpoint = 0.0f;
    efAxis->velocityIntegral = 0.0f;
    efAxis->previousVelocityError = 0.0f;
}

static void resetUpsampleFilters(void)
{
    for (unsigned i = 0; i < ARRAYLEN(ap.upsampleLpfBF); i++) {
        pt3FilterInit(&ap.upsampleLpfBF[i], ap.upsampleLpfGain);
    }
}

// get sanity distance based on speed
static inline float sanityCheckDistance(const float gpsGroundSpeedCmS)
{
    return fmaxf(1000.0f, gpsGroundSpeedCmS * 2.0f);
    // distance flown in 2s at current speed. with minimum of 10m
}

void resetPositionControl(const gpsLocation_t *initialTargetLocation, unsigned taskRateHz)
{
    // from pos_hold.c (or other client) when initiating position hold at target location
    ap.targetLocation = *initialTargetLocation;
    ap.sticksActive = false;
    // set sanity check distance according to groundspeed at start, minimum of 10m
    ap.sanityCheckDistance = sanityCheckDistance(gpsSol.groundSpeed);
    for (unsigned i = 0; i < ARRAYLEN(ap.efAxis); i++) {
        // clear anything stored in the filter at first call
        resetEFAxisParams(&ap.efAxis[i], 1.0f);
    }
    const float taskInterval = 1.0f / taskRateHz;
    ap.upsampleLpfGain = pt3FilterGain(UPSAMPLING_CUTOFF_HZ, taskInterval); // 5Hz; normally at 100Hz task rate
    resetUpsampleFilters(); // clear accumlator from previous iterations
}

void autopilotInit(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    ap.sticksActive = false;
    ap.maxAngle = cfg->maxAngle;
    altitudePidCoeffs.Kp = cfg->altitudeP * ALTITUDE_P_SCALE;
    altitudePidCoeffs.Ki = cfg->altitudeI * ALTITUDE_I_SCALE;
    altitudePidCoeffs.Kd = cfg->altitudeD * ALTITUDE_D_SCALE;
    altitudePidCoeffs.Kf = cfg->altitudeF * ALTITUDE_F_SCALE;
    positionPidCoeffs.Kp = cfg->positionP * POSITION_P_SCALE;
    positionPidCoeffs.Ki = cfg->positionI * POSITION_I_SCALE;
    positionPidCoeffs.Kd = cfg->positionD * POSITION_D_SCALE;
    positionPidCoeffs.Kf = cfg->positionA * POSITION_A_SCALE; // Kf used for acceleration
    // initialise filters with approximate filter gains; location isn't used at this point.
    ap.upsampleLpfGain = pt3FilterGain(UPSAMPLING_CUTOFF_HZ, 0.01f); // 5Hz, assuming 100Hz task rate at init
    resetUpsampleFilters();
    // Initialise PT1 filters for velocity and acceleration in earth frame axes
    ap.vaLpfCutoff = cfg->positionCutoff * 0.01f;
    const float vaGain = pt1FilterGain(ap.vaLpfCutoff,  0.1f); // assume 10Hz GPS connection at start; value is overwritten before first filter use
    for (unsigned i = 0; i < ARRAYLEN(ap.efAxis); i++) {
        resetEFAxisFilters(&ap.efAxis[i], vaGain);
    }

#if ENABLE_FLIGHT_PLAN
    // Initialize waypoint system
    waypointInit();

    // Initialize yaw control
    ap.desiredYawRate = 0.0f;
    ap.previousYawError = 0.0f;

    // Initialize pilot override flags
    ap.pilotOverrideRoll = false;
    ap.pilotOverridePitch = false;
    ap.pilotOverrideYaw = false;
    ap.pilotOverrideThrottle = false;
#endif
}

void resetAltitudeControl (void) {
    altitudeI = 0.0f;
}

void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeStep)
{
    const float verticalVelocityCmS = getAltitudeDerivative();
    const float altitudeErrorCm = targetAltitudeCm - getAltitudeCm();
    const float altitudeP = altitudeErrorCm * altitudePidCoeffs.Kp;

    // reduce the iTerm gain for errors greater than 200cm (2m), otherwise it winds up too much
    const float itermRelax = (fabsf(altitudeErrorCm) < 200.0f) ? 1.0f : 0.1f;
    altitudeI += altitudeErrorCm * altitudePidCoeffs.Ki * itermRelax * taskIntervalS;
    // limit iTerm to not more than 200 throttle units
    altitudeI = constrainf(altitudeI, -200.0f, 200.0f);

    // increase D when velocity is high, typically when initiating hold at high vertical speeds
    // 1.0 when less than 5 m/s, 2x at 10m/s, 2.5 at 20 m/s, 2.8 at 50 m/s, asymptotes towards max 3.0.
    float dBoost = 1.0f;
    {
        const float startValue = 500.0f; // velocity at which D should start to increase
        const float altDeriv = fabsf(verticalVelocityCmS);
        if (altDeriv > startValue) {
            const float ratio = altDeriv / startValue;
            dBoost = (3.0f * ratio - 2.0f) / ratio;
        }
    }

    const float altitudeD = verticalVelocityCmS * dBoost * altitudePidCoeffs.Kd;

    const float altitudeF = targetAltitudeStep * altitudePidCoeffs.Kf;

    const float hoverOffset = autopilotConfig()->hoverThrottle - PWM_RANGE_MIN;
    float throttleOffset = altitudeP + altitudeI - altitudeD + altitudeF + hoverOffset;

    const float tiltMultiplier = 1.0f / fmaxf(getCosTiltAngle(), 0.5f);
    // 1 = flat, 1.3 at 40 degrees, 1.56 at 50 deg, max 2.0 at 60 degrees or higher
    // note: the default limit of Angle Mode is 60 degrees

    throttleOffset *= tiltMultiplier;

    float newThrottle = PWM_RANGE_MIN + throttleOffset;
    newThrottle = constrainf(newThrottle, autopilotConfig()->throttleMin, autopilotConfig()->throttleMax);
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 0, lrintf(newThrottle)); // normal range 1000-2000 but is before constraint

    newThrottle = scaleRangef(newThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);

    throttleOut = constrainf(newThrottle, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 1, lrintf(tiltMultiplier * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 3, lrintf(targetAltitudeCm));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 4, lrintf(altitudeP));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 5, lrintf(altitudeI));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 6, lrintf(-altitudeD));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 7, lrintf(altitudeF));
}

void setSticksActiveStatus(bool areSticksActive)
{
    ap.sticksActive = areSticksActive;
}

static void setTargetLocationByAxis(const gpsLocation_t* newTargetLocation, axisEF_e efAxisIdx)
// not used at present but needed by upcoming GPS code
{
    if (efAxisIdx == EAST) {
        ap.targetLocation.lon = newTargetLocation->lon; // update East-West / / longitude position
    } else {
        ap.targetLocation.lat = newTargetLocation->lat; // update North-South / latitude position
    }
}

// Calculate maximum bank angle based on turn rate and velocity
// Ensures coordinated turns at configured rate (e.g., rate-1 turn = 3 deg/s)
// Returns max bank angle in degrees
#if ENABLE_FLIGHT_PLAN
static float calculateMaxBankAngle(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    // Get current ground speed in m/s
    const float velocityMS = gpsSol.groundSpeed / 100.0f;  // cm/s to m/s

    // Minimum velocity for coordinated turn calculation
    // Below this, use configured maxAngle
    if (velocityMS < 1.0f) {
        return cfg->maxAngle;
    }

    // Calculate bank angle for coordinated turn
    // turn_rate = (g * tan(bank_angle)) / velocity
    // bank_angle = atan((turn_rate * velocity) / g)
    const float g = 9.81f;  // m/s²
    const float turnRateRad = cfg->maxTurnRate * 0.0174533f;  // deg/s to rad/s
    const float bankAngleRad = atanf((turnRateRad * velocityMS) / g);
    const float bankAngleDeg = bankAngleRad * 57.2958f;  // rad to deg

    // Limit to configured max angle (don't exceed structural/control limits)
    return fminf(bankAngleDeg, cfg->maxAngle);
}
#endif

// Cascaded velocity controller with quadratic drag compensation
// Outer loop: Position error → Velocity setpoint
// Inner loop: Velocity error → Pitch angle (with drag model)
static float velocityToAngle(efPidAxis_t *efAxis, float positionError, float currentVelocity, float gpsDataInterval, const autopilotConfig_t *cfg)
{
    // Outer loop: Position → Velocity setpoint
    // Simple proportional control: velocity setpoint proportional to position error
    const float posToVelGain = cfg->positionP * 0.1f;  // Scale position P for velocity setpoint
    efAxis->velocitySetpoint = positionError * posToVelGain;

    // Limit velocity setpoint to configured maximum
    const float maxVel = (float)cfg->maxVelocity;
    efAxis->velocitySetpoint = constrainf(efAxis->velocitySetpoint, -maxVel, maxVel);

    // Inner loop: Velocity error → Angle with drag compensation
    const float velocityError = efAxis->velocitySetpoint - currentVelocity;

    // Velocity PID controller
    const float velP = velocityError * (cfg->velocityP * 0.1f);

    // Velocity I-term (with anti-windup)
    if (!efAxis->isStopping) {
        efAxis->velocityIntegral += velocityError * (cfg->velocityI * 0.1f) * gpsDataInterval;
        // Limit integral to ±10 degrees
        efAxis->velocityIntegral = constrainf(efAxis->velocityIntegral, -10.0f, 10.0f);
    } else {
        // Decay integral when stopping
        efAxis->velocityIntegral *= 0.98f;
    }
    const float velI = efAxis->velocityIntegral;

    // Velocity D-term (on error, not measurement)
    const float velocityErrorRate = (velocityError - efAxis->previousVelocityError) / gpsDataInterval;
    efAxis->previousVelocityError = velocityError;
    const float velD = velocityErrorRate * (cfg->velocityD * 0.1f);

    // Quadratic drag compensation
    // Drag force opposes motion: F_drag = k * v²
    // To maintain velocity, need pitch angle to compensate
    // Small angle approximation: pitch ≈ drag_force / (m * g)
    // Simplified: drag_term = k_drag * velocity²
    const float dragCoeff = cfg->velocityDragCoeff * 0.0001f;  // Scale from 10000
    const float velocitySign = (currentVelocity >= 0.0f) ? 1.0f : -1.0f;
    const float dragCompensation = dragCoeff * currentVelocity * fabsf(currentVelocity) * velocitySign;

    // Total angle output (degrees)
    float angleOutput = velP + velI + velD + dragCompensation;

    // Limit to max angle
    angleOutput = constrainf(angleOutput, -cfg->maxAngle, cfg->maxAngle);

    return angleOutput;
}

bool positionControl(void)
{
    unsigned debugAxis = gyroConfig()->gyro_filter_debug_axis;
    static vector2_t debugGpsDistance = { 0 };     // keep last calculated distance for DEBUG
    static vector2_t debugPidSumEF = { 0 };        // and last pidsum in EF
    static uint16_t gpsStamp = 0;
    if (gpsHasNewData(&gpsStamp)) {
        const float gpsDataInterval = getGpsDataIntervalSeconds(); // interval for current GPS data value 0.05 - 2.5s
        const float gpsDataFreq = getGpsDataFrequencyHz();

        // Update rolling origin (resets every 500m for precision)
        navOriginUpdate(&gpsSol.llh);

        // Convert current and target positions to NED coordinates using rolling origin
        vector3_t currentNED, targetNED;
        navOriginLLHtoNED(&gpsSol.llh, &currentNED);
        navOriginLLHtoNED(&ap.targetLocation, &targetNED);

        // Extract 2D position (North, East) from 3D NED
        vector2_t currentPos2D = { .x = currentNED.x, .y = currentNED.y };  // North, East
        vector2_t targetPos2D = { .x = targetNED.x, .y = targetNED.y };     // North, East

#if ENABLE_FLIGHT_PLAN
        // L1 Nonlinear Guidance - use carrot point if active
        const autopilotConfig_t *cfg = autopilotConfig();
        if (cfg->l1Enable && l1GuidanceIsActive()) {
            // Update L1 with current position and groundspeed
            l1GuidanceUpdate(&currentPos2D, gpsSol.groundSpeed);

            // Use carrot point as target (if L1 is still active after update)
            if (l1GuidanceIsActive()) {
                const vector2_t *carrot = l1GuidanceGetCarrot();
                targetPos2D = *carrot;
            }
        }

        // Debug output for navigation (rolling origin + L1 guidance)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 0, lrintf(navOriginGetDistanceCm() / 100.0f));  // Origin distance (m)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 1, lrintf(l1GuidanceGetCrossTrackError()));     // XTE (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 2, lrintf(l1GuidanceGetAlongTrackDistance())); // Along-track (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 3, lrintf(l1GuidanceGetLookaheadDistance()));  // Lookahead (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 4, lrintf(currentPos2D.x));                    // Current North (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 5, lrintf(currentPos2D.y));                    // Current East (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 6, lrintf(targetPos2D.x));                     // Target/Carrot North (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 7, lrintf(targetPos2D.y));                     // Target/Carrot East (cm)
#endif

        // Calculate position error in NED frame
        // NED 3D vector: North=X, East=Y, Down=Z
        // PID controller 2D array: efAxis[EAST]=index 0, efAxis[NORTH]=index 1
        vector2_t gpsDistance;
        gpsDistance.x = targetPos2D.y - currentPos2D.y;  // East error → efAxis[EAST]
        gpsDistance.y = targetPos2D.x - currentPos2D.x;  // North error → efAxis[NORTH]
        debugGpsDistance = gpsDistance;
        const float distanceNormCm = vector2Norm(&gpsDistance);

        // ** Sanity check **
        // primarily to detect flyaway from no Mag or badly oriented Mag
        // must accept some overshoot at the start, especially if entering at high speed
        if (distanceNormCm > ap.sanityCheckDistance) {
            return false;
        }

        // update filters according to current GPS update rate
        const float vaGain = pt1FilterGain(ap.vaLpfCutoff, gpsDataInterval);
        const float iTermLeakGain = 1.0f - pt1FilterGainFromDelay(2.5f, gpsDataInterval);   // 2.5s time constant
        vector2_t pidSum = { 0 };       // P+I in loop, D+A added after the axis loop (after limiting it)
        vector2_t pidDA;                // D+A

        for (axisEF_e efAxisIdx = EAST; efAxisIdx <= NORTH; efAxisIdx++) {
            efPidAxis_t *efAxis = &ap.efAxis[efAxisIdx];
            const autopilotConfig_t *cfg = autopilotConfig();
            // separate PID controllers for longitude (EastWest or EW, X) and latitude (NorthSouth or NS, Y)
            const float axisDistance = gpsDistance.v[efAxisIdx];

            // Calculate current velocity
            const float velocity = (axisDistance - efAxis->previousDistance) * gpsDataFreq; // cm/s
            efAxis->previousDistance = axisDistance;
            pt1FilterUpdateCutoff(&efAxis->velocityLpf, vaGain);
            const float velocityFiltered = pt1FilterApply(&efAxis->velocityLpf, velocity);

            float pidP = 0.0f, pidI = 0.0f, pidD = 0.0f, pidA = 0.0f;

            if (cfg->velocityControlEnable) {
                // ** NEW: Cascaded velocity controller with drag compensation **
                const float angleOutput = velocityToAngle(efAxis, axisDistance, velocityFiltered, gpsDataInterval, cfg);
                pidSum.v[efAxisIdx] = angleOutput;
                // No separate D and A terms in velocity mode
                pidP = angleOutput;  // For debug display
            } else {
                // ** LEGACY: Direct position→angle PID controller **
                // ** P **
                pidP = axisDistance * positionPidCoeffs.Kp;
                pidSum.v[efAxisIdx] += pidP;

                // ** I **
                // only add to iTerm while in hold phase
                efAxis->integral += efAxis->isStopping ? 0.0f : axisDistance * gpsDataInterval;
                pidI = efAxis->integral * positionPidCoeffs.Ki;
                pidSum.v[efAxisIdx] += pidI;

                // ** D **
                pidD = velocityFiltered * positionPidCoeffs.Kd;

                // ** A (acceleration feedforward) **
                float acceleration = (velocityFiltered - efAxis->previousVelocity) * gpsDataFreq;
                efAxis->previousVelocity = velocityFiltered;
                pt1FilterUpdateCutoff(&efAxis->accelerationLpf, vaGain);
                const float accelerationFiltered = pt1FilterApply(&efAxis->accelerationLpf, acceleration);
                pidA = accelerationFiltered * positionPidCoeffs.Kf;
            }

            if (ap.sticksActive) {
                // sticks active 'phase', prepare to enter stopping
                efAxis->isStopping = true;
                // slowly leak iTerm away (both position and velocity integrals)
                efAxis->integral *= iTermLeakGain;
                efAxis->velocityIntegral *= iTermLeakGain;
                efAxis->previousDistance = 0.0f; // avoid D and A spikes
                efAxis->previousVelocityError = 0.0f;
                // rest is handled after axis loop
            } else if (efAxis->isStopping) {
                // 'phase' after sticks are centered, but before craft has stopped; in given Earth axis
                if (!cfg->velocityControlEnable) {
                    pidD *= 1.6f; // arbitrary D boost to stop more quickly (legacy mode only)
                }
                // detect when axis has nearly stopped by sign reversal of velocity
                if (velocity * velocityFiltered < 0.0f) {
                    setTargetLocationByAxis(&gpsSol.llh, efAxisIdx);  // reset target location for this axis, forcing P to zero
                    efAxis->previousDistance = 0.0f;                  // ensure minimal D jump from the updated location
                    efAxis->previousVelocityError = 0.0f;
                    efAxis->isStopping = false;                       // end the 'stopping' phase
                    if (ap.efAxis[NORTH].isStopping == ap.efAxis[EAST].isStopping) {
                        // when both axes have stopped moving, reset the sanity distance to 10m default
                        ap.sanityCheckDistance = sanityCheckDistance(1000);
                    }
                }
            }
            pidDA.v[efAxisIdx] = pidD + pidA;    // save DA here, processed after axis loop (legacy mode only)
            if (debugAxis == efAxisIdx) {
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(distanceNormCm));   // same for both axes
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(pidP * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(pidI * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(pidD * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(pidA * 10));
            }
        } // end for loop

        // D+A limiting (legacy mode only)
        if (!autopilotConfig()->velocityControlEnable) {
            // limit sum of D and A per axis based on total DA vector length, otherwise can be too aggressive when starting at speed
            // limit is 35 degrees from D and A alone, arbitrary value.  20 is a bit too low, allows a lot of overshoot
            // note: an angle of more than 35 degrees can still be achieved as P and I grow
            const float maxDAAngle = 35.0f; // D+A limit in degrees; arbitrary angle
            const float mag = vector2Norm(&pidDA);
            if (mag > maxDAAngle) {
                vector2Scale(&pidDA, &pidDA, maxDAAngle / mag);
            }
        }

        // add constrained DA to sum (legacy mode only)
        if (!autopilotConfig()->velocityControlEnable) {
            vector2Add(&pidSum, &pidSum, &pidDA);
        }
        debugPidSumEF = pidSum;
        vector2_t anglesBF;

        if (ap.sticksActive) {
            // if a Position Hold deadband is set, and sticks are outside deadband, allow pilot control in angle mode
            anglesBF = (vector2_t){{0, 0}};             // set output PIDS to 0; upsampling filter will smooth this
            // reset target location each cycle (and set previousDistance to zero in for loop), to keep D current, and avoid a spike when stopping
            ap.targetLocation = gpsSol.llh;
            // keep updating sanity check distance while sticks are out because speed may get high
            ap.sanityCheckDistance = sanityCheckDistance(gpsSol.groundSpeed);
        } else {
            // ** Rotate PID sum from NED frame to body frame **
            // PID is running in NED frame (North=0°, East=90°, clockwise)
            // attitude.values.yaw is in centidegrees, clockwise from North (NED convention)
            // Rotation from Earth Frame (NED) to Body Frame
            const float angle = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
            vector2_t pidBodyFrame;   // PID output in body frame; X is forward, Y is right
            vector2Rotate(&pidBodyFrame, &pidSum, angle);         // Rotate by yaw angle
            anglesBF.v[AI_ROLL] = -pidBodyFrame.y;         // Negative roll for left bank
            anglesBF.v[AI_PITCH] = pidBodyFrame.x;         // Positive pitch for nose up
             // limit angle vector to maxAngle
            const float mag = vector2Norm(&anglesBF);
#if ENABLE_FLIGHT_PLAN
            // Apply turn rate coordination when in waypoint navigation
            const float effectiveMaxAngle = FLIGHT_MODE(AUTOPILOT_MODE) ?
                                            calculateMaxBankAngle() : ap.maxAngle;
#else
            const float effectiveMaxAngle = ap.maxAngle;
#endif
            if (mag > effectiveMaxAngle && mag > 0.0f) {
                vector2Scale(&anglesBF, &anglesBF, effectiveMaxAngle / mag);
            }
        }
        ap.pidSumBF = anglesBF;    // this value will be upsampled
    }

    // Final output to pid.c Angle Mode at 100Hz with PT3 upsampling
    for (unsigned i = 0; i < RP_AXIS_COUNT; i++) {
        // note: upsampling should really be done in earth frame, to avoid 10Hz wobbles if pilot yaws and the controller is applying significant pitch or roll
        autopilotAngle[i] = pt3FilterApply(&ap.upsampleLpfBF[i], ap.pidSumBF.v[i]);
    }

    if (debugAxis < 2) {
        // this is different from @ctzsnooze version
        // debugAxis = 0: store longitude + roll
        // debugAxis = 1: store latitude + pitch
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(debugGpsDistance.v[debugAxis]));    // cm
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(debugPidSumEF.v[debugAxis] * 10));  // deg * 10
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(autopilotAngle[debugAxis] * 10));   // deg * 10
    }
    return true;
}

bool isBelowLandingAltitude(void)
{
    return getAltitudeCm() < 100.0f * autopilotConfig()->landingAltitudeM;
}

float getAutopilotThrottle(void)
{
    return throttleOut;
}

bool isAutopilotInControl(void)
{
    return !ap.sticksActive;
}

#if ENABLE_FLIGHT_PLAN

// Global state for velocity buildup
static timeUs_t velocityBuildupStartTime = 0;
static bool velocityBuildupActive = false;

// Calculate velocity buildup pitch bias
// Returns additional pitch angle (centidegrees) to add for acceleration
static float calculateVelocityBuildupBias(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    // Check if we're below minimum velocity and need to accelerate
    if (gpsSol.groundSpeed >= cfg->minYawVelocity) {
        // Velocity is sufficient - no bias needed
        velocityBuildupActive = false;
        return 0.0f;
    }

    // Below minimum velocity - need to accelerate
    if (!velocityBuildupActive) {
        // Just entered velocity buildup phase
        velocityBuildupActive = true;
        velocityBuildupStartTime = micros();
    }

    // Calculate time in buildup phase (seconds)
    const float buildupTimeS = (micros() - velocityBuildupStartTime) / 1000000.0f;

    // Progressive pitch bias: ramp up over 2 seconds
    // Start at 2° pitch forward, ramp to max over 2 seconds
    const float minBias = 200.0f;  // 2° in centidegrees
    const float maxBias = cfg->velocityBuildupMaxPitch * 100.0f;  // convert degrees to centidegrees
    const float rampTime = 2.0f;  // seconds

    float bias = minBias + (maxBias - minBias) * constrainf(buildupTimeS / rampTime, 0.0f, 1.0f);

    // Safety timeout: after 5 seconds, assume something is wrong
    // Could be on ground, blocked, or position controller fighting us
    // Reduce bias to prevent flyaway
    if (buildupTimeS > 5.0f) {
        bias = minBias;
    }

    return bias;  // Return in centidegrees
}

// Calculate desired yaw rate for multirotor heading control
static void calculateYawControl(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    // Get current yaw (centidegrees, 0-35999)
    const float currentYaw = attitude.values.yaw;  // centidegrees

    float targetYaw = 0.0f;  // Target yaw in centidegrees
    bool yawControlActive = false;  // Is yaw control active this cycle

    switch (cfg->yawMode) {
    case YAW_MODE_VELOCITY:
        // Point nose in direction of movement
        // Only apply when moving above minimum velocity
        if (gpsSol.groundSpeed >= cfg->minYawVelocity) {
            targetYaw = gpsSol.groundCourse * 10.0f;  // decidegrees to centidegrees
            yawControlActive = true;
        } else {
            // Too slow, maintain current heading
            targetYaw = currentYaw;
            yawControlActive = false;
        }
        break;

    case YAW_MODE_BEARING:
        // Point nose toward waypoint
        targetYaw = waypointGetBearingCdeg();
        yawControlActive = true;
        break;

    case YAW_MODE_HYBRID: {
        // Blend between bearing and velocity based on distance
        // Close to waypoint: use velocity direction
        // Far from waypoint: use bearing
        const uint32_t distCm = waypointGetDistanceCm();
        const uint32_t blendDistance = 1000;  // 10m blend zone

        float blend = constrainf((float)distCm / blendDistance, 0.0f, 1.0f);

        float velocityYaw = gpsSol.groundCourse * 10.0f;
        float bearingYaw = waypointGetBearingCdeg();

        // Use bearing when far, velocity when close
        targetYaw = bearingYaw * blend + velocityYaw * (1.0f - blend);
        yawControlActive = true;
        break;
    }

    case YAW_MODE_FIXED:
    default:
        // No yaw control
        yawControlActive = false;
        break;
    }

    // Soft-start yaw: Track when yaw control becomes active
    const timeUs_t currentTimeUs = micros();

    if (yawControlActive && !ap.yawControlWasActive) {
        // Yaw control just activated - start the soft-start timer
        ap.yawControlStartTime = currentTimeUs;
    } else if (!yawControlActive) {
        // Yaw control inactive - reset timer
        ap.yawControlStartTime = 0;
    }

    ap.yawControlWasActive = yawControlActive;

    if (!yawControlActive) {
        ap.desiredYawRate = 0.0f;
        return;
    }

    // Calculate shortest yaw error (handle wrap-around at 360 degrees)
    float yawError = targetYaw - currentYaw;

    // Normalize to ±180 degrees (±18000 centidegrees)
    if (yawError > 18000.0f) {
        yawError -= 36000.0f;
    } else if (yawError < -18000.0f) {
        yawError += 36000.0f;
    }

    // P controller: yawError is in centidegrees, output in deg/s
    const float Kp = cfg->yawP * 0.01f;  // Scale from config (100 = 1.0)
    float yawRate = yawError * 0.01f * Kp;  // Convert centidegrees to degrees

    // Constrain to max yaw rate
    yawRate = constrainf(yawRate, -(float)cfg->maxYawRate, (float)cfg->maxYawRate);

    // Apply soft-start gain (0.0 → 1.0 over 2 seconds)
    const float softStartDuration = 2000000.0f;  // 2 seconds in microseconds
    const float elapsedTime = currentTimeUs - ap.yawControlStartTime;
    const float softStartGain = constrainf(elapsedTime / softStartDuration, 0.0f, 1.0f);

    ap.desiredYawRate = yawRate * softStartGain;
}

// Getter function for rc.c integration
float autopilotGetYawRate(void)
{
    return ap.desiredYawRate;
}

// Detect pilot override on any axis
static void detectPilotOverride(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    // Check stick deflection against deadband
    // rcCommandFromReceiver[] contains raw receiver data (bypasses autopilot)
    // This is updated by updateRcCommands() before autopilot task runs
    // Roll/Pitch/Yaw: ±500 range
    // Throttle: 1000-2000 range

    const float stickDeadband = cfg->stickDeadband;
    const float throttleDeadband = cfg->throttleDeadband;

    // Roll, Pitch, Yaw: Check deflection from center position (0)
    ap.pilotOverrideRoll = (fabsf(rcCommandFromReceiver[ROLL]) > stickDeadband);
    ap.pilotOverridePitch = (fabsf(rcCommandFromReceiver[PITCH]) > stickDeadband);
    ap.pilotOverrideYaw = (fabsf(rcCommandFromReceiver[YAW]) > stickDeadband);

    // Throttle: Check deflection from 0 position (1000 = throttle low)
    const float throttleDeviation = fabsf(rcCommandFromReceiver[THROTTLE] - 1000.0f);
    ap.pilotOverrideThrottle = (throttleDeviation > throttleDeadband);

    // Update stick status for position controller
    const bool sticksActive = ap.pilotOverrideRoll || ap.pilotOverridePitch;
    setSticksActiveStatus(sticksActive);
}

// Generate RC commands from autopilot outputs
static void generateRcCommands(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    // CRITICAL: Fuse pilot and autopilot commands
    // If pilot is overriding an axis, use receiver data
    // Otherwise, use autopilot command

    // Roll and Pitch: Convert angles (centidegrees) to RC range (±500)
    // autopilotAngle[] is in centidegrees
    if (!ap.pilotOverrideRoll) {
        float rollAngle = autopilotAngle[ROLL] * 0.01f;  // Convert to degrees
        rcCommand[ROLL] = constrainf(rollAngle * 500.0f / cfg->maxAngle, -500.0f, 500.0f);
    } else {
        rcCommand[ROLL] = rcCommandFromReceiver[ROLL];
    }

    if (!ap.pilotOverridePitch) {
        float pitchAngle = autopilotAngle[PITCH] * 0.01f;  // Convert to degrees
        rcCommand[PITCH] = constrainf(pitchAngle * 500.0f / cfg->maxAngle, -500.0f, 500.0f);
    } else {
        rcCommand[PITCH] = rcCommandFromReceiver[PITCH];
    }

    // Yaw: Convert rate (deg/s) to RC range (±500)
    if (!ap.pilotOverrideYaw) {
        rcCommand[YAW] = constrainf(ap.desiredYawRate * 500.0f / cfg->maxYawRate, -500.0f, 500.0f);
    } else {
        rcCommand[YAW] = rcCommandFromReceiver[YAW];
    }

    // Throttle: Convert 0.0-1.0 to 1000-2000 PWM
    if (!ap.pilotOverrideThrottle) {
        float throttle = getAutopilotThrottle();
        rcCommand[THROTTLE] = 1000.0f + throttle * 1000.0f;
        rcCommand[THROTTLE] = constrainf(rcCommand[THROTTLE], 1000.0f, 2000.0f);
    } else {
        rcCommand[THROTTLE] = rcCommandFromReceiver[THROTTLE];
    }
}

// Main autopilot update function, called at 100Hz from tasks.c.
// Position PID runs at GPS rate (5-10Hz) via gpsHasNewData() guard in positionControl(),
// then PT3 upsamples the output to 100Hz for smooth angle mode control.
// Altitude, yaw, orbit patterns, and RC generation run every cycle.
void updateAutopilot(void)
{
    // 0. Defensive check: should not be called if mode is not active
    // But if it is, reset velocity buildup state
    if (!FLIGHT_MODE(AUTOPILOT_MODE)) {
        velocityBuildupActive = false;
        return;
    }

    // 1. Update waypoint state machine
    waypointUpdate(micros());

    // 2. Safety: check if waypoint system is valid
    if (!waypointIsSystemValid()) {
        DISABLE_FLIGHT_MODE(AUTOPILOT_MODE);
        velocityBuildupActive = false;
        return;
    }

    // 3. Get current target from waypoint tracker
    const gpsLocation_t *target = waypointGetTarget();

    // 4. Update position controller target on GPS updates
    static uint16_t gpsStamp = 0;
    if (gpsHasNewData(&gpsStamp)) {
        ap.targetLocation = *target;
    }

    // 5. Run position controller (updates autopilotAngle[] array)
    if (!positionControl()) {
        // Flyaway detection triggered
        DISABLE_FLIGHT_MODE(AUTOPILOT_MODE);
        velocityBuildupActive = false;  // Reset state on flyaway
        return;
    }

    // 5a. Apply velocity buildup bias if below minimum velocity
    // CRITICAL: This ensures we accelerate forward even from stationary hover
    if (gpsSol.groundSpeed < autopilotConfig()->minYawVelocity) {
        const float pitchBias = calculateVelocityBuildupBias();  // Returns centidegrees

        // Apply pitch bias to accelerate forward
        // TODO: In future, rotate bias into body frame based on bearing to waypoint
        // For now, simple addition assumes aircraft is roughly oriented toward target
        autopilotAngle[PITCH] += pitchBias;

        // Limit total pitch to safety bounds (±45° = ±4500 centidegrees)
        autopilotAngle[PITCH] = constrainf(autopilotAngle[PITCH], -4500.0f, 4500.0f);
    }

    // 6. Get target altitude from current waypoint
    const flightPlanConfig_t *plan = flightPlanConfig();
    const waypoint_t *wp = &plan->waypoints[waypointGetCurrentIndex()];
    float targetAltCm = wp->altitude;

    // 7. Run altitude controller (updates internal throttle)
    altitudeControl(targetAltCm, 0.01f, 0.0f);

    // 8. Calculate yaw control
    calculateYawControl();

    // 9. Detect pilot override
    detectPilotOverride();

    // 10. Generate RC commands
    generateRcCommands();
}

#endif // ENABLE_FLIGHT_PLAN

#endif // !USE_WING
