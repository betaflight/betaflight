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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "build_config.h"
#include "platform.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"

#include "config/runtime_config.h"
#include "config/config.h"

#if defined(NAV)

static bool isPitchAndThrottleAdjustmentValid = false;
static bool isRollAdjustmentValid = false;

/*-----------------------------------------------------------
 * Backdoor to MW heading controller
 *-----------------------------------------------------------*/
extern int16_t magHold;

/*-----------------------------------------------------------
 * Altitude controller
 *-----------------------------------------------------------*/
void setupFixedWingAltitudeController(void)
{
    // TODO
}

void resetFixedWingAltitudeController()
{
    navPidReset(&posControl.pids.fw_alt);
    posControl.rcAdjustment[PITCH] = 0;
    isPitchAndThrottleAdjustmentValid = false;
}

bool adjustFixedWingAltitudeFromRCInput(void)
{
    int16_t rcAdjustment = applyDeadband(rcCommand[PITCH], posControl.rcControlsConfig->alt_hold_deadband);

    if (rcAdjustment) {
        // set velocity proportional to stick movement
        float rcClimbRate = -rcAdjustment * posControl.navConfig->max_manual_climb_rate / (500.0f - posControl.rcControlsConfig->alt_hold_deadband);
        updateAltitudeTargetFromClimbRate(rcClimbRate);
        return true;
    }
    else {
        return false;
    }
}

// Position to velocity controller for Z axis
static void updateAltitudeVelocityAndPitchController_FW(uint32_t deltaMicros)
{
    static filterStatePt1_t velzFilterState;

    // On a fixed wing we might not have a reliable climb rate source (if no BARO available), so we can't apply PID controller to
    // velocity error. We use PID controller on altitude error and calculate desired pitch angle from desired climb rate and forward velocity

    // FIXME: Use airspeed here
    float forwardVelocity = sqrtf(sq(posControl.actualState.vel.V.X) + sq(posControl.actualState.vel.V.Y));
    forwardVelocity = MAX(forwardVelocity, 300.0f);   // Limit min velocity for PID controller at about 10 km/h

    // Calculate max climb rate from current forward velocity and maximum pitch angle (climb angle is fairly small, approximate tan=sin)
    float maxVelocityClimb = forwardVelocity * sin_approx(DEGREES_TO_RADIANS(posControl.navConfig->fw_max_climb_angle));
    float maxVelocityDive = -forwardVelocity * sin_approx(DEGREES_TO_RADIANS(posControl.navConfig->fw_max_dive_angle));

    posControl.desiredState.vel.V.Z = navPidApply2(posControl.desiredState.pos.V.Z, posControl.actualState.pos.V.Z, US2S(deltaMicros), &posControl.pids.fw_alt, maxVelocityDive, maxVelocityClimb, false);
    posControl.desiredState.vel.V.Z = filterApplyPt1(posControl.desiredState.vel.V.Z, &velzFilterState, NAV_FW_VEL_CUTOFF_FREQENCY_HZ, US2S(deltaMicros));

    // Calculate climb angle ( >0 - climb, <0 - dive)
    int16_t climbAngleDeciDeg = RADIANS_TO_DECIDEGREES(atan2_approx(posControl.desiredState.vel.V.Z, forwardVelocity));
    climbAngleDeciDeg = constrain(climbAngleDeciDeg, -posControl.navConfig->fw_max_dive_angle * 10, posControl.navConfig->fw_max_climb_angle * 10);
    posControl.rcAdjustment[PITCH] = climbAngleDeciDeg;

    // Calculate throttle adjustment
    posControl.rcAdjustment[THROTTLE] = posControl.navConfig->fw_cruise_throttle + DECIDEGREES_TO_DEGREES(climbAngleDeciDeg) * posControl.navConfig->fw_pitch_to_throttle;
    posControl.rcAdjustment[THROTTLE] = constrain(posControl.rcAdjustment[THROTTLE], posControl.navConfig->fw_min_throttle, posControl.navConfig->fw_max_throttle);

#if defined(NAV_BLACKBOX)
    navDesiredVelocity[Z] = constrain(posControl.desiredState.vel.V.Z, -32678, 32767);
    navTargetPosition[Z] = constrain(posControl.desiredState.pos.V.Z, -32678, 32767);
#endif
}

void applyFixedWingAltitudeController(uint32_t currentTime)
{
    static uint32_t previousTimePositionUpdate;         // Occurs @ altitude sensor update rate (max MAX_ALTITUDE_UPDATE_RATE_HZ)
    static uint32_t previousTimeUpdate;                 // Occurs @ looptime rate

    uint32_t deltaMicros = currentTime - previousTimeUpdate;
    previousTimeUpdate = currentTime;

    // If last time Z-controller was called is too far in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTime;
        previousTimePositionUpdate = currentTime;
        resetFixedWingAltitudeController();
        return;
    }

    if (posControl.flags.hasValidPositionSensor) {
        // If we have an update on vertical position data - update velocity and accel targets
        if (posControl.flags.verticalPositionNewData) {
            uint32_t deltaMicrosPositionUpdate = currentTime - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTime;

            // Check if last correction was too log ago - ignore this update
            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                updateAltitudeVelocityAndPitchController_FW(deltaMicrosPositionUpdate);
            }
            else {
                // due to some glitch position update has not occurred in time, reset altitude controller
                resetFixedWingAltitudeController();
            }

            // Indicate that information is no longer usable
            posControl.flags.verticalPositionNewData = 0;
        }

        isPitchAndThrottleAdjustmentValid = true;
    }
    else {
        // No valid altitude sensor data, don't adjust pitch automatically, rcCommand[PITCH] is passed through to PID controller
        isPitchAndThrottleAdjustmentValid = false;
    }
}

/*-----------------------------------------------------------
 * Adjusts desired heading from pilot's input
 *-----------------------------------------------------------*/
bool adjustFixedWingHeadingFromRCInput(void)
{
    return false;
}

/*-----------------------------------------------------------
 * XY-position controller for multicopter aircraft
 *-----------------------------------------------------------*/
t_fp_vector virtualDesiredPosition;

void resetFixedWingPositionController(void)
{
    virtualDesiredPosition.V.X = 0;
    virtualDesiredPosition.V.Y = 0;
    virtualDesiredPosition.V.Z = 0;

    navPidReset(&posControl.pids.fw_nav);
    posControl.rcAdjustment[ROLL] = 0;
    isRollAdjustmentValid = false;
}

static void calculateVirtualPositionTarget_FW(float trackingPeriod)
{
    float posErrorX = posControl.desiredState.pos.V.X - posControl.actualState.pos.V.X;
    float posErrorY = posControl.desiredState.pos.V.Y - posControl.actualState.pos.V.Y;

    float distanceToActualTarget = sqrtf(sq(posErrorX) + sq(posErrorY));
    float forwardVelocity = sqrtf(sq(posControl.actualState.vel.V.X) + sq(posControl.actualState.vel.V.Y));

    // Limit minimum forward velocity to 1 m/s
    float trackingDistance = trackingPeriod * MAX(forwardVelocity, 100.0f);

    // If angular visibility of a waypoint is less than 30deg, don't calculate circular loiter, go straight to the target
    #define TAN_15DEG    0.26795f
    bool needToCalculateCircularLoiter = isApproachingLastWaypoint()
                                            && (distanceToActualTarget <= (posControl.navConfig->fw_loiter_radius / TAN_15DEG))
                                            && (distanceToActualTarget > 50.0f);

    // Calculate virtual position for straight movement
    if (needToCalculateCircularLoiter) {
        // We are closing in on a waypoint, calculate circular loiter
        float loiterAngle = atan2_approx(-posErrorY, -posErrorX) + DEGREES_TO_RADIANS(45.0f);

        float loiterTargetX = posControl.desiredState.pos.V.X + posControl.navConfig->fw_loiter_radius * cos_approx(loiterAngle);
        float loiterTargetY = posControl.desiredState.pos.V.Y + posControl.navConfig->fw_loiter_radius * sin_approx(loiterAngle);

        // We have temporary loiter target. Recalculate distance and position error
        posErrorX = loiterTargetX - posControl.actualState.pos.V.X;
        posErrorY = loiterTargetY - posControl.actualState.pos.V.Y;
        distanceToActualTarget = sqrtf(sq(posErrorX) + sq(posErrorY));
    }

    // Calculate virtual waypoint
    virtualDesiredPosition.V.X = posControl.actualState.pos.V.X + posErrorX * (trackingDistance / distanceToActualTarget);
    virtualDesiredPosition.V.Y = posControl.actualState.pos.V.Y + posErrorY * (trackingDistance / distanceToActualTarget);

    // Shift position according to pilot's ROLL input (up to max_manual_speed velocity)
    if (posControl.flags.isAdjustingPosition) {
        int16_t rcRollAdjustment = applyDeadband(rcCommand[ROLL], posControl.rcControlsConfig->pos_hold_deadband);

        if (rcRollAdjustment) {
            float rcShiftY = rcRollAdjustment * posControl.navConfig->max_manual_speed / 500.0f * trackingPeriod;

            // Rotate this target shift from body frame to to earth frame and apply to position target
            virtualDesiredPosition.V.X += -rcShiftY * posControl.actualState.sinYaw;
            virtualDesiredPosition.V.Y +=  rcShiftY * posControl.actualState.cosYaw;

            posControl.flags.isAdjustingPosition = true;
        }
    }
}

bool adjustFixedWingPositionFromRCInput(void)
{
    int16_t rcRollAdjustment = applyDeadband(rcCommand[ROLL], posControl.rcControlsConfig->pos_hold_deadband);
    return (rcRollAdjustment);
}

static void updatePositionHeadingController_FW(uint32_t deltaMicros)
{
    // We have virtual position target, calculate heading error
    int32_t virtualTargetBearing = calculateBearingToDestination(&virtualDesiredPosition);

    // Calculate NAV heading error
    int32_t headingError = wrap_18000(virtualTargetBearing - posControl.actualState.yaw);

    // Forced turn direction
    if (ABS(headingError) > 17000) {
        headingError = 17500;
    }

    // Input error in (deg*100), output pitch angle (deg*100)
    float rollAdjustment = navPidApply2(posControl.actualState.yaw + headingError, posControl.actualState.yaw, US2S(deltaMicros), &posControl.pids.fw_nav,
                                       -DEGREES_TO_CENTIDEGREES(posControl.navConfig->fw_max_bank_angle),
                                        DEGREES_TO_CENTIDEGREES(posControl.navConfig->fw_max_bank_angle),
                                        false);

    // Convert rollAdjustment to decidegrees (rcAdjustment holds decidegrees)
    posControl.rcAdjustment[ROLL] = CENTIDEGREES_TO_DECIDEGREES(rollAdjustment);

    // Update magHold heading lock in case pilot is using MAG mode (prevent MAGHOLD to fight navigation)
    posControl.desiredState.yaw = wrap_36000(posControl.actualState.yaw + headingError);
    magHold = CENTIDEGREES_TO_DEGREES(posControl.desiredState.yaw);

    // Add pitch compensation
    //posControl.rcAdjustment[PITCH] = -CENTIDEGREES_TO_DECIDEGREES(ABS(rollAdjustment)) * 0.50f;
}

void applyFixedWingPositionController(uint32_t currentTime)
{
    static uint32_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static uint32_t previousTimeUpdate;                 // Occurs @ looptime rate

    uint32_t deltaMicros = currentTime - previousTimeUpdate;
    previousTimeUpdate = currentTime;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTime;
        previousTimePositionUpdate = currentTime;
        resetFixedWingPositionController();
        return;
    }

    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    if (posControl.flags.hasValidPositionSensor) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionNewData) {
            uint32_t deltaMicrosPositionUpdate = currentTime - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTime;

            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                // Calculate virtual position target at a distance of forwardVelocity * HZ2S(POSITION_TARGET_UPDATE_RATE_HZ)
                // Account for pilot's roll input (move position target left/right at max of max_manual_speed)
                // POSITION_TARGET_UPDATE_RATE_HZ should be chosen keeping in mind that position target shouldn't be reached until next pos update occurs
                // FIXME: verify the above
                calculateVirtualPositionTarget_FW(HZ2S(MIN_POSITION_UPDATE_RATE_HZ) * 2);

                updatePositionHeadingController_FW(deltaMicrosPositionUpdate);
            }
            else {
                resetFixedWingPositionController();
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionNewData = 0;
        }

        isRollAdjustmentValid = true;
    }
    else {
        // No valid pos sensor data, don't adjust pitch automatically, rcCommand[ROLL] is passed through to PID controller
        isRollAdjustmentValid = false;
    }
}

void applyFixedWingPitchRollThrottleController(void)
{
    int16_t pitchCorrection = 0;        // >0 climb, <0 dive
    int16_t rollCorrection = 0;         // >0 right, <0 left
    int16_t throttleCorrection = 0;     // raw throttle

    // Mix Pitch/Roll/Throttle
    if (isPitchAndThrottleAdjustmentValid) {
        pitchCorrection += posControl.rcAdjustment[PITCH];
        throttleCorrection += posControl.rcAdjustment[THROTTLE];
    }

    if (isRollAdjustmentValid) {
        pitchCorrection += ABS(posControl.rcAdjustment[ROLL]) * 0.5f;
        rollCorrection += posControl.rcAdjustment[ROLL];
    }

    // Limit and apply
    if (isPitchAndThrottleAdjustmentValid) {
        // PITCH angle is measured in opposite direction ( >0 - dive, <0 - climb)
        pitchCorrection = constrain(pitchCorrection, -DEGREES_TO_CENTIDEGREES(posControl.navConfig->fw_max_dive_angle), DEGREES_TO_CENTIDEGREES(posControl.navConfig->fw_max_climb_angle));
        rcCommand[PITCH] = -leanAngleToRcCommand(pitchCorrection);
        rcCommand[THROTTLE] = constrain(throttleCorrection, posControl.escAndServoConfig->minthrottle, posControl.escAndServoConfig->maxthrottle);
    }

    if (isRollAdjustmentValid) {
        rollCorrection = constrain(rollCorrection, -DEGREES_TO_CENTIDEGREES(posControl.navConfig->fw_max_bank_angle), DEGREES_TO_CENTIDEGREES(posControl.navConfig->fw_max_bank_angle));
        rcCommand[ROLL] = leanAngleToRcCommand(rollCorrection);
    }
}

/*-----------------------------------------------------------
 * FixedWing land detector
 *-----------------------------------------------------------*/
bool isFixedWingLandingDetected(uint32_t * landingTimer)
{
    uint32_t currentTime = micros();

    // TODO

    *landingTimer = currentTime;
    return false;
}

/*-----------------------------------------------------------
 * FixedWing emergency landing
 *-----------------------------------------------------------*/
void applyFixedWingEmergencyLandingController(void)
{
    // TODO
}

/*-----------------------------------------------------------
 * Calculate loiter target based on current position and velocity
 *-----------------------------------------------------------*/
void calculateFixedWingInitialHoldPosition(t_fp_vector * pos)
{
    // TODO: stub, this should account for velocity and target loiter radius
    *pos = posControl.actualState.pos;
}

void resetFixedWingHeadingController(void)
{
    magHold = CENTIDEGREES_TO_DEGREES(posControl.actualState.yaw);
}

void applyFixedWingNavigationController(navigationFSMStateFlags_t navStateFlags, uint32_t currentTime)
{
    if (navStateFlags & NAV_CTL_EMERG) {
        applyFixedWingEmergencyLandingController();
    }
    else {
        // Don't apply anything if ground speed is too low (<3m/s)
        float forwardVelocitySquared = sq(posControl.actualState.vel.V.X) + sq(posControl.actualState.vel.V.Y);
        if (forwardVelocitySquared > (300 * 300)) {
            if (navStateFlags & NAV_CTL_ALT)
                applyFixedWingAltitudeController(currentTime);

            if (navStateFlags & NAV_CTL_POS)
                applyFixedWingPositionController(currentTime);
        }
        else {
            posControl.rcAdjustment[PITCH] = 0;
            posControl.rcAdjustment[ROLL] = 0;
            posControl.rcAdjustment[THROTTLE] = 0;
        }

        //if (navStateFlags & NAV_CTL_YAW)
        if ((navStateFlags & NAV_CTL_ALT) || (navStateFlags & NAV_CTL_POS))
            applyFixedWingPitchRollThrottleController();
    }
}

#endif  // NAV
