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

#include "platform.h"

#if defined(NAV)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/system.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/navigation_rewrite_private.h"

#include "fc/runtime_config.h"
#include "config/config.h"

// If we are going slower than NAV_FW_MIN_VEL_SPEED_BOOST - boost throttle to fight against the wind
#define NAV_FW_THROTTLE_SPEED_BOOST_GAIN        1.5f
#define NAV_FW_MIN_VEL_SPEED_BOOST              700.0f      // 7 m/s

// If this is enabled navigation won't be applied if velocity is below 3 m/s
//#define NAV_FW_LIMIT_MIN_FLY_VELOCITY

static bool isPitchAdjustmentValid = false;
static bool isRollAdjustmentValid = false;
static float throttleSpeedAdjustment = 0;

/*-----------------------------------------------------------
 * Backdoor to MW
 *-----------------------------------------------------------*/
extern controlRateConfig_t *currentControlRateProfile;

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
    isPitchAdjustmentValid = false;
    throttleSpeedAdjustment = 0;
}

bool adjustFixedWingAltitudeFromRCInput(void)
{
    int16_t rcAdjustment = applyDeadband(rcCommand[PITCH], posControl.rcControlsConfig->alt_hold_deadband);

    if (rcAdjustment) {
        // set velocity proportional to stick movement
        float rcClimbRate = -rcAdjustment * posControl.navConfig->general.max_manual_climb_rate / (500.0f - posControl.rcControlsConfig->alt_hold_deadband);
        updateAltitudeTargetFromClimbRate(rcClimbRate, CLIMB_RATE_RESET_SURFACE_TARGET);
        return true;
    }
    else {
        return false;
    }
}

// Position to velocity controller for Z axis
static void updateAltitudeVelocityAndPitchController_FW(uint32_t deltaMicros)
{
    static pt1Filter_t velzFilterState;

    // On a fixed wing we might not have a reliable climb rate source (if no BARO available), so we can't apply PID controller to
    // velocity error. We use PID controller on altitude error and calculate desired pitch angle from desired climb rate and forward velocity

    // FIXME: Use airspeed here
    float forwardVelocity = MAX(posControl.actualState.velXY, 300.0f);   // Limit min velocity for PID controller at about 10 km/h

    // Calculate max climb rate from current forward velocity and maximum pitch angle (climb angle is fairly small, approximate tan=sin)
    float maxVelocityClimb = forwardVelocity * sin_approx(DEGREES_TO_RADIANS(posControl.navConfig->fw.max_climb_angle));
    float maxVelocityDive = -forwardVelocity * sin_approx(DEGREES_TO_RADIANS(posControl.navConfig->fw.max_dive_angle));

    posControl.desiredState.vel.V.Z = navPidApply2(posControl.desiredState.pos.V.Z, posControl.actualState.pos.V.Z, US2S(deltaMicros), &posControl.pids.fw_alt, maxVelocityDive, maxVelocityClimb, false);
    posControl.desiredState.vel.V.Z = pt1FilterApply4(&velzFilterState, posControl.desiredState.vel.V.Z, NAV_FW_VEL_CUTOFF_FREQENCY_HZ, US2S(deltaMicros));

    // Calculate climb angle ( >0 - climb, <0 - dive)
    int16_t climbAngleDeciDeg = RADIANS_TO_DECIDEGREES(atan2_approx(posControl.desiredState.vel.V.Z, forwardVelocity));
    climbAngleDeciDeg = constrain(climbAngleDeciDeg, -posControl.navConfig->fw.max_dive_angle * 10, posControl.navConfig->fw.max_climb_angle * 10);
    posControl.rcAdjustment[PITCH] = climbAngleDeciDeg;

#if defined(NAV_BLACKBOX)
    navDesiredVelocity[Z] = constrain(posControl.desiredState.vel.V.Z, -32678, 32767);
    navTargetPosition[Z] = constrain(posControl.desiredState.pos.V.Z, -32678, 32767);
#endif
}

void applyFixedWingAltitudeController(timeUs_t currentTimeUs)
{
    static uint32_t previousTimePositionUpdate;         // Occurs @ altitude sensor update rate (max MAX_ALTITUDE_UPDATE_RATE_HZ)
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    timeUs_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last time Z-controller was called is too far in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetFixedWingAltitudeController();
        return;
    }

    if (posControl.flags.hasValidPositionSensor) {
        // If we have an update on vertical position data - update velocity and accel targets
        if (posControl.flags.verticalPositionDataNew) {
            timeUs_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            // Check if last correction was too log ago - ignore this update
            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                updateAltitudeVelocityAndPitchController_FW(deltaMicrosPositionUpdate);
            }
            else {
                // due to some glitch position update has not occurred in time, reset altitude controller
                resetFixedWingAltitudeController();
            }

            // Indicate that information is no longer usable
            posControl.flags.verticalPositionDataConsumed = 1;
        }

        isPitchAdjustmentValid = true;
    }
    else {
        // No valid altitude sensor data, don't adjust pitch automatically, rcCommand[PITCH] is passed through to PID controller
        isPitchAdjustmentValid = false;
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
static t_fp_vector virtualDesiredPosition;
static pt1Filter_t fwPosControllerCorrectionFilterState;

void resetFixedWingPositionController(void)
{
    virtualDesiredPosition.V.X = 0;
    virtualDesiredPosition.V.Y = 0;
    virtualDesiredPosition.V.Z = 0;

    navPidReset(&posControl.pids.fw_nav);
    posControl.rcAdjustment[ROLL] = 0;
    isRollAdjustmentValid = false;

    pt1FilterReset(&fwPosControllerCorrectionFilterState, 0.0f);
}

static void calculateVirtualPositionTarget_FW(float trackingPeriod)
{
    float posErrorX = posControl.desiredState.pos.V.X - posControl.actualState.pos.V.X;
    float posErrorY = posControl.desiredState.pos.V.Y - posControl.actualState.pos.V.Y;

    float distanceToActualTarget = sqrtf(sq(posErrorX) + sq(posErrorY));

    // Limit minimum forward velocity to 1 m/s
    float trackingDistance = trackingPeriod * MAX(posControl.actualState.velXY, 100.0f);

    // If angular visibility of a waypoint is less than 30deg, don't calculate circular loiter, go straight to the target
    #define TAN_15DEG    0.26795f
    bool needToCalculateCircularLoiter = isApproachingLastWaypoint()
                                            && (distanceToActualTarget <= (posControl.navConfig->fw.loiter_radius / TAN_15DEG))
                                            && (distanceToActualTarget > 50.0f);

    // Calculate virtual position for straight movement
    if (needToCalculateCircularLoiter) {
        // We are closing in on a waypoint, calculate circular loiter
        float loiterAngle = atan2_approx(-posErrorY, -posErrorX) + DEGREES_TO_RADIANS(45.0f);

        float loiterTargetX = posControl.desiredState.pos.V.X + posControl.navConfig->fw.loiter_radius * cos_approx(loiterAngle);
        float loiterTargetY = posControl.desiredState.pos.V.Y + posControl.navConfig->fw.loiter_radius * sin_approx(loiterAngle);

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
            float rcShiftY = rcRollAdjustment * posControl.navConfig->general.max_manual_speed / 500.0f * trackingPeriod;

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
    static bool forceTurnDirection = false;

    // We have virtual position target, calculate heading error
    int32_t virtualTargetBearing = calculateBearingToDestination(&virtualDesiredPosition);

    // Calculate NAV heading error
    int32_t headingError = wrap_18000(virtualTargetBearing - posControl.actualState.yaw);

    // Forced turn direction
    // If heading error is close to 180 deg we initiate forced turn and only disable it when heading error goes below 90 deg
    if (ABS(headingError) > 17000) {
        forceTurnDirection = true;
    }
    else if (ABS(headingError) < 9000 && forceTurnDirection) {
        forceTurnDirection = false;
    }

    // If forced turn direction flag is enabled we fix the sign of the direction
    if (forceTurnDirection) {
        headingError = ABS(headingError);
    }

    // Input error in (deg*100), output pitch angle (deg*100)
    float rollAdjustment = navPidApply2(posControl.actualState.yaw + headingError, posControl.actualState.yaw, US2S(deltaMicros), &posControl.pids.fw_nav,
                                       -DEGREES_TO_CENTIDEGREES(posControl.navConfig->fw.max_bank_angle),
                                        DEGREES_TO_CENTIDEGREES(posControl.navConfig->fw.max_bank_angle),
                                        true);

    // Apply low-pass filter to prevent rapid correction
    rollAdjustment = pt1FilterApply4(&fwPosControllerCorrectionFilterState, rollAdjustment, NAV_FW_ROLL_CUTOFF_FREQUENCY_HZ, US2S(deltaMicros));

    // Convert rollAdjustment to decidegrees (rcAdjustment holds decidegrees)
    posControl.rcAdjustment[ROLL] = CENTIDEGREES_TO_DECIDEGREES(rollAdjustment);

    // Update magHold heading lock in case pilot is using MAG mode (prevent MAGHOLD to fight navigation)
    posControl.desiredState.yaw = wrap_36000(posControl.actualState.yaw + headingError);
    updateMagHoldHeading(CENTIDEGREES_TO_DEGREES(posControl.desiredState.yaw));

    // Add pitch compensation
    //posControl.rcAdjustment[PITCH] = -CENTIDEGREES_TO_DECIDEGREES(ABS(rollAdjustment)) * 0.50f;
}

void applyFixedWingPositionController(timeUs_t currentTimeUs)
{
    static uint32_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    timeUs_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        resetFixedWingPositionController();
        return;
    }

    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    if (posControl.flags.hasValidPositionSensor) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionDataNew) {
            timeUs_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

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
            posControl.flags.horizontalPositionDataConsumed = 1;
        }

        isRollAdjustmentValid = true;
    }
    else {
        // No valid pos sensor data, don't adjust pitch automatically, rcCommand[ROLL] is passed through to PID controller
        isRollAdjustmentValid = false;
    }
}

int16_t applyFixedWingMinSpeedController(timeUs_t currentTimeUs)
{
    static uint32_t previousTimePositionUpdate;         // Occurs @ GPS update rate
    static timeUs_t previousTimeUpdate;                 // Occurs @ looptime rate

    timeUs_t deltaMicros = currentTimeUs - previousTimeUpdate;
    previousTimeUpdate = currentTimeUs;

    // If last position update was too long in the past - ignore it (likely restarting altitude controller)
    if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
        previousTimeUpdate = currentTimeUs;
        previousTimePositionUpdate = currentTimeUs;
        throttleSpeedAdjustment = 0;
        return 0;
    }

    // Apply controller only if position source is valid
    if (posControl.flags.hasValidPositionSensor) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionDataNew) {
            timeUs_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
                float velThrottleBoost = (NAV_FW_MIN_VEL_SPEED_BOOST - posControl.actualState.velXY) * NAV_FW_THROTTLE_SPEED_BOOST_GAIN * US2S(deltaMicrosPositionUpdate);

                // If we are in the deadband of 50cm/s - don't update speed boost
                if (ABS(posControl.actualState.velXY - NAV_FW_MIN_VEL_SPEED_BOOST) > 50) {
                    throttleSpeedAdjustment += velThrottleBoost;
                }

                throttleSpeedAdjustment = constrainf(throttleSpeedAdjustment, 0.0f, 500.0f);
            }
            else {
                throttleSpeedAdjustment = 0;
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionDataConsumed = 1;
        }
    }
    else {
        // No valid pos sensor data, we can't calculate speed
        throttleSpeedAdjustment = 0;
    }

    return throttleSpeedAdjustment;
}

void applyFixedWingPitchRollThrottleController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    int16_t pitchCorrection = 0;        // >0 climb, <0 dive
    int16_t rollCorrection = 0;         // >0 right, <0 left
    int16_t throttleCorrection = 0;     // raw throttle

    int16_t minThrottleCorrection = posControl.navConfig->fw.min_throttle - posControl.navConfig->fw.cruise_throttle;
    int16_t maxThrottleCorrection = posControl.navConfig->fw.max_throttle - posControl.navConfig->fw.cruise_throttle;

    // Mix Pitch/Roll/Throttle
    if (isRollAdjustmentValid && (navStateFlags & NAV_CTL_POS)) {
        pitchCorrection += ABS(posControl.rcAdjustment[ROLL]) * (posControl.navConfig->fw.roll_to_pitch / 100.0f);
        rollCorrection += posControl.rcAdjustment[ROLL];
    }

    if (isPitchAdjustmentValid && (navStateFlags & NAV_CTL_ALT)) {
        pitchCorrection += posControl.rcAdjustment[PITCH];
        throttleCorrection += DECIDEGREES_TO_DEGREES(pitchCorrection) * posControl.navConfig->fw.pitch_to_throttle;
        throttleCorrection = constrain(throttleCorrection, minThrottleCorrection, maxThrottleCorrection);
    }

    // Speed controller - only apply in POS mode
    if (navStateFlags & NAV_CTL_POS) {
        throttleCorrection += applyFixedWingMinSpeedController(currentTimeUs);
        throttleCorrection = constrain(throttleCorrection, minThrottleCorrection, maxThrottleCorrection);
    }

    // Limit and apply
    if (isPitchAdjustmentValid && (navStateFlags & NAV_CTL_ALT)) {
        // PITCH angle is measured in opposite direction ( >0 - dive, <0 - climb)
        pitchCorrection = constrain(pitchCorrection, -DEGREES_TO_DECIDEGREES(posControl.navConfig->fw.max_dive_angle), DEGREES_TO_DECIDEGREES(posControl.navConfig->fw.max_climb_angle));
        rcCommand[PITCH] = -pidAngleToRcCommand(pitchCorrection, posControl.pidProfile->max_angle_inclination[FD_PITCH]);
    }

    if (isRollAdjustmentValid && (navStateFlags & NAV_CTL_POS)) {
        rollCorrection = constrain(rollCorrection, -DEGREES_TO_DECIDEGREES(posControl.navConfig->fw.max_bank_angle), DEGREES_TO_DECIDEGREES(posControl.navConfig->fw.max_bank_angle));
        rcCommand[ROLL] = pidAngleToRcCommand(rollCorrection, posControl.pidProfile->max_angle_inclination[FD_ROLL]);

        // Calculate coordinated turn rate based on velocity and banking angle
        if (posControl.actualState.velXY >= 300.0f) {
            float targetYawRateDPS = RADIANS_TO_DEGREES(tan_approx(DECIDEGREES_TO_RADIANS(-rollCorrection)) * GRAVITY_CMSS / posControl.actualState.velXY);
            rcCommand[YAW] = pidRateToRcCommand(targetYawRateDPS, currentControlRateProfile->rates[FD_YAW]);
        }
        else {
            rcCommand[YAW] = 0;
        }
    }

    if ((navStateFlags & NAV_CTL_ALT) || (navStateFlags & NAV_CTL_POS)) {
        uint16_t correctedThrottleValue = constrain(posControl.navConfig->fw.cruise_throttle + throttleCorrection, posControl.navConfig->fw.min_throttle, posControl.navConfig->fw.max_throttle);
        rcCommand[THROTTLE] = constrain(correctedThrottleValue, posControl.motorConfig->minthrottle, posControl.motorConfig->maxthrottle);
    }
}

/*-----------------------------------------------------------
 * FixedWing land detector
 *-----------------------------------------------------------*/
static timeUs_t landingTimerUs;

void resetFixedWingLandingDetector(void)
{
    landingTimerUs = micros();
}

bool isFixedWingLandingDetected(void)
{
    timeUs_t currentTimeUs = micros();

    landingTimerUs = currentTimeUs;
    return false;
}

/*-----------------------------------------------------------
 * FixedWing emergency landing
 *-----------------------------------------------------------*/
#define FW_EMERGENCY_DIVE_DECIDEG   100
#define FW_EMERGENCY_ROLL_DECIDEG   200
#define FW_EMERGENCY_YAW_RATE_DPS   20
void applyFixedWingEmergencyLandingController(void)
{
    // FIXME: Make this configurable, use altitude controller if available (similar to MC code)
    rcCommand[PITCH] = pidAngleToRcCommand(-FW_EMERGENCY_DIVE_DECIDEG, posControl.pidProfile->max_angle_inclination[FD_PITCH]);
    rcCommand[ROLL] = pidAngleToRcCommand(-FW_EMERGENCY_ROLL_DECIDEG, posControl.pidProfile->max_angle_inclination[FD_ROLL]);
    rcCommand[YAW] = pidRateToRcCommand(-FW_EMERGENCY_YAW_RATE_DPS, currentControlRateProfile->rates[FD_YAW]);
    rcCommand[THROTTLE] = posControl.navConfig->fw.min_throttle;
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
    updateMagHoldHeading(CENTIDEGREES_TO_DEGREES(posControl.actualState.yaw));
}

void applyFixedWingNavigationController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    if (navStateFlags & NAV_CTL_LAUNCH) {
        applyFixedWingLaunchController(currentTimeUs);
    }
    else if (navStateFlags & NAV_CTL_EMERG) {
        applyFixedWingEmergencyLandingController();
    }
    else {
#ifdef NAV_FW_LIMIT_MIN_FLY_VELOCITY
        // Don't apply anything if ground speed is too low (<3m/s)
        if (posControl.actualState.velXY > 300) {
            if (navStateFlags & NAV_CTL_ALT)
                applyFixedWingAltitudeController(currentTimeUs);

            if (navStateFlags & NAV_CTL_POS)
                applyFixedWingPositionController(currentTimeUs);
        }
        else {
            posControl.rcAdjustment[PITCH] = 0;
            posControl.rcAdjustment[ROLL] = 0;
        }
#else
        if (navStateFlags & NAV_CTL_ALT)
            applyFixedWingAltitudeController(currentTimeUs);

        if (navStateFlags & NAV_CTL_POS)
            applyFixedWingPositionController(currentTimeUs);
#endif

        //if (navStateFlags & NAV_CTL_YAW)
        if ((navStateFlags & NAV_CTL_ALT) || (navStateFlags & NAV_CTL_POS))
            applyFixedWingPitchRollThrottleController(navStateFlags, currentTimeUs);
    }
}

#endif  // NAV
