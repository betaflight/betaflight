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
#include <string.h>

#include "platform.h"

#if defined(NAV)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/rangefinder.h"
#include "sensors/barometer.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"

#include "io/gps.h"
#include "io/beeper.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"


typedef struct FixedWingLaunchState_s {
    /* Launch detection */
    timeUs_t launchDetectorPreviosUpdate;
    timeUs_t launchDetectionTimeAccum;
    bool launchDetected;

    /* Launch progress */
    timeUs_t launchStartedTime;
    bool launchFinished;
    bool motorControlAllowed;
} FixedWingLaunchState_t;

static FixedWingLaunchState_t   launchState;

#define COS_MAX_LAUNCH_ANGLE                0.70710678f                 // cos(45), just to be safe
#define SWING_LAUNCH_MIN_ROTATION_RATE      DEGREES_TO_RADIANS(100)     // expect minimum 100dps rotation rate
static void updateFixedWingLaunchDetector(timeUs_t currentTimeUs)
{
    const float swingVelocity = (ABS(imuMeasuredRotationBF.A[Z]) > SWING_LAUNCH_MIN_ROTATION_RATE) ? (imuMeasuredAccelBF.A[Y] / imuMeasuredRotationBF.A[Z]) : 0;
    const bool isForwardAccelerationHigh = (imuMeasuredAccelBF.A[X] > navConfig()->fw.launch_accel_thresh);
    const bool isAircraftAlmostLevel = (calculateCosTiltAngle() >= cos_approx(DEGREES_TO_RADIANS(navConfig()->fw.launch_max_angle)));

    const bool isBungeeLaunched = isForwardAccelerationHigh && isAircraftAlmostLevel;
    const bool isSwingLaunched = (swingVelocity > navConfig()->fw.launch_velocity_thresh) && (imuMeasuredAccelBF.A[X] > 0);

    if (isBungeeLaunched || isSwingLaunched) {
        launchState.launchDetectionTimeAccum += (currentTimeUs - launchState.launchDetectorPreviosUpdate);
        launchState.launchDetectorPreviosUpdate = currentTimeUs;
        if (launchState.launchDetectionTimeAccum >= MS2US((uint32_t)navConfig()->fw.launch_time_thresh)) {
            launchState.launchDetected = true;
        }
    }
    else {
        launchState.launchDetectorPreviosUpdate = currentTimeUs;
        launchState.launchDetectionTimeAccum = 0;
    }
}

void resetFixedWingLaunchController(timeUs_t currentTimeUs)
{
    launchState.launchDetectorPreviosUpdate = currentTimeUs;
    launchState.launchDetectionTimeAccum = 0;
    launchState.launchStartedTime = 0;
    launchState.launchDetected = false;
    launchState.launchFinished = false;
    launchState.motorControlAllowed = false;
}

bool isFixedWingLaunchDetected(void)
{
    return launchState.launchDetected;
}

void enableFixedWingLaunchController(timeUs_t currentTimeUs)
{
    launchState.launchStartedTime = currentTimeUs;
    launchState.motorControlAllowed = true;
}

bool isFixedWingLaunchFinishedOrAborted(void)
{
    return launchState.launchFinished;
}

static void applyFixedWingLaunchIdleLogic(void)
{
    // Until motors are started don't use PID I-term
    pidResetErrorAccumulators();

    // Throttle control logic
    if (navConfig()->fw.launch_idle_throttle <= motorConfig()->minthrottle) {
        ENABLE_STATE(NAV_MOTOR_STOP_OR_IDLE);                       // If MOTOR_STOP is enabled mixer will keep motor stopped
        rcCommand[THROTTLE] = motorConfig()->minthrottle;  // If MOTOR_STOP is disabled, motors will spin at minthrottle
    }
    else {
        rcCommand[THROTTLE] = navConfig()->fw.launch_idle_throttle;
    }
}

void applyFixedWingLaunchController(timeUs_t currentTimeUs)
{
    // Called at PID rate

    if (launchState.launchDetected) {
        bool applyLaunchIdleLogic = true;

        if (launchState.motorControlAllowed) {
            // If launch detected we are in launch procedure - control airplane
            const float timeElapsedSinceLaunchMs = US2MS(currentTimeUs - launchState.launchStartedTime);

            // If user moves the stick - finish the launch
            if ((ABS(rcCommand[ROLL]) > rcControlsConfig()->pos_hold_deadband) || (ABS(rcCommand[PITCH]) > rcControlsConfig()->pos_hold_deadband)) {
                launchState.launchFinished = true;
            }

            // Abort launch after a pre-set time
            if (timeElapsedSinceLaunchMs >= navConfig()->fw.launch_timeout) {
                launchState.launchFinished = true;
            }

            // Motor control enabled
            if (timeElapsedSinceLaunchMs >= navConfig()->fw.launch_motor_timer) {
                // Don't apply idle logic anymore
                applyLaunchIdleLogic = false;

                // Increase throttle gradually over `launch_motor_spinup_time` milliseconds
                if (navConfig()->fw.launch_motor_spinup_time > 0) {
                    const float timeSinceMotorStartMs = constrainf(timeElapsedSinceLaunchMs - navConfig()->fw.launch_motor_timer, 0.0f, navConfig()->fw.launch_motor_spinup_time);
                    const uint16_t minIdleThrottle = MAX(motorConfig()->minthrottle, navConfig()->fw.launch_idle_throttle);
                    rcCommand[THROTTLE] = scaleRangef(timeSinceMotorStartMs,
                                                      0.0f, navConfig()->fw.launch_motor_spinup_time,
                                                      minIdleThrottle, navConfig()->fw.launch_throttle);
                }
                else {
                    rcCommand[THROTTLE] = navConfig()->fw.launch_throttle;
                }
            }
        }

        if (applyLaunchIdleLogic) {
            // Launch idle logic - low throttle and zero out PIDs
            applyFixedWingLaunchIdleLogic();
        }
    }
    else {
        // We are waiting for launch - update launch detector
        updateFixedWingLaunchDetector(currentTimeUs);

        // Launch idle logic - low throttle and zero out PIDs
        applyFixedWingLaunchIdleLogic();
    }

    // Control beeper
    if (!launchState.launchFinished) {
        beeper(BEEPER_LAUNCH_MODE_ENABLED);
    }

    // Lock out controls
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = pidAngleToRcCommand(-DEGREES_TO_DECIDEGREES(navConfig()->fw.launch_climb_angle), pidProfile()->max_angle_inclination[FD_PITCH]);
    rcCommand[YAW] = 0;
}

#endif
